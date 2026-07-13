/*
  Proyecto UWB + IMU - etapa DW1000/BU01
  Version: UWB_DW1000_ESP32C3_Range_v0_3_2

  Objetivo:
  - Modo Range DS-TWR 1 TAG <-> 1 ANCHOR.
  - Consola Serial limpia:
      {"role":"tag","seq":12,"cm":32.40,"m":0.324}
      {"role":"anchor","seq":12,"cm":32.60,"m":0.326}
  - Debug disponible, pero apagado por defecto.
  - LED RGB:
      ANCHOR: ciclo de todos los colores.
      TAG:
        <= 20 cm  -> rojo
        20..50 cm -> magenta/intermedio
        > 50 cm   -> azul
        sin lectura reciente -> apagado

  Archivos requeridos:
  - dw1000_driver.h
  - dw1000_driver.cpp
  - dw1000_range.h
  - dw1000_range.cpp
*/

#include <Arduino.h>
#include <SPI.h>
#include "dw1000_range.h"

#define FW_NAME    "UWB_DW1000_ESP32C3_Range"
#define FW_VERSION "v0.3.2"

// =====================================================
// ESP32-C3 - MAPEO FINAL HARDWARE
// =====================================================

// BNO08X - IMU por I2C, reservado para siguiente etapa.
#define PIN_BNO_SDA      5
#define PIN_BNO_SCL      4
#define PIN_BNO_INT      6
#define PIN_BNO_RST     -1
#define BNO_I2C_ADDR    0x4A

// BU01 / DW1000 - UWB por SPI
#define PIN_UWB_CS       0
#define PIN_UWB_SCK      10
#define PIN_UWB_MOSI     21
#define PIN_UWB_MISO     3
#define PIN_UWB_IRQ      20
#define PIN_UWB_RST     -1
#define PIN_UWB_WAKEUP  -1

// LED RGB integrado en PCB
#define PIN_LED_B        7
#define PIN_LED_R        8
#define PIN_LED_G        9

// Boton integrado en PCB
#define PIN_BTN          1

// USB nativo
#define PIN_USB_DN       18
#define PIN_USB_DP       19

// =====================================================
// Ajuste LED
// =====================================================
// Si tu LED RGB fuera activo en bajo, cambia esto a 0.
#define LED_ACTIVE_HIGH 1

// =====================================================
// Seleccion de rol
// =====================================================
// Para TAG:
#define NODE_ROLE DW1000_ROLE_TAG

// Para ANCHOR, comentar la linea anterior y descomentar esta:
//#define NODE_ROLE DW1000_ROLE_ANCHOR

// =====================================================
// Configuracion DW1000
// =====================================================
static const uint16_t ANTENNA_DELAY = 16480;

static const uint32_t RANGE_PERIOD_MS = 700;
static const uint32_t RANGE_TIMEOUT_MS = 900;

static const uint32_t RESP_DELAY_UUS   = 12000;
static const uint32_t FINAL_DELAY_UUS  = 12000;
static const uint32_t RESULT_DELAY_UUS = 8000;

// Si no llega lectura en este tiempo, el TAG apaga el LED.
static const uint32_t DISTANCE_STALE_MS = 1600;

// Debug apagado por defecto para consola limpia.
static bool debugEnabled = false;
static bool outputEnabled = true;
static bool qualityOutputEnabled = true;

static const uint32_t DEBUG_PERIOD_MS = 1000;
static uint32_t lastDebugMs = 0;

// Objeto principal.
Dw1000Range dw1000Range;

// Calibracion software:
// corrected_cm = raw_cm * scale + offset_cm
float calibrationOffsetCm = 0.0f;
float calibrationScale = 1.0f;

// Ultima distancia para LED del TAG.
static float lastDistanceCm = 0.0f;
static uint32_t lastDistanceMs = 0;
static bool hasDistance = false;

// LED anchor rainbow.
static uint32_t lastAnchorLedMs = 0;
static uint8_t anchorColorIndex = 0;

// =====================================================
// Helpers
// =====================================================
static const char* roleName() {
  return NODE_ROLE == DW1000_ROLE_TAG ? "tag" : "anchor";
}

static void ledWritePin(uint8_t pin, bool on) {
#if LED_ACTIVE_HIGH
  digitalWrite(pin, on ? HIGH : LOW);
#else
  digitalWrite(pin, on ? LOW : HIGH);
#endif
}

static void setLed(bool r, bool g, bool b) {
  ledWritePin(PIN_LED_R, r);
  ledWritePin(PIN_LED_G, g);
  ledWritePin(PIN_LED_B, b);
}

static bool distanceIsRecent() {
  if (!hasDistance) return false;
  return millis() - lastDistanceMs <= DISTANCE_STALE_MS;
}

// =====================================================
// LED visual
// =====================================================
static void updateAnchorLed() {
  // Anchor fijo en verde para este ejemplo.
  setLed(false, true, false);
}

static void updateTagLed() {
  if (!distanceIsRecent()) {
    setLed(false, false, false);
    return;
  }

  if (lastDistanceCm <= 20.0f) {
    // Cerca: rojo.
    setLed(true, false, false);
  } else if (lastDistanceCm > 50.0f) {
    // Lejos: azul.
    setLed(false, false, true);
  } else {
    // Intermedio: mezcla rojo + azul.
    setLed(true, false, true);
  }
}

static void updateVisualLed() {
  if (NODE_ROLE == DW1000_ROLE_ANCHOR) {
    updateAnchorLed();
  } else {
    updateTagLed();
  }
}

// =====================================================
// Serial
// =====================================================
static void printHelp() {
  Serial.println();
  Serial.println("Comandos:");
  Serial.println("  h                  -> ayuda");
  Serial.println("  s                  -> status");
  Serial.println("  d                  -> ultima distancia");
  Serial.println("  r                  -> recover radio");
  Serial.println("  c <offset> <scale> -> calibracion software, ej: c -12.5 1.0");
  Serial.println("  a <delay>          -> antenna delay, ej: a 16480");
  Serial.println("  p                  -> imprimir pines");
  Serial.println("  v                  -> activar/desactivar debug");
  Serial.println("  o                  -> activar/desactivar salida de distancia");
  Serial.println("  q                  -> activar/desactivar salida de calidad RX");
  Serial.println();
}

static void printPins() {
  Serial.println("Pines:");
  Serial.printf("  UWB_CS=%d\n", PIN_UWB_CS);
  Serial.printf("  UWB_SCK=%d\n", PIN_UWB_SCK);
  Serial.printf("  UWB_MOSI=%d\n", PIN_UWB_MOSI);
  Serial.printf("  UWB_MISO=%d\n", PIN_UWB_MISO);
  Serial.printf("  UWB_IRQ=%d\n", PIN_UWB_IRQ);
  Serial.printf("  UWB_RST=%d\n", PIN_UWB_RST);
  Serial.printf("  BNO_SDA=%d BNO_SCL=%d BNO_INT=%d\n", PIN_BNO_SDA, PIN_BNO_SCL, PIN_BNO_INT);
  Serial.printf("  LED_R=%d LED_G=%d LED_B=%d BTN=%d\n", PIN_LED_R, PIN_LED_G, PIN_LED_B, PIN_BTN);
}

static void printStatus() {
  Dw1000RangeDebug dbg;
  dw1000Range.getDebugInfo(dbg);

  Serial.printf("FW=%s %s\n", FW_NAME, FW_VERSION);
  Serial.printf("role=%s\n", roleName());
  Serial.printf("status=%s\n", dw1000Range.getStatus());
  Serial.printf("device_id=0x%08lX\n", (unsigned long)dw1000Range.getDeviceId());
  Serial.printf("antenna_delay=%u\n", dw1000Range.getAntennaDelay());
  Serial.printf("spi_mode=%u spi_hz=%lu\n",
                dw1000Range.getSpiMode(),
                (unsigned long)dw1000Range.getSpiHz());

  Serial.printf("distance_cm=%.2f raw_cm=%.2f recent=%s\n",
                lastDistanceCm,
                dw1000Range.getRawDistanceCm(),
                distanceIsRecent() ? "true" : "false");

  Serial.printf("counters: tx=%lu rx=%lu rxe=%lu txe=%lu to=%lu\n",
                (unsigned long)dbg.txDoneCount,
                (unsigned long)dbg.rxDoneCount,
                (unsigned long)dbg.rxErrorCount,
                (unsigned long)dbg.txErrorCount,
                (unsigned long)dbg.timeoutCount);

  Serial.printf("poll_tx=%lu poll_rx=%lu resp_tx=%lu resp_rx=%lu final_tx=%lu final_rx=%lu result_tx=%lu result_rx=%lu\n",
                (unsigned long)dbg.pollTxCount,
                (unsigned long)dbg.pollRxCount,
                (unsigned long)dbg.respTxCount,
                (unsigned long)dbg.respRxCount,
                (unsigned long)dbg.finalTxCount,
                (unsigned long)dbg.finalRxCount,
                (unsigned long)dbg.resultTxCount,
                (unsigned long)dbg.resultRxCount);

  Serial.printf("cal_offset_cm=%.2f cal_scale=%.6f\n",
                calibrationOffsetCm,
                calibrationScale);
}

static void printLastDistance() {
  Serial.printf("{\"role\":\"%s\",\"cm\":%.2f,\"m\":%.3f,\"recent\":%s}\n",
                roleName(),
                lastDistanceCm,
                lastDistanceCm / 100.0f,
                distanceIsRecent() ? "true" : "false");
}

static void printDistanceReading(const Dw1000Reading& reading) {
  if (qualityOutputEnabled && reading.quality.valid) {
    Serial.printf(
      "{\"role\":\"%s\",\"seq\":%u,"
      "\"cm\":%.2f,\"m\":%.3f,"
      "\"rxp\":%.1f,\"fpp\":%.1f,\"gap\":%.1f,"
      "\"pacc\":%u,\"fpidx\":%.2f,"
      "\"cir\":%u,\"noise\":%u,"
      "\"f1\":%u,\"f2\":%u,\"f3\":%u}\n",
      roleName(),
      reading.sequence,
      reading.correctedCm,
      reading.correctedCm / 100.0f,
      reading.quality.rxPowerDbm,
      reading.quality.fpPowerDbm,
      reading.quality.powerDiffDb,
      reading.quality.rxPacc,
      reading.quality.fpIndex,
      reading.quality.cirPower,
      reading.quality.stdNoise,
      reading.quality.fpAmpl1,
      reading.quality.fpAmpl2,
      reading.quality.fpAmpl3
    );
  } else {
    Serial.printf(
      "{\"role\":\"%s\",\"seq\":%u,\"cm\":%.2f,\"m\":%.3f}\n",
      roleName(),
      reading.sequence,
      reading.correctedCm,
      reading.correctedCm / 100.0f
    );
  }
}

static void printAutoDebug() {
  if (!debugEnabled) return;
  if (millis() - lastDebugMs < DEBUG_PERIOD_MS) return;

  lastDebugMs = millis();

  Dw1000RangeDebug dbg;
  dw1000Range.getDebugInfo(dbg);

  Serial.printf(
    "{\"type\":\"debug\",\"role\":\"%s\",\"ms\":%lu,"
    "\"status\":\"%s\",\"tx\":%lu,\"rx\":%lu,\"rxe\":%lu,\"txe\":%lu,\"to\":%lu,"
    "\"poll_tx\":%lu,\"poll_rx\":%lu,"
    "\"resp_tx\":%lu,\"resp_rx\":%lu,"
    "\"final_tx\":%lu,\"final_rx\":%lu,"
    "\"result_tx\":%lu,\"result_rx\":%lu,"
    "\"cm\":%.2f,\"recent\":%s}\n",
    roleName(),
    (unsigned long)millis(),
    dbg.status,
    (unsigned long)dbg.txDoneCount,
    (unsigned long)dbg.rxDoneCount,
    (unsigned long)dbg.rxErrorCount,
    (unsigned long)dbg.txErrorCount,
    (unsigned long)dbg.timeoutCount,
    (unsigned long)dbg.pollTxCount,
    (unsigned long)dbg.pollRxCount,
    (unsigned long)dbg.respTxCount,
    (unsigned long)dbg.respRxCount,
    (unsigned long)dbg.finalTxCount,
    (unsigned long)dbg.finalRxCount,
    (unsigned long)dbg.resultTxCount,
    (unsigned long)dbg.resultRxCount,
    lastDistanceCm,
    distanceIsRecent() ? "true" : "false"
  );
}

// =====================================================
// Configuracion DW1000
// =====================================================
static void fillDw1000Config(Dw1000RangeConfig& cfg) {
  cfg.role = NODE_ROLE;

  cfg.pins.cs = PIN_UWB_CS;
  cfg.pins.irq = PIN_UWB_IRQ;
  cfg.pins.rst = PIN_UWB_RST;
  cfg.pins.sck = PIN_UWB_SCK;
  cfg.pins.miso = PIN_UWB_MISO;
  cfg.pins.mosi = PIN_UWB_MOSI;

  cfg.radio.channel = 5;
  cfg.radio.preambleCode = 9;
  cfg.radio.preambleLength = 128;
  cfg.radio.antennaDelay = ANTENNA_DELAY;

  cfg.radio.initSpiHz = 1000000UL;
  cfg.radio.spiHz = 2000000UL;
  cfg.radio.spiMode = SPI_MODE0;
  cfg.radio.autoDetectSpi = true;

  cfg.rangePeriodMs = RANGE_PERIOD_MS;
  cfg.timeoutMs = RANGE_TIMEOUT_MS;

  cfg.respDelayUus = RESP_DELAY_UUS;
  cfg.finalDelayUus = FINAL_DELAY_UUS;
  cfg.resultDelayUus = RESULT_DELAY_UUS;
}

// =====================================================
// Comandos Serial
// =====================================================
static void handleSerialCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);

  if (cmd == 'h') {
    printHelp();

  } else if (cmd == 's') {
    printStatus();

  } else if (cmd == 'd') {
    printLastDistance();

  } else if (cmd == 'r') {
    Serial.println("Recover radio...");
    dw1000Range.recover();

  } else if (cmd == 'p') {
    printPins();

  } else if (cmd == 'v') {
    debugEnabled = !debugEnabled;
    Serial.printf("debug=%s\n", debugEnabled ? "true" : "false");

  } else if (cmd == 'o') {
    outputEnabled = !outputEnabled;
    Serial.printf("output=%s\n", outputEnabled ? "true" : "false");

  } else if (cmd == 'c') {
    float offset = calibrationOffsetCm;
    float scale = calibrationScale;

    int parsed = sscanf(line.c_str(), "c %f %f", &offset, &scale);

    if (parsed >= 1) {
      calibrationOffsetCm = offset;

      if (parsed >= 2) {
        calibrationScale = scale;
      }

      dw1000Range.setCalibration(calibrationOffsetCm, calibrationScale);

      Serial.printf("Calibration set: offset_cm=%.2f scale=%.6f\n",
                    calibrationOffsetCm,
                    calibrationScale);
    } else {
      Serial.println("Uso: c <offsetCm> <scale>");
    }

  } else if (cmd == 'a') {
    int delayValue = -1;
    int parsed = sscanf(line.c_str(), "a %d", &delayValue);

    if (parsed == 1 && delayValue >= 0 && delayValue <= 65535) {
      dw1000Range.setAntennaDelay((uint16_t)delayValue);

      Serial.printf("Antenna delay set: %u\n",
                    dw1000Range.getAntennaDelay());
    } else {
      Serial.println("Uso: a <0..65535>");
    }

  } else if (cmd == 'q') {
    qualityOutputEnabled = !qualityOutputEnabled;
    Serial.printf("quality_output=%s\n", qualityOutputEnabled ? "true" : "false");
  } else {
    Serial.println("Comando no reconocido. Usa h para ayuda.");
  }
}

// =====================================================
// Setup / Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);
  delay(1800);

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);

  setLed(false, false, true);

  Serial.println();
  Serial.printf("%s %s\n", FW_NAME, FW_VERSION);
  Serial.println("DW1000/BU01 Range DS-TWR - consola limpia.");
  Serial.printf("role=%s\n", roleName());

  Dw1000RangeConfig cfg;
  fillDw1000Config(cfg);

  if (!dw1000Range.begin(cfg, SPI)) {
    setLed(true, false, false);

    Serial.println("ERROR: No se pudo inicializar DW1000/BU01.");
    Serial.println("Revisa 3V3, GND, CS, SCK, MOSI, MISO e IRQ.");
    Serial.printf("device_id leido: 0x%08lX\n",
                  (unsigned long)dw1000Range.getLastDeviceId());

    while (true) {
      delay(500);
      ledWritePin(PIN_LED_R, !digitalRead(PIN_LED_R));
    }
  }

  dw1000Range.setCalibration(calibrationOffsetCm, calibrationScale);

  setLed(false, true, false);

  printStatus();
  printHelp();
}

void loop() {
  dw1000Range.update();

  Dw1000Reading reading;
  if (dw1000Range.read(reading)) {
    lastDistanceCm = reading.correctedCm;
    lastDistanceMs = millis();
    hasDistance = true;

    if (outputEnabled) {
      printDistanceReading(reading);
    }
  }

  handleSerialCommand();
  printAutoDebug();
  updateVisualLed();
}