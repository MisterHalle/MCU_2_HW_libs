/*
  TFT_ST7735_ESP32S3_multiinit_diag_v1.1

  Diagnóstico para TFT SPI 1.77" con ESP32-S3.
  Prueba varias inicializaciones ST7735 a baja velocidad SPI.

  Librerías:
  - Adafruit_GFX
  - Adafruit_ST7735
  - Adafruit_BusIO

  Conexión típica:
  TFT VCC       -> 3V3
  TFT GND       -> GND
  TFT SCL/SCK   -> TFT_SCLK
  TFT SDA/MOSI  -> TFT_MOSI
  TFT CS        -> TFT_CS
  TFT DC/A0/RS  -> TFT_DC
  TFT RES/RST   -> TFT_RST
  TFT LED/BL    -> TFT_BL o 3V3
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// Pines recomendados temporalmente.
// Ajusta si tu PCB custom usa otros.
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS   10
#define TFT_DC    9
#define TFT_RST  14
#define TFT_BL   13

// Hardware SPI con pines definidos manualmente
Adafruit_ST7735 tft = Adafruit_ST7735(&SPI, TFT_CS, TFT_DC, TFT_RST);

struct InitMode {
  const char* name;
  uint8_t mode;
};

InitMode modes[] = {
  {"INITR_BLACKTAB", INITR_BLACKTAB},
  {"INITR_GREENTAB", INITR_GREENTAB},
  {"INITR_REDTAB",   INITR_REDTAB},
  {"INITR_144GREENTAB", INITR_144GREENTAB}
};

const uint8_t modeCount = sizeof(modes) / sizeof(modes[0]);

void hardResetDisplay() {
  Serial.println("Hard reset TFT...");

  pinMode(TFT_RST, OUTPUT);

  digitalWrite(TFT_RST, HIGH);
  delay(100);

  digitalWrite(TFT_RST, LOW);
  delay(200);

  digitalWrite(TFT_RST, HIGH);
  delay(500);
}

void basicColorTest(const char* label) {
  Serial.print("Mostrando colores para: ");
  Serial.println(label);

  tft.fillScreen(ST77XX_RED);
  delay(1000);

  tft.fillScreen(ST77XX_GREEN);
  delay(1000);

  tft.fillScreen(ST77XX_BLUE);
  delay(1000);

  tft.fillScreen(ST77XX_BLACK);
  delay(300);

  tft.setRotation(1);

  tft.fillScreen(ST77XX_BLACK);
  delay(200);

  tft.drawRect(0, 0, tft.width(), tft.height(), ST77XX_WHITE);

  tft.setCursor(5, 5);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.println("ESP32-S3 TFT");

  tft.setCursor(5, 20);
  tft.setTextColor(ST77XX_GREEN);
  tft.println(label);

  tft.setCursor(5, 40);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("W:");
  tft.print(tft.width());
  tft.print(" H:");
  tft.println(tft.height());

  tft.fillRect(10, 65, 30, 30, ST77XX_RED);
  tft.fillRect(50, 65, 30, 30, ST77XX_GREEN);
  tft.fillRect(90, 65, 30, 30, ST77XX_BLUE);

  delay(3000);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("=================================");
  Serial.println("TFT_ST7735_ESP32S3_multiinit_diag_v1.1");
  Serial.println("=================================");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_MOSI, OUTPUT);
  pinMode(TFT_SCLK, OUTPUT);

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  Serial.println("Backlight ON");

  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  // Muy lento para descartar problemas de integridad de señal.
  tft.setSPISpeed(1000000);

  Serial.println("SPI iniciado a 1 MHz");
}

void loop() {
  for (uint8_t i = 0; i < modeCount; i++) {
    Serial.println();
    Serial.print("Probando modo: ");
    Serial.println(modes[i].name);

    hardResetDisplay();

    tft.initR(modes[i].mode);

    // También prueba inversión apagada/encendida dentro del mismo modo
    tft.invertDisplay(false);
    basicColorTest(modes[i].name);

    Serial.print("Probando inversion con: ");
    Serial.println(modes[i].name);

    tft.invertDisplay(true);
    basicColorTest("INVERT ON");

    tft.invertDisplay(false);
  }

  Serial.println();
  Serial.println("Ciclo completo. Repitiendo pruebas...");
  delay(2000);
}