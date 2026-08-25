#include <Arduino.h>
#include "PulseAPDS9008.h"

// ============================================================
// HARDWARE
// ============================================================

#define PULSE_PIN      2
#define PULSE_LED_PIN  0

// ============================================================
// CONFIGURACION GENERAL
// ============================================================

const uint8_t PULSE_LED_POWER = 255;

// 10 ms = 100 Hz.
const uint16_t PULSE_SAMPLE_INTERVAL_MS = 10;

// Guardrail fisiológico amplio.
// NO representa un BPM esperado.
const uint16_t PULSE_MIN_BPM = 35;
const uint16_t PULSE_MAX_BPM = 220;

// Piso mínimo AC.
const float PULSE_MIN_AMPLITUDE = 12.0f;

// Estado general cada 500 ms.
const uint32_t SERIAL_STATUS_INTERVAL_MS = 500;

// ============================================================
// SALIDA SERIAL
// ============================================================
//
// false -> Monitor Serial normal.
// true  -> Arduino Serial Plotter.
//
// IMPORTANTE:
// Cuando SERIAL_PLOTTER_MODE=true no se imprimen textos de
// diagnóstico, sólo series numéricas compatibles con Plotter.
const bool SERIAL_PLOTTER_MODE = true;

// Desplazamiento visual para superponer la señal AC alrededor
// del mismo nivel del ADC RAW/DC sin usar valores negativos.
const float PLOTTER_AC_CENTER = 2048.0f;

// ============================================================
// SENSOR
// ============================================================

PulseAPDS9008 pulseSensor;

// ============================================================
// CONFIGURACION DEL SENSOR
// ============================================================

void configurePulseSensor() {
  PulseAPDS9008Config cfg;

  cfg.signalPin =
    PULSE_PIN;

  cfg.ledPin =
    PULSE_LED_PIN;

  cfg.sampleIntervalMs =
    PULSE_SAMPLE_INTERVAL_MS;

  cfg.ledPower =
    PULSE_LED_POWER;

  cfg.autoLedControl =
    false;

  // --------------------------------------------------------
  // GUARDRAIL FISIOLOGICO
  // --------------------------------------------------------

  cfg.minBpm =
    PULSE_MIN_BPM;

  cfg.maxBpm =
    PULSE_MAX_BPM;

  cfg.warmupMs =
    1200;

  cfg.minPulseAmplitude =
    PULSE_MIN_AMPLITUDE;

  // --------------------------------------------------------
  // FILTRADO
  // --------------------------------------------------------

  cfg.dcAlpha =
    0.005f;

  cfg.signalAlpha =
    0.25f;

  cfg.envelopeAlpha =
    0.035f;

  cfg.noiseAlpha =
    0.06f;

  // --------------------------------------------------------
  // DETECTOR DE PULSO
  // --------------------------------------------------------

  cfg.envelopeThresholdFactor =
    0.35f;

  cfg.noiseThresholdFactor =
    3.5f;

  cfg.releaseFactor =
    0.25f;

  cfg.prominenceFactor =
    1.15f;

  cfg.noiseProminenceFactor =
    4.0f;

  cfg.peakFallNoiseFactor =
    1.25f;

  cfg.minPeakFall =
    3.0f;

  cfg.fallingSamplesToConfirm =
    2;

  cfg.candidateTimeoutMs =
    280;

  // --------------------------------------------------------
  // CONTACTO OPTICO
  // --------------------------------------------------------
  //
  // Más estricto que la versión básica anterior.
  // No basta con tener envelope: también se exige buena
  // relación señal/ruido.

  cfg.contactAdcMin =
    80;

  cfg.contactAdcMax =
    4015;

  cfg.contactEnvelopeFactor =
    0.35f;

  cfg.contactMinEnvelope =
    8.0f;

  cfg.contactMinSnr =
    3.5f;

  cfg.contactMaxNoiseRatio =
    0.30f;

  cfg.contactConfirmMs =
    850;

  cfg.contactLostMs =
    1300;

  // --------------------------------------------------------
  // MOVIMIENTO
  // --------------------------------------------------------

  cfg.motionRawJumpFloor =
    90.0f;

  cfg.motionAcFloor =
    260.0f;

  cfg.motionNoiseRatio =
    0.55f;

  cfg.motionConfirmSamples =
    2;

  cfg.motionHoldMs =
    700;

  // --------------------------------------------------------
  // ADQUISICION
  // --------------------------------------------------------

  cfg.acquisitionSettleMs =
    450;

  cfg.acquisitionMinIbi =
    4;

  cfg.acquisitionTolerance =
    0.22f;

  // Si el cluster sugiere >=120 BPM se exige más evidencia.
  cfg.highRateBpmThreshold =
    120;

  cfg.highRateAcquisitionMinIbi =
    5;

  // --------------------------------------------------------
  // TRACKING
  // --------------------------------------------------------

  cfg.trackingIbiTolerance =
    0.25f;

  cfg.dynamicRefractoryFactor =
    0.52f;

  // --------------------------------------------------------
  // REARME MORFOLOGICO
  // --------------------------------------------------------
  //
  // Después de un pico, no se permite buscar otro hasta
  // que la AC atraviese un valle negativo suficiente.

  cfg.beatRearmEnabled =
    true;

  cfg.beatRearmMinAc =
    4.0f;

  cfg.beatRearmNoiseFactor =
    1.0f;

  // --------------------------------------------------------
  // RE-ADQUISICION
  // --------------------------------------------------------

  cfg.reacquireMinIbi =
    3;

  cfg.reacquireTolerance =
    0.22f;

  cfg.maxConsecutiveOutliers =
    5;

  cfg.signalHoldMs =
    1800;

  pulseSensor.begin(cfg);
}

// ============================================================
// PRINT DE EVENTO DE PULSO
// ============================================================

void printBeat(
  const PulseAPDS9008Data& p
) {
  Serial.printf(
    "[PULSE] #%lu | state=%s | bpm=%u | ibi=%u ms | accepted=%s | "
    "raw=%u | peak=%.1f | prom=%.1f | snr=%.2f | rearm=%s\n",
    (unsigned long)p.beatCount,
    pulseSensor.stateName(),
    p.bpm,
    p.ibiMs,
    p.ibiAccepted ? "YES" : "NO",
    p.raw,
    p.peak,
    p.prominence,
    p.contactSnr,
    p.beatRearmed ? "YES" : "NO"
  );
}

// ============================================================
// PRINT PERIODICO
// ============================================================

void printStatus(
  const PulseAPDS9008Data& p
) {
  Serial.printf(
    "[PULSE-SIGNAL] "
    "state=%s | contact=%s | motion=%s | "
    "raw=%u | dc=%.1f | ac=%+.1f | env=%.1f | noise=%.1f | "
    "snr=%.2f | rearm=%s | bpm=%u | stable=%s | "
    "quality=%u | confidence=%u | led=%u\n",
    pulseSensor.stateName(),
    p.contactLikely ? "YES" : "NO",
    p.motionDetected ? "YES" : "NO",
    p.raw,
    p.dc,
    p.ac,
    p.envelope,
    p.noise,
    p.contactSnr,
    p.beatRearmed ? "YES" : "NO",
    p.bpm,
    p.bpmStable ? "YES" : "NO",
    p.quality,
    p.confidence,
    p.ledPower
  );
}

// ============================================================
// SERIAL PLOTTER
// ============================================================
//
// Series:
//   RAW       -> ADC real.
//   DC        -> baseline estimada.
//   AC        -> componente pulsátil filtrada, centrada en 2048.
//   ENV       -> envolvente, centrada en 2048.
//   THR_POS   -> threshold positivo de detección.
//   THR_NEG   -> espejo negativo del threshold para referencia.
//   REARM     -> valle negativo mínimo requerido para rearmar.
//
// El detector inicia un candidato cuando AC supera THR_POS.
// Después de un pico, el siguiente ciclo queda habilitado cuando
// AC cae por debajo de REARM.
//
// Todas las trazas AC/ENV/THR/REARM se desplazan alrededor de
// PLOTTER_AC_CENTER sólo para facilitar la visualización.

void printPlotter(
  const PulseAPDS9008Data& p
) {
  float rearmMagnitude =
    p.noise;

  if (rearmMagnitude < 4.0f) {
    rearmMagnitude = 4.0f;
  }

  float acPlot =
    PLOTTER_AC_CENTER + p.ac;

  float envPlot =
    PLOTTER_AC_CENTER + p.envelope;

  float thresholdPositive =
    PLOTTER_AC_CENTER + p.threshold;

  float thresholdNegative =
    PLOTTER_AC_CENTER - p.threshold;

  float rearmLevel =
    PLOTTER_AC_CENTER - rearmMagnitude;

  Serial.print("RAW:");
  Serial.print(p.raw);

  Serial.print(",DC:");
  Serial.print(p.dc, 1);

  Serial.print(",AC:");
  Serial.print(acPlot, 1);

  Serial.print(",ENV:");
  Serial.print(envPlot, 1);

  Serial.print(",THR_POS:");
  Serial.print(thresholdPositive, 1);

  Serial.print(",THR_NEG:");
  Serial.print(thresholdNegative, 1);

  Serial.print(",REARM:");
  Serial.print(rearmLevel, 1);

  Serial.println();
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  delay(1000);

  analogReadResolution(12);

  configurePulseSensor();

  if (!SERIAL_PLOTTER_MODE) {
    Serial.println();
    Serial.println("==============================================================");
    Serial.println(" APDS-9008 BASIC EXAMPLE v0.2.1 PLOTTER");
    Serial.println("==============================================================");

    Serial.printf(
      "ADC APDS-9008       : GPIO%d\n",
      PULSE_PIN
    );

    Serial.printf(
      "LED verde           : GPIO%d\n",
      PULSE_LED_PIN
    );

    Serial.printf(
      "LED power           : %u / 255\n",
      PULSE_LED_POWER
    );

    Serial.printf(
      "Sampling            : %u Hz\n",
      1000 / PULSE_SAMPLE_INTERVAL_MS
    );

    Serial.printf(
      "BPM guardrail       : %u - %u\n",
      PULSE_MIN_BPM,
      PULSE_MAX_BPM
    );

    Serial.println(
      "Contact SNR minimo  : 3.5"
    );

    Serial.println(
      "IBI adquisicion     : 4"
    );

    Serial.println(
      "IBI si BPM >=120    : 5"
    );

    Serial.println(
      "Tracking tolerance  : +/-25%"
    );

    Serial.println(
      "Serial Plotter mode : OFF"
    );

    Serial.println();
    Serial.println(
      "Estados: NO_CONTACT / ACQUIRING / TRACKING / MOTION"
    );

    Serial.println(
      "Espere TRACKING + stable=YES para considerar BPM adquirido."
    );

    Serial.println("==============================================================");
    Serial.println();
  }
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  if (!pulseSensor.update()) {
    return;
  }

  const PulseAPDS9008Data& p =
    pulseSensor.data();

  // --------------------------------------------------------
  // SERIAL PLOTTER
  // --------------------------------------------------------
  //
  // El plotter recibe una línea en cada nueva muestra (~100 Hz)
  // para conservar correctamente la forma de onda.
  if (SERIAL_PLOTTER_MODE) {
    printPlotter(p);
    return;
  }

  // --------------------------------------------------------
  // MONITOR SERIAL
  // --------------------------------------------------------

  if (p.beat) {
    printBeat(p);
  }

  static uint32_t lastStatusMs = 0;

  if (
    millis() - lastStatusMs >=
    SERIAL_STATUS_INTERVAL_MS
  ) {
    lastStatusMs =
      millis();

    printStatus(p);
  }
}
