/*
  APDS9008_BASIC_EXAMPLE_v0.1.0

  Ejemplo básico para ESP32-C3 + APDS-9008.

  GPIO2 -> señal analógica APDS-9008
  GPIO0 -> LED verde del sensor

  Sin WiFi, NodeDiscovery, TCP, BQ27427, NVS,
  botones ni deep sleep.
*/

#include <Arduino.h>
#include "PulseAPDS9008.h"

#define PULSE_PIN      2
#define PULSE_LED_PIN  0

const uint8_t PULSE_LED_POWER = 255;
const uint16_t PULSE_SAMPLE_INTERVAL_MS = 10;  // 100 Hz

// Guardrail fisiológico amplio.
// No representa un BPM esperado.
const uint16_t PULSE_MIN_BPM = 35;
const uint16_t PULSE_MAX_BPM = 220;

const float PULSE_MIN_AMPLITUDE = 12.0f;
const uint32_t SERIAL_STATUS_INTERVAL_MS = 500;

PulseAPDS9008 pulseSensor;

void configurePulseSensor() {
  PulseAPDS9008Config cfg;

  cfg.signalPin = PULSE_PIN;
  cfg.ledPin = PULSE_LED_PIN;
  cfg.sampleIntervalMs = PULSE_SAMPLE_INTERVAL_MS;

  cfg.ledPower = PULSE_LED_POWER;
  cfg.autoLedControl = false;

  cfg.minBpm = PULSE_MIN_BPM;
  cfg.maxBpm = PULSE_MAX_BPM;
  cfg.warmupMs = 1200;

  cfg.minPulseAmplitude = PULSE_MIN_AMPLITUDE;

  // Filtros.
  cfg.dcAlpha = 0.005f;
  cfg.signalAlpha = 0.25f;
  cfg.envelopeAlpha = 0.035f;
  cfg.noiseAlpha = 0.06f;

  // Detector de pulso.
  cfg.envelopeThresholdFactor = 0.35f;
  cfg.noiseThresholdFactor = 3.5f;
  cfg.releaseFactor = 0.25f;

  cfg.prominenceFactor = 1.15f;
  cfg.noiseProminenceFactor = 4.0f;
  cfg.peakFallNoiseFactor = 1.25f;
  cfg.minPeakFall = 3.0f;
  cfg.fallingSamplesToConfirm = 2;
  cfg.candidateTimeoutMs = 280;

  // Contacto.
  cfg.contactAdcMin = 80;
  cfg.contactAdcMax = 4015;
  cfg.contactEnvelopeFactor = 0.35f;
  cfg.contactConfirmMs = 450;
  cfg.contactLostMs = 2200;

  // Movimiento.
  cfg.motionRawJumpFloor = 90.0f;
  cfg.motionAcFloor = 260.0f;
  cfg.motionNoiseRatio = 0.55f;
  cfg.motionConfirmSamples = 2;
  cfg.motionHoldMs = 700;

  // Adquisición.
  cfg.acquisitionSettleMs = 450;
  cfg.acquisitionMinIbi = 3;
  cfg.acquisitionTolerance = 0.22f;

  // Tracking.
  cfg.trackingIbiTolerance = 0.32f;
  cfg.dynamicRefractoryFactor = 0.52f;

  // Re-adquisición.
  cfg.reacquireMinIbi = 3;
  cfg.reacquireTolerance = 0.22f;
  cfg.maxConsecutiveOutliers = 5;

  cfg.signalHoldMs = 1800;

  pulseSensor.begin(cfg);
}

void printBeat(const PulseAPDS9008Data& p) {
  Serial.printf(
    "PULSO #%lu | estado=%s | BPM=%u | IBI=%u ms | aceptado=%s | peak=%.1f | prom=%.1f\n",
    (unsigned long)p.beatCount,
    pulseSensor.stateName(),
    p.bpm,
    p.ibiMs,
    p.ibiAccepted ? "SI" : "NO",
    p.peak,
    p.prominence
  );
}

void printStatus(const PulseAPDS9008Data& p) {
  Serial.printf(
    "raw=%u | dc=%.1f | ac=%+.1f | env=%.1f | noise=%.1f | "
    "estado=%s | contacto=%s | movimiento=%s | bpm=%u | "
    "estable=%s | calidad=%u | confianza=%u\n",
    p.raw,
    p.dc,
    p.ac,
    p.envelope,
    p.noise,
    pulseSensor.stateName(),
    p.contactLikely ? "SI" : "NO",
    p.motionDetected ? "SI" : "NO",
    p.bpm,
    p.bpmStable ? "SI" : "NO",
    p.quality,
    p.confidence
  );
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);

  configurePulseSensor();

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" APDS-9008 BASIC EXAMPLE v0.1.0");
  Serial.println("==============================================");
  Serial.printf("ADC APDS-9008 : GPIO%d\n", PULSE_PIN);
  Serial.printf("LED verde     : GPIO%d\n", PULSE_LED_PIN);
  Serial.printf("LED power     : %u / 255\n", PULSE_LED_POWER);
  Serial.printf("Sampling      : %u Hz\n", 1000 / PULSE_SAMPLE_INTERVAL_MS);
  Serial.printf("BPM guardrail : %u - %u\n", PULSE_MIN_BPM, PULSE_MAX_BPM);
  Serial.println();
  Serial.println("Coloque el sensor sobre la zona de medicion.");
  Serial.println("Espere a que el estado pase a TRACKING.");
  Serial.println("==============================================");
  Serial.println();
}

void loop() {
  if (!pulseSensor.update()) {
    return;
  }

  const PulseAPDS9008Data& p = pulseSensor.data();

  if (p.beat) {
    printBeat(p);
  }

  static uint32_t lastStatusMs = 0;

  if (millis() - lastStatusMs >= SERIAL_STATUS_INTERVAL_MS) {
    lastStatusMs = millis();
    printStatus(p);
  }
}