/*
  APDS9008_HYBRID_3METHOD_v0.5.3 COOPERATIVE-HARMONIC

  Experimental:
  1) Peak / IBI
  2) Autocorrelacion
  3) Analisis espectral Fourier/Goertzel
  4) Fusion 2-de-3

  IMPORTANTE:
  - bpm legado sigue siendo el metodo IBI.
  - bpmFused es experimental.
  - No hay WiFi / TCP / NodeDiscovery en este ejemplo.
*/

#include <Arduino.h>
#include "PulseAPDS9008.h"

#define PULSE_PIN      2
#define PULSE_LED_PIN  0

const uint8_t PULSE_LED_POWER = 255;
const uint16_t PULSE_SAMPLE_INTERVAL_MS = 10;

// 0 = Monitor Serial
// 1 = Plotter señal
// 2 = Plotter BPM/metodos
const uint8_t SERIAL_OUTPUT_MODE = 1;

const uint32_t SERIAL_STATUS_INTERVAL_MS = 500;

// El sensor sigue muestreando a 100 Hz. Solo reducimos la
// cantidad de texto enviado al Plotter para evitar bloquear UART.
const uint8_t SIGNAL_PLOTTER_DECIMATION = 2;   // ~50 Hz visible
const uint32_t BPM_PLOTTER_INTERVAL_MS = 100;  // 10 Hz visible

const float PLOT_CENTER = 2048.0f;

PulseAPDS9008 pulseSensor;

void configurePulseSensor() {
  PulseAPDS9008Config cfg;

  cfg.signalPin = PULSE_PIN;
  cfg.ledPin = PULSE_LED_PIN;
  cfg.sampleIntervalMs = PULSE_SAMPLE_INTERVAL_MS;

  cfg.ledPower = PULSE_LED_POWER;
  cfg.autoLedControl = false;

  // Baseline CONTACT-GUARD validada.
  cfg.minBpm = 35;
  cfg.maxBpm = 220;
  cfg.warmupMs = 1200;
  cfg.minPulseAmplitude = 12.0f;

  cfg.dcAlpha = 0.005f;
  cfg.signalAlpha = 0.25f;
  cfg.envelopeAlpha = 0.035f;
  cfg.noiseAlpha = 0.06f;

  cfg.envelopeThresholdFactor = 0.35f;
  cfg.noiseThresholdFactor = 3.5f;
  cfg.releaseFactor = 0.25f;

  cfg.prominenceFactor = 1.15f;
  cfg.noiseProminenceFactor = 4.0f;
  cfg.peakFallNoiseFactor = 1.25f;
  cfg.minPeakFall = 3.0f;
  cfg.fallingSamplesToConfirm = 2;
  cfg.candidateTimeoutMs = 280;

  cfg.contactAdcMin = 80;
  cfg.contactAdcMax = 4015;
  cfg.contactEnvelopeFactor = 0.35f;
  cfg.contactMinEnvelope = 8.0f;
  cfg.contactMinSnr = 3.5f;
  cfg.contactMaxNoiseRatio = 0.30f;
  cfg.contactConfirmMs = 850;
  cfg.contactLostMs = 1300;

  cfg.motionRawJumpFloor = 90.0f;
  cfg.motionAcFloor = 260.0f;
  cfg.motionNoiseRatio = 0.55f;
  cfg.motionConfirmSamples = 2;
  cfg.motionHoldMs = 700;

  cfg.acquisitionSettleMs = 450;
  cfg.acquisitionMinIbi = 4;
  cfg.acquisitionTolerance = 0.22f;

  cfg.highRateBpmThreshold = 120;
  cfg.highRateAcquisitionMinIbi = 5;

  cfg.trackingIbiTolerance = 0.25f;
  cfg.dynamicRefractoryFactor = 0.52f;

  cfg.beatRearmEnabled = true;
  cfg.beatRearmMinAc = 4.0f;
  cfg.beatRearmNoiseFactor = 1.0f;

  cfg.reacquireMinIbi = 3;
  cfg.reacquireTolerance = 0.22f;
  cfg.maxConsecutiveOutliers = 5;

  cfg.signalHoldMs = 1800;

  // ========================================================
  // SISTEMA REDUNDANTE
  // ========================================================

  cfg.hybridEnabled = true;

  // 8 segundos maximo, comienza a estimar despues de 5.
  cfg.hybridWindowMs = 8000;
  cfg.hybridMinWindowMs = 5000;
  cfg.hybridUpdateMs = 1000;

  // Analisis cooperativo: no bloquear el muestreo de 10 ms.
  cfg.hybridSliceBudgetUs = 1800;

  // Barrido espectral grueso de 2 BPM + interpolacion.
  cfg.hybridSpectralStepBpm = 2;

  // ADC/IBI = 100 Hz.
  // Analizador CORR/SPEC = 25 Hz despues del filtro PPG.
  cfg.hybridDecimation = 4;

  // Banda PPG.
  cfg.ppgHighpassHz = 0.60f;
  cfg.ppgLowpassHz = 4.00f;

  cfg.autocorrMinQuality = 35;
  cfg.spectralMinQuality = 35;

  cfg.fusionToleranceFraction = 0.10f;
  cfg.fusionToleranceBpm = 6;
  cfg.fusionMinConfidence = 55;

  pulseSensor.begin(cfg);
}

void printBeat(
  const PulseAPDS9008Data& p
) {
  Serial.printf(
    "[PULSE] #%lu state=%s bpmIBI=%u ibi=%u accepted=%s "
    "peak=%.1f prom=%.1f\n",
    (unsigned long)p.beatCount,
    pulseSensor.stateName(),
    p.bpmIbi,
    p.ibiMs,
    p.ibiAccepted ? "YES" : "NO",
    p.peak,
    p.prominence
  );
}

const char* relationName(uint8_t relation) {
  switch (relation) {
    case 1: return "0.5x";
    case 2: return "1x";
    case 3: return "2x";
    default: return "--";
  }
}

void printHybrid(
  const PulseAPDS9008Data& p
) {
  Serial.printf(
    "[HYBRID] "
    "IBI=%u Q=%u REL=%s | "
    "CORR=%u Q=%u R=%.2f REL=%s | "
    "SPEC=%u RAW=%u Q=%u DOM=%.1f H2=%.2f REL=%s | "
    "FUSED=%u Q=%u METHODS=%u VALID=%s HARMONIC=%s RESOLVED=%s SCORE=%.1f "
    "SAMPLES=%u CPU=%luus MAXCPU=%luus SLICE=%uus MAXSLICE=%uus CYCLE=%ums GAPMAX=%ums\n",

    p.bpmIbi,
    p.ibiMethodQuality,
    relationName(p.ibiFusionRelation),

    p.bpmAutocorr,
    p.autocorrQuality,
    p.autocorrStrength,
    relationName(p.autocorrFusionRelation),

    p.bpmSpectral,
    p.bpmSpectralRawPeak,
    p.spectralQuality,
    p.spectralDominance,
    p.spectralSecondHarmonicRatio,
    relationName(p.spectralFusionRelation),

    p.bpmFused,
    p.fusionConfidence,
    p.methodsAgree,
    p.fusionValid ? "YES" : "NO",
    p.harmonicSuspect ? "YES" : "NO",
    p.harmonicResolved ? "YES" : "NO",
    p.fusionScore,

    p.hybridSamples,
    (unsigned long)p.hybridAnalysisUs,
    (unsigned long)p.hybridAnalysisMaxUs,
    p.hybridSliceUs,
    p.hybridSliceMaxUs,
    p.hybridCycleElapsedMs,
    p.sampleGapMaxMs
  );
}

void printStatus(
  const PulseAPDS9008Data& p
) {
  Serial.printf(
    "[SIGNAL] state=%s contact=%s motion=%s "
    "raw=%u dc=%.1f ac=%+.1f ppg=%+.1f env=%.1f noise=%.1f "
    "snr=%.2f bpmIBI=%u fused=%u valid=%s\n",
    pulseSensor.stateName(),
    p.contactLikely ? "YES" : "NO",
    p.motionDetected ? "YES" : "NO",
    p.raw,
    p.dc,
    p.ac,
    p.ppgFiltered,
    p.envelope,
    p.noise,
    p.contactSnr,
    p.bpmIbi,
    p.bpmFused,
    p.fusionValid ? "YES" : "NO"
  );
}

void printSignalPlotter(
  const PulseAPDS9008Data& p
) {
  float rearmMagnitude = p.noise;

  if (rearmMagnitude < 4.0f) {
    rearmMagnitude = 4.0f;
  }

  Serial.print("RAW:");
  Serial.print(p.raw);

  Serial.print(",DC:");
  Serial.print(p.dc, 1);

  Serial.print(",AC:");
  Serial.print(PLOT_CENTER + p.ac, 1);

  Serial.print(",PPG:");
  Serial.print(PLOT_CENTER + p.ppgFiltered, 1);

  Serial.print(",ENV:");
  Serial.print(PLOT_CENTER + p.envelope, 1);

  Serial.print(",THR:");
  Serial.print(PLOT_CENTER + p.threshold, 1);

  Serial.print(",REARM:");
  Serial.print(PLOT_CENTER - rearmMagnitude, 1);

  Serial.println();
}

void printBpmPlotter(
  const PulseAPDS9008Data& p
) {
  Serial.print("IBI:");
  Serial.print(p.bpmIbi);

  Serial.print(",CORR:");
  Serial.print(p.bpmAutocorr);

  Serial.print(",SPEC:");
  Serial.print(p.bpmSpectral);

  Serial.print(",FUSED:");
  Serial.print(p.bpmFused);

  Serial.print(",FUSION_Q:");
  Serial.print(p.fusionConfidence);

  Serial.print(",VALID:");
  Serial.print(
    p.fusionValid ? 100 : 0
  );

  Serial.print(",HARMONIC:");
  Serial.print(
    p.harmonicSuspect ? 100 : 0
  );

  Serial.print(",RESOLVED:");
  Serial.print(
    p.harmonicResolved ? 100 : 0
  );

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);

  configurePulseSensor();

  if (SERIAL_OUTPUT_MODE == 0) {
    Serial.println();
    Serial.println("============================================================");
    Serial.println(" APDS-9008 HYBRID 3-METHOD v0.5.3 COOPERATIVE-HARMONIC");
    Serial.println("============================================================");
    Serial.println("1: Peak/IBI");
    Serial.println("2: Autocorrelacion");
    Serial.println("3: Fourier/Goertzel");
    Serial.println("Fusion: familias armonicas 0.5x / 1x / 2x");
    Serial.println();
    Serial.println("PPG band: ~0.6-4 Hz");
    Serial.println("Sensor/IBI: 100 Hz");
    Serial.println("Hybrid CORR/SPEC: 25 Hz (decimation x4)");
    Serial.println("Hybrid window: 8 s");
    Serial.println("Hybrid starts: 5 s");
    Serial.println("Hybrid update target: 1 s");
    Serial.println("Hybrid CPU: cooperativo <= ~1.8 ms por muestra");
    Serial.println("Spectral scan: 2 BPM + interpolacion");
    Serial.println();
    Serial.println("bpm legado NO ha sido reemplazado aun por bpmFused.");
    Serial.println("============================================================");
    Serial.println();
  }
}

void loop() {
  if (!pulseSensor.update()) {
    return;
  }

  const PulseAPDS9008Data& p =
    pulseSensor.data();

  if (SERIAL_OUTPUT_MODE == 1) {
    static uint8_t plotDivider = 0;

    plotDivider++;

    if (
      plotDivider >=
      SIGNAL_PLOTTER_DECIMATION
    ) {
      plotDivider = 0;
      printSignalPlotter(p);
    }

    return;
  }

  if (SERIAL_OUTPUT_MODE == 2) {
    static uint32_t lastBpmPlotMs = 0;

    if (
      millis() - lastBpmPlotMs >=
      BPM_PLOTTER_INTERVAL_MS
    ) {
      lastBpmPlotMs = millis();
      printBpmPlotter(p);
    }

    return;
  }

  if (p.beat) {
    printBeat(p);
  }

  if (p.hybridUpdated) {
    printHybrid(p);
  }

  static uint32_t lastStatusMs = 0;

  if (
    millis() - lastStatusMs >=
    SERIAL_STATUS_INTERVAL_MS
  ) {
    lastStatusMs = millis();
    printStatus(p);
  }
}
