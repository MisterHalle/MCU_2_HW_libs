/*
  MAX30102 WristFilter Rebased OpticalGate v1.1.3
  ESP32-C3 + MAX30102

  Base:
  - Detector RhythmLock v1.1.1, que obtuvo las mejores mediciones en quietud.

  Mejoras v1.1.3:
  - Compuerta óptica MOVING / SETTLING / ACQUIRING / TRACKING.
  - Limpieza de candidatos al volver a quietud.
  - BPM provisional con 2 intervalos y referencia reciente.
  - BPM válido con 3 intervalos coherentes.
  - Confianza temporal separada de confianza morfológica.
  - Recuperación restringida a un solo pulso omitido, solo en TRACKING.
  - Snapshot exacto de métricas por cada muestra del CSV.

  IMPORTANTE:
  La compuerta óptica incluida está ajustada para MUÑECA.
  Para validar en la yema del dedo, cambia WRIST_MOTION_GATE a false.
*/

#include <Wire.h>
#include "PulseOximeter-MAX30102.h"

PulseOximeter pox;
PulseOximeter::Sample samples[8];

#define SDA_PIN 5
#define SCL_PIN 4
#define WRIST_MOTION_GATE true

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!pox.begin()) {
    Serial.println("MAX30102 no encontrado o fallo de inicializacion");
    while (1) delay(1000);
  }

  // Configuración óptica conservada de las versiones anteriores.
  pox.setExposureTargets(70000, 180000);
  pox.setLedBackoffMargin(10000);
  pox.setMinLedPulseAmplitude(0x04);
  pox.setFingerThresholds(10000, 5000);
  pox.setSaturationThreshold(250000);
  pox.setUsableStableSamples(50);
  pox.setDCSettleTolerance(3000, 3000);
  pox.enableAutoExposure(true);

  // Filtro que ya demostró funcionar en muñeca y dedo.
  pox.setPulseFilter(0.50f, 4.00f, 0.20f);
  pox.setPulseQualityThreshold(20.0f);
  pox.setMinimumPulseRms(35.0f);

  // Núcleo RhythmLock v1.1.1.
  pox.setHeartRatePeakDetection(0.20f, 35.0f);
  pox.setBeatRefractory(320);
  pox.setBeatHysteresis(0.45f);
  pox.setRhythmAcquisition(3, 25.0f);
  pox.setHeartRateRange(45.0f, 200.0f);

  // La confianza temporal no representa BPM.
  pox.setMinimumRhythmConfidence(65.0f);
  pox.setMinimumMorphologyConfidence(45.0f);

  // Respuesta provisional basada en una referencia reciente.
  pox.setProvisionalRhythm(75.0f, 2, 15000UL);

  // Solo permite recuperar un pulso omitido, nunca cadenas x3/x4.
  pox.setMissedBeatRecovery(2, 18.0f);

  // Ventana corta usada exclusivamente después de confirmar quietud.
  pox.setFastReacquireThreshold(0.80f, 1.35f);

  // Umbrales obtenidos a partir de las capturas reales de muñeca.
  pox.setOpticalMotionGate(
    0.95f,  // RED: movimiento
    1.15f,  // IR: movimiento
    0.65f,  // RED: vuelve a quietud
    0.80f,  // IR: vuelve a quietud
    650UL,  // quietud sostenida antes de adquirir
    1.0f    // ventana de análisis, segundos
  );
  pox.enableOpticalMotionGate(WRIST_MOTION_GATE);

  pox.setHeartRateSmoothing(0.40f, 0.18f, 8.0f);
  pox.setHeartRateTimeout(2500);
  pox.setHeartRateHoldTimeout(5000);
  pox.setHeartRateJumpLimit(15.0f);

  Serial.print("PART_ID: 0x");
  Serial.println(pox.readPartID(), HEX);
  Serial.print("REV_ID: 0x");
  Serial.println(pox.readRevisionID(), HEX);
  Serial.print("SPS efectivos: ");
  Serial.println(pox.getEffectiveSampleRateHz(), 1);
  Serial.print("Optical Motion Gate: ");
  Serial.println(WRIST_MOTION_GATE ? "WRIST ON" : "OFF / FINGER TEST");

  Serial.println(
    "ts_ms,red,ir,red_f,ir_f,thr,quality,pi,corr,gain,gain_dev,"
    "optical_ok,state,quiet_ok,motion,candidate,beat,ibi_ms,polarity,"
    "temporal_conf,morph_conf,min_temporal,min_morph,reject,recovered_n,"
    "hr,hr_valid,hr_provisional,hr_held,hr_provisional_bpm,ref_ibi_ms,"
    "red_range_pct,ir_range_pct,R,SpO2,usable,stableN,adc,redPA,irPA,adj"
  );
}

void loop() {
  uint8_t avail = pox.availableSamples();
  if (avail == 0) {
    delay(5);
    return;
  }

  size_t n = pox.readAvailableSamples(samples, 8);
  if (n == 0) {
    delay(5);
    return;
  }

  bool adjusted = pox.processSamples(samples, n);

  for (size_t i = 0; i < n; i++) {
    const PulseOximeter::Sample& s = samples[i];

    Serial.print(s.timestampMs);
    Serial.print(','); Serial.print(s.red);
    Serial.print(','); Serial.print(s.ir);
    Serial.print(','); Serial.print(s.redFiltered, 2);
    Serial.print(','); Serial.print(s.irFiltered, 2);
    Serial.print(','); Serial.print(s.adaptiveThreshold, 2);
    Serial.print(','); Serial.print(s.pulseQuality, 1);
    Serial.print(','); Serial.print(s.perfusionIndexPercent, 4);
    Serial.print(','); Serial.print(s.redIrCorrelation, 3);
    Serial.print(','); Serial.print(s.redIrGain, 3);
    Serial.print(','); Serial.print(s.redIrGainDeviation, 3);
    Serial.print(','); Serial.print(s.opticalSignalUsable ? 1 : 0);
    Serial.print(','); Serial.print(s.acquisitionState);
    Serial.print(','); Serial.print(s.quietStable ? 1 : 0);
    Serial.print(','); Serial.print(s.motionArtifact ? 1 : 0);
    Serial.print(','); Serial.print(s.candidateDetected ? 1 : 0);
    Serial.print(','); Serial.print(s.beatDetected ? 1 : 0);
    Serial.print(','); Serial.print(s.beatIntervalMs);
    Serial.print(','); Serial.print(s.pulsePolarity);
    Serial.print(','); Serial.print(s.temporalConfidence, 1);
    Serial.print(','); Serial.print(s.morphologyConfidence, 1);
    Serial.print(','); Serial.print(pox.getMinimumRhythmConfidence(), 1);
    Serial.print(','); Serial.print(pox.getMinimumMorphologyConfidence(), 1);
    Serial.print(','); Serial.print(s.rejectCode);
    Serial.print(','); Serial.print(s.recoveredBeatMultiple);
    Serial.print(','); Serial.print(s.heartRateBpm, 1);
    Serial.print(','); Serial.print(s.heartRateValid ? 1 : 0);
    Serial.print(','); Serial.print(s.heartRateProvisional ? 1 : 0);
    Serial.print(','); Serial.print(s.heartRateHeld ? 1 : 0);
    Serial.print(','); Serial.print(s.provisionalHeartRateBpm, 1);
    Serial.print(','); Serial.print(s.referenceBeatIntervalMs);
    Serial.print(','); Serial.print(s.redMotionRangePercent, 3);
    Serial.print(','); Serial.print(s.irMotionRangePercent, 3);
    Serial.print(','); Serial.print(pox.getRValue(), 4);
    Serial.print(','); Serial.print(pox.getSpO2(), 1);
    Serial.print(','); Serial.print(pox.isSignalUsable() ? 1 : 0);
    Serial.print(','); Serial.print(pox.getStableSampleCount());
    Serial.print(','); Serial.print(pox.getAdcRange());
    Serial.print(','); Serial.print(pox.getConfig().redLedPA);
    Serial.print(','); Serial.print(pox.getConfig().irLedPA);
    Serial.print(','); Serial.println(adjusted ? 1 : 0);
  }

  delay(5);
}