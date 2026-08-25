/* Copyright 2026 Hall-e SpA

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     www.apache.org

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License. */

#include "PulseAPDS9008.h"

PulseAPDS9008::PulseAPDS9008() {
}

bool PulseAPDS9008::begin(
  const PulseAPDS9008Config& config
) {
  _cfg = config;

  if (_cfg.sampleIntervalMs < 2) {
    _cfg.sampleIntervalMs = 2;
  }

  if (_cfg.minBpm < 20) {
    _cfg.minBpm = 20;
  }

  if (_cfg.maxBpm <= _cfg.minBpm) {
    _cfg.maxBpm = _cfg.minBpm + 1;
  }

  if (_cfg.minPulseAmplitude < 1.0f) {
    _cfg.minPulseAmplitude = 1.0f;
  }

  if (_cfg.fallingSamplesToConfirm < 1) {
    _cfg.fallingSamplesToConfirm = 1;
  }

  if (_cfg.acquisitionMinIbi < 3) {
    _cfg.acquisitionMinIbi = 3;
  }

  if (_cfg.reacquireMinIbi < 3) {
    _cfg.reacquireMinIbi = 3;
  }

  pinMode(_cfg.signalPin, INPUT);
  pinMode(_cfg.ledPin, OUTPUT);

  _data.ledPower = _cfg.ledPower;

  applyLedPower();

  configureHybridAnalyzer();

  reset();

  _startMs = millis();
  _stateEnteredMs = _startMs;

  _started = true;

  return true;
}

bool PulseAPDS9008::update() {
  if (!_started) {
    return false;
  }

  uint32_t nowMs = millis();

  uint32_t sampleGap =
    nowMs - _lastSampleMs;

  if (
    sampleGap <
    _cfg.sampleIntervalMs
  ) {
    return false;
  }

  if (_lastSampleMs != 0) {
    if (sampleGap > 65535UL) {
      sampleGap = 65535UL;
    }

    _data.sampleGapMs =
      (uint16_t)sampleGap;

    if (
      _data.sampleGapMs >
      _data.sampleGapMaxMs
    ) {
      _data.sampleGapMaxMs =
        _data.sampleGapMs;
    }
  }

  _lastSampleMs = nowMs;

  uint16_t raw =
    analogRead(_cfg.signalPin);

  processSample(raw, nowMs);

  return true;
}

void PulseAPDS9008::reset() {
  uint8_t currentLedPower =
    _data.ledPower;

  _data = PulseAPDS9008Data();
  _data.ledPower = currentLedPower;

  _signalInitialized = false;

  _lastSampleMs = 0;
  _stateEnteredMs = 0;

  _lastDetectedBeatMs = 0;
  _lastAcceptedBeatMs = 0;
  _lastSignalEvidenceMs = 0;

  _contactEvidenceStartMs = 0;
  _motionUntilMs = 0;

  _filteredAc = 0.0f;
  _previousFilteredAc = 0.0f;
  _previousRaw = 0.0f;

  _clipLevel = 0.0f;
  _qualityEma = 0.0f;
  _motionEma = 0.0f;

  _motionEvidenceCount = 0;
  _consecutiveOutliers = 0;

  _beatRearmed = true;
  _data.beatRearmed = true;

  _cycleValley = 0.0f;

  cancelCandidate();

  clearStableHistory();
  clearAcquisition();
  clearReacquisition();

  resetHybridAnalyzer();

  _data.state = PULSE_STATE_NO_CONTACT;
}

const PulseAPDS9008Data&
PulseAPDS9008::data() const {
  return _data;
}

bool PulseAPDS9008::beatDetected() const {
  return _data.beat;
}

uint16_t PulseAPDS9008::bpm() const {
  return _data.bpm;
}

uint16_t PulseAPDS9008::ibiMs() const {
  return _data.ibiMs;
}

uint32_t PulseAPDS9008::beatCount() const {
  return _data.beatCount;
}

uint8_t PulseAPDS9008::quality() const {
  return _data.quality;
}

uint8_t PulseAPDS9008::confidence() const {
  return _data.confidence;
}

bool PulseAPDS9008::contactLikely() const {
  return _data.contactLikely;
}

bool PulseAPDS9008::motionDetected() const {
  return _data.motionDetected;
}

bool PulseAPDS9008::measurementValid() const {
  return _data.measurementValid;
}

PulseMeasurementState
PulseAPDS9008::state() const {
  return _data.state;
}

const char* PulseAPDS9008::stateName() const {
  switch (_data.state) {
    case PULSE_STATE_NO_CONTACT:
      return "NO_CONTACT";

    case PULSE_STATE_ACQUIRING:
      return "ACQUIRING";

    case PULSE_STATE_TRACKING:
      return "TRACKING";

    case PULSE_STATE_MOTION:
      return "MOTION";

    default:
      return "UNKNOWN";
  }
}

void PulseAPDS9008::setLedPower(
  uint8_t power
) {
  _cfg.ledPower = power;
  _data.ledPower = power;

  if (_started) {
    applyLedPower();
  }
}

uint8_t PulseAPDS9008::ledPower() const {
  return _data.ledPower;
}

void PulseAPDS9008::setAutoLedControl(
  bool enabled
) {
  _cfg.autoLedControl = enabled;
}

bool PulseAPDS9008::autoLedControl() const {
  return _cfg.autoLedControl;
}

void PulseAPDS9008::setMinPulseAmplitude(
  float amplitude
) {
  if (amplitude < 1.0f) {
    amplitude = 1.0f;
  }

  _cfg.minPulseAmplitude = amplitude;
}

float PulseAPDS9008::minPulseAmplitude()
const {
  return _cfg.minPulseAmplitude;
}

uint16_t PulseAPDS9008::fusedBpm() const {
  return _data.bpmFused;
}

uint8_t PulseAPDS9008::fusionConfidence()
const {
  return _data.fusionConfidence;
}

bool PulseAPDS9008::fusionValid() const {
  return _data.fusionValid;
}

void PulseAPDS9008::processSample(
  uint16_t raw,
  uint32_t nowMs
) {
  _data.beat = false;
  _data.ibiAccepted = false;
  _data.hybridUpdated = false;
  _data.raw = raw;

  // El metodo IBI es el estimador legado. Se copia como una
  // salida separada para compararlo con los otros dos metodos.
  _data.bpmIbi =
    _data.bpmStable
      ? _data.bpm
      : 0;

  if (_data.bpmStable) {
    uint8_t ibiQ =
      _data.confidence;

    if (
      _data.quality > ibiQ
    ) {
      ibiQ =
        _data.quality;
    }

    _data.ibiMethodQuality =
      ibiQ;
  } else {
    _data.ibiMethodQuality = 0;
  }

  // La fusion se refresca tambien entre analisis espectrales,
  // permitiendo reaccionar a cambios del metodo IBI.
  updateFusion();

  if (!_signalInitialized) {
    _data.dc = (float)raw;
    _data.ac = 0.0f;
    _data.envelope = 0.0f;
    _data.noise = 0.0f;
    _data.threshold =
      _cfg.minPulseAmplitude;

    _filteredAc = 0.0f;
    _previousFilteredAc = 0.0f;
    _previousRaw = (float)raw;

    _cycleValley = 0.0f;

    _ppgInitialized = false;
    _data.ppgFiltered = 0.0f;

    _signalInitialized = true;
    return;
  }

  // ========================================================
  // 1. DC
  // ========================================================

  _data.dc +=
    _cfg.dcAlpha *
    ((float)raw - _data.dc);

  // ========================================================
  // 2. AC SUAVIZADA
  // ========================================================

  float previousAc =
    _filteredAc;

  float ac =
    (float)raw - _data.dc;

  _filteredAc +=
    _cfg.signalAlpha *
    (ac - _filteredAc);

  _data.ac = _filteredAc;

  // Banda PPG paralela (~0.6-4 Hz). No reemplaza el AC legado:
  // se usa exclusivamente para autocorrelacion/espectro.
  updatePpgBandpass(
    _filteredAc
  );

  float slope =
    _filteredAc - previousAc;

  // ========================================================
  // 3. ENVOLVENTE
  // ========================================================

  float absAc =
    absFloat(_filteredAc);

  _data.envelope +=
    _cfg.envelopeAlpha *
    (absAc - _data.envelope);

  // ========================================================
  // 4. RUIDO RAPIDO
  // ========================================================

  float sampleDelta =
    absFloat(
      _filteredAc -
      _previousFilteredAc
    );

  _data.noise +=
    _cfg.noiseAlpha *
    (sampleDelta - _data.noise);

  _previousFilteredAc =
    _filteredAc;

  // ========================================================
  // 5. THRESHOLD ADAPTATIVO
  // ========================================================

  float thresholdFromEnvelope =
    _data.envelope *
    _cfg.envelopeThresholdFactor;

  float thresholdFromNoise =
    _data.noise *
    _cfg.noiseThresholdFactor;

  float adaptiveThreshold =
    _cfg.minPulseAmplitude;

  if (
    thresholdFromEnvelope >
    adaptiveThreshold
  ) {
    adaptiveThreshold =
      thresholdFromEnvelope;
  }

  if (
    thresholdFromNoise >
    adaptiveThreshold
  ) {
    adaptiveThreshold =
      thresholdFromNoise;
  }

  _data.threshold =
    adaptiveThreshold;

  float releaseThreshold =
    adaptiveThreshold *
    _cfg.releaseFactor;

  // ========================================================
  // 6. MOVIMIENTO / CONTACTO / CALIDAD
  // ========================================================

  updateMotion(raw, nowMs);
  updateContact(raw, nowMs);
  updateQuality(raw);
  updateState(nowMs);
  updateConfidence();

  // La historia espectral solo se alimenta cuando el contacto
  // permanece probable y no existe MOTION. Un IBI puede fallar
  // sin borrar esta historia: esa es precisamente la redundancia.
  if (
    _cfg.hybridEnabled &&
    _data.contactLikely &&
    !_data.motionDetected
  ) {
    _hybridDecimationCounter++;

    if (
      _hybridDecimationCounter >=
      _cfg.hybridDecimation
    ) {
      _hybridDecimationCounter = 0;

      pushHybridSample(
        _data.ppgFiltered
      );
    }

    maybeRunHybridAnalysis(
      nowMs
    );
  } else {
    _data.fusionValid = false;
  }

  // ========================================================
  // 6B. REARME ENTRE ONDAS
  // ========================================================
  //
  // Un segundo hombro/dicrotic wave no puede iniciar otro
  // candidato hasta que la AC pase por un valle negativo.
  if (
    _cfg.beatRearmEnabled &&
    !_beatRearmed
  ) {
    float rearmFloor =
      _data.noise *
      _cfg.beatRearmNoiseFactor;

    if (
      rearmFloor <
      _cfg.beatRearmMinAc
    ) {
      rearmFloor =
        _cfg.beatRearmMinAc;
    }

    if (
      _filteredAc <=
      -rearmFloor
    ) {
      _beatRearmed = true;
    }
  }

  if (!_cfg.beatRearmEnabled) {
    _beatRearmed = true;
  }

  _data.beatRearmed =
    _beatRearmed;

  _previousRaw =
    (float)raw;

  // ========================================================
  // 7. WARMUP
  // ========================================================

  if (
    nowMs - _startMs <
    _cfg.warmupMs
  ) {
    return;
  }

  // No procesamos latidos durante ausencia de contacto
  // o movimiento.
  if (
    _data.state ==
      PULSE_STATE_NO_CONTACT ||
    _data.state ==
      PULSE_STATE_MOTION
  ) {
    return;
  }

  // Al entrar en ACQUIRING dejamos estabilizar el contacto.
  if (
    _data.state ==
      PULSE_STATE_ACQUIRING &&
    nowMs - _stateEnteredMs <
      _cfg.acquisitionSettleMs
  ) {
    return;
  }

  // ========================================================
  // 8. VALLE
  // ========================================================

  if (!_candidateActive) {
    if (
      _cycleValley == 0.0f ||
      _filteredAc < _cycleValley
    ) {
      _cycleValley =
        _filteredAc;
    }
  }

  // ========================================================
  // 9. CANDIDATO ACTIVO
  // ========================================================

  if (_candidateActive) {
    updateCandidate(
      nowMs,
      releaseThreshold
    );

    return;
  }

  // ========================================================
  // 10. INICIO DE CANDIDATO
  // ========================================================

  uint32_t refractoryMs =
    currentRefractoryMs();

  bool refractoryFinished =
    (_lastAcceptedBeatMs == 0) ||
    (
      nowMs -
      _lastAcceptedBeatMs >=
      refractoryMs
    );

  float minimumRise =
    0.5f +
    (_data.noise * 0.10f);

  bool rising =
    slope >= minimumRise;

  bool crossing =
    _filteredAc >=
    adaptiveThreshold;

  bool adcUsable =
    raw > _cfg.contactAdcMin &&
    raw < _cfg.contactAdcMax;

  bool morphologyRearmed =
    !_cfg.beatRearmEnabled ||
    _beatRearmed;

  if (
    refractoryFinished &&
    morphologyRearmed &&
    rising &&
    crossing &&
    adcUsable
  ) {
    startCandidate(
      nowMs,
      adaptiveThreshold
    );
  }
}

void PulseAPDS9008::updateMotion(
  uint16_t raw,
  uint32_t nowMs
) {
  float rawJump =
    absFloat(
      (float)raw -
      _previousRaw
    );

  float jumpReference =
    _data.envelope * 3.5f;

  if (
    jumpReference <
    _cfg.motionRawJumpFloor
  ) {
    jumpReference =
      _cfg.motionRawJumpFloor;
  }

  float jumpScore =
    rawJump / jumpReference;

  float acReference =
    _data.envelope * 6.0f;

  if (
    acReference <
    _cfg.motionAcFloor
  ) {
    acReference =
      _cfg.motionAcFloor;
  }

  float acScore =
    absFloat(_filteredAc) /
    acReference;

  float denominator =
    _data.envelope;

  if (
    denominator <
    _cfg.minPulseAmplitude
  ) {
    denominator =
      _cfg.minPulseAmplitude;
  }

  float noiseRatio =
    _data.noise /
    denominator;

  float noiseScore = 0.0f;

  if (
    _data.envelope >=
    _cfg.minPulseAmplitude * 0.70f
  ) {
    noiseScore =
      noiseRatio /
      _cfg.motionNoiseRatio;
  }

  float instantMotion =
    jumpScore;

  if (acScore > instantMotion) {
    instantMotion = acScore;
  }

  if (noiseScore > instantMotion) {
    instantMotion = noiseScore;
  }

  instantMotion =
    clampFloat(
      instantMotion,
      0.0f,
      2.0f
    );

  _motionEma +=
    0.18f *
    (instantMotion - _motionEma);

  _data.motionScore =
    clampFloat(
      _motionEma * 100.0f,
      0.0f,
      100.0f
    );

  bool motionEvidence =
    instantMotion >= 1.0f;

  if (motionEvidence) {
    if (_motionEvidenceCount < 255) {
      _motionEvidenceCount++;
    }
  } else {
    _motionEvidenceCount = 0;
  }

  if (
    _motionEvidenceCount >=
    _cfg.motionConfirmSamples
  ) {
    if (!_data.motionDetected) {
      _data.artifactCount++;
    }

    _motionUntilMs =
      nowMs +
      _cfg.motionHoldMs;

    _motionEvidenceCount = 0;
  }

  _data.motionDetected =
    nowMs < _motionUntilMs;
}

void PulseAPDS9008::updateContact(
  uint16_t raw,
  uint32_t nowMs
) {
  bool adcUsable =
    raw > _cfg.contactAdcMin &&
    raw < _cfg.contactAdcMax &&
    _data.dc > _cfg.contactAdcMin &&
    _data.dc < _cfg.contactAdcMax;

  float contactEnvelopeFloor =
    _cfg.minPulseAmplitude *
    _cfg.contactEnvelopeFactor;

  if (
    contactEnvelopeFloor <
    _cfg.contactMinEnvelope
  ) {
    contactEnvelopeFloor =
      _cfg.contactMinEnvelope;
  }

  bool enoughEnvelope =
    _data.envelope >=
    contactEnvelopeFloor;

  float contactSnr =
    _data.envelope /
    (_data.noise + 0.5f);

  _data.contactSnr =
    contactSnr;

  float noiseRatio =
    _data.noise /
    (_data.envelope + 0.5f);

  bool snrGood =
    contactSnr >=
    _cfg.contactMinSnr;

  bool noiseRatioGood =
    noiseRatio <=
    _cfg.contactMaxNoiseRatio;

  bool cleanEnough =
    !_data.motionDetected &&
    _data.motionScore < 70.0f;

  bool evidence =
    adcUsable &&
    enoughEnvelope &&
    snrGood &&
    noiseRatioGood &&
    cleanEnough;

  if (evidence) {
    _lastSignalEvidenceMs =
      nowMs;

    if (_contactEvidenceStartMs == 0) {
      _contactEvidenceStartMs =
        nowMs;
    }

    if (
      !_data.contactLikely &&
      nowMs -
      _contactEvidenceStartMs >=
      _cfg.contactConfirmMs
    ) {
      _data.contactLikely = true;
    }
  } else {
    _contactEvidenceStartMs = 0;
  }

  if (
    _data.contactLikely &&
    _lastSignalEvidenceMs != 0 &&
    nowMs -
    _lastSignalEvidenceMs >
      _cfg.contactLostMs
  ) {
    _data.contactLikely = false;
  }
}

void PulseAPDS9008::updateState(
  uint32_t nowMs
) {
  if (_data.motionDetected) {
    if (
      _data.state !=
      PULSE_STATE_MOTION
    ) {
      enterState(
        PULSE_STATE_MOTION,
        nowMs
      );
    }

    return;
  }

  if (!_data.contactLikely) {
    if (
      _data.state !=
      PULSE_STATE_NO_CONTACT
    ) {
      enterState(
        PULSE_STATE_NO_CONTACT,
        nowMs
      );
    }

    return;
  }

  if (
    _data.state ==
      PULSE_STATE_NO_CONTACT ||
    _data.state ==
      PULSE_STATE_MOTION
  ) {
    enterState(
      PULSE_STATE_ACQUIRING,
      nowMs
    );
  }
}

void PulseAPDS9008::enterState(
  PulseMeasurementState newState,
  uint32_t nowMs
) {
  if (_data.state == newState) {
    return;
  }

  _data.state =
    newState;

  _stateEnteredMs =
    nowMs;

  _data.measurementValid =
    false;

  cancelCandidate();

  if (
    newState ==
      PULSE_STATE_NO_CONTACT ||
    newState ==
      PULSE_STATE_MOTION
  ) {
    _lastDetectedBeatMs = 0;
    _lastAcceptedBeatMs = 0;

    _beatRearmed = true;
    _data.beatRearmed = true;

    clearAcquisition();
    clearReacquisition();

    _consecutiveOutliers = 0;

    // La medicion anterior deja de ser publicable.
    _data.bpm = 0;
    _data.ibiMs = 0;
    _data.robustIbiMs = 0;
    _data.bpmStable = false;

    clearStableHistory();

    // No se conserva una conclusion espectral a traves de
    // ausencia de contacto o movimiento real.
    resetHybridAnalyzer();
  }

  if (
    newState ==
      PULSE_STATE_ACQUIRING
  ) {
    _lastDetectedBeatMs = 0;
    _lastAcceptedBeatMs = 0;

    _beatRearmed = true;
    _data.beatRearmed = true;

    _data.bpm = 0;
    _data.ibiMs = 0;
    _data.robustIbiMs = 0;
    _data.bpmStable = false;

    clearStableHistory();
    clearAcquisition();
    clearReacquisition();

    _consecutiveOutliers = 0;
  }

  if (
    newState ==
      PULSE_STATE_TRACKING
  ) {
    _data.bpmStable = true;
  }
}

uint32_t PulseAPDS9008::currentRefractoryMs()
const {
  uint32_t physiologicalMin =
    60000UL / _cfg.maxBpm;

  uint32_t refractory =
    physiologicalMin;

  if (
    _data.state ==
      PULSE_STATE_TRACKING &&
    _data.robustIbiMs > 0
  ) {
    uint32_t dynamicValue =
      (uint32_t)(
        (float)_data.robustIbiMs *
        _cfg.dynamicRefractoryFactor
      );

    if (
      dynamicValue >
      refractory
    ) {
      refractory =
        dynamicValue;
    }
  }

  return refractory;
}

void PulseAPDS9008::startCandidate(
  uint32_t nowMs,
  float threshold
) {
  _candidateActive = true;

  _candidateStartMs = nowMs;
  _candidateThreshold = threshold;

  _candidateValley =
    _cycleValley;

  _candidatePeak =
    _filteredAc;

  _candidatePeakMs =
    nowMs;

  _fallingSamples = 0;

  _data.candidateActive = true;
}

void PulseAPDS9008::updateCandidate(
  uint32_t nowMs,
  float releaseThreshold
) {
  if (
    _filteredAc >
    _candidatePeak
  ) {
    _candidatePeak =
      _filteredAc;

    _candidatePeakMs =
      nowMs;

    _fallingSamples = 0;
  } else {
    float fallNeeded =
      _data.noise *
      _cfg.peakFallNoiseFactor;

    if (
      fallNeeded <
      _cfg.minPeakFall
    ) {
      fallNeeded =
        _cfg.minPeakFall;
    }

    if (
      _filteredAc <=
      _candidatePeak -
      fallNeeded
    ) {
      if (_fallingSamples < 255) {
        _fallingSamples++;
      }
    } else {
      _fallingSamples = 0;
    }
  }

  bool enoughFalling =
    _fallingSamples >=
    _cfg.fallingSamplesToConfirm;

  bool released =
    _filteredAc <=
    releaseThreshold;

  bool timedOut =
    nowMs -
    _candidateStartMs >=
    _cfg.candidateTimeoutMs;

  bool peakIsOldEnough =
    nowMs -
    _candidatePeakMs >=
    _cfg.sampleIntervalMs;

  if (
    peakIsOldEnough &&
    (
      enoughFalling ||
      released ||
      timedOut
    )
  ) {
    finalizeCandidate(nowMs);
  }
}

void PulseAPDS9008::finalizeCandidate(
  uint32_t nowMs
) {
  (void)nowMs;

  float prominence =
    _candidatePeak -
    _candidateValley;

  float minProminence =
    _cfg.minPulseAmplitude *
    _cfg.prominenceFactor;

  float noiseProminence =
    _data.noise *
    _cfg.noiseProminenceFactor;

  if (
    noiseProminence >
    minProminence
  ) {
    minProminence =
      noiseProminence;
  }

  bool enoughPeak =
    _candidatePeak >=
    _candidateThreshold;

  bool enoughProminence =
    prominence >=
    minProminence;

  bool notMotion =
    !_data.motionDetected;

  if (
    enoughPeak &&
    enoughProminence &&
    notMotion
  ) {
    processBeat(
      _candidatePeakMs,
      _candidatePeak,
      prominence
    );
  } else {
    _data.rejectedPeakCount++;
  }

  _cycleValley =
    _filteredAc;

  cancelCandidate();
}

void PulseAPDS9008::cancelCandidate() {
  _candidateActive = false;

  _candidateStartMs = 0;
  _candidateThreshold = 0.0f;
  _candidateValley = 0.0f;
  _candidatePeak = 0.0f;
  _candidatePeakMs = 0;
  _fallingSamples = 0;

  _data.candidateActive = false;
}

void PulseAPDS9008::processBeat(
  uint32_t peakMs,
  float peakAmplitude,
  float prominence
) {
  _data.beat = true;
  _data.beatCount++;

  _data.peak =
    peakAmplitude;

  _data.prominence =
    prominence;

  // Un pico NO confirma contacto por sí solo.
  //
  // El contacto pertenece exclusivamente a updateContact(),
  // evitando que ruido de mesa perpetúe contactLikely=true.
  if (_cfg.beatRearmEnabled) {
    _beatRearmed = false;
    _data.beatRearmed = false;
  }

  // ========================================================
  // PRIMER PICO TEMPORAL
  // ========================================================

  if (_lastDetectedBeatMs == 0) {
    _lastDetectedBeatMs =
      peakMs;

    _lastAcceptedBeatMs =
      peakMs;

    _data.ibiMs = 0;
    _data.ibiAccepted = false;

    return;
  }

  uint32_t detectedIbi =
    peakMs -
    _lastDetectedBeatMs;

  _lastDetectedBeatMs =
    peakMs;

  _data.ibiMs =
    (uint16_t)detectedIbi;

  if (!physiologicIbi(detectedIbi)) {
    _data.rejectedIbiCount++;
    _data.ibiAccepted = false;
    return;
  }

  if (
    _data.state ==
      PULSE_STATE_ACQUIRING
  ) {
    processAcquisitionIbi(
      (uint16_t)detectedIbi,
      peakMs
    );

    return;
  }

  if (
    _data.state ==
      PULSE_STATE_TRACKING
  ) {
    processTrackingIbi(
      (uint16_t)detectedIbi,
      peakMs
    );
  }
}

bool PulseAPDS9008::physiologicIbi(
  uint32_t ibi
) const {
  uint32_t minIbi =
    60000UL / _cfg.maxBpm;

  uint32_t maxIbi =
    60000UL / _cfg.minBpm;

  return
    ibi >= minIbi &&
    ibi <= maxIbi;
}

bool PulseAPDS9008::ibiCloseTo(
  uint32_t ibi,
  uint16_t reference,
  float tolerance
) const {
  if (reference == 0) {
    return false;
  }

  float deviation =
    absFloat(
      (float)ibi -
      (float)reference
    ) /
    (float)reference;

  return
    deviation <= tolerance;
}

void PulseAPDS9008::processAcquisitionIbi(
  uint16_t ibi,
  uint32_t peakMs
) {
  pushAcqIbi(ibi);

  _data.ibiAccepted = false;

  uint16_t clusterMedian = 0;

  if (
    acquisitionClusterReady(
      _acqBuffer,
      _acqCount,
      clusterMedian
    )
  ) {
    lockFromAcquisition(
      clusterMedian,
      peakMs
    );

    _data.ibiAccepted = true;
  }
}

void PulseAPDS9008::processTrackingIbi(
  uint16_t detectedIbi,
  uint32_t peakMs
) {
  uint16_t reference =
    medianIbi();

  if (reference == 0) {
    enterState(
      PULSE_STATE_ACQUIRING,
      peakMs
    );

    return;
  }

  // --------------------------------------------------------
  // ANCLA AL ULTIMO PULSO ACEPTADO
  //
  // Si hubo un falso pico intermedio, el IBI detectado puede
  // ser corto, pero peakMs-lastAcceptedBeatMs puede coincidir
  // perfectamente con el ritmo real.
  // --------------------------------------------------------

  uint32_t anchoredIbi =
    detectedIbi;

  if (_lastAcceptedBeatMs != 0) {
    anchoredIbi =
      peakMs -
      _lastAcceptedBeatMs;
  }

  bool anchoredPhys =
    physiologicIbi(
      anchoredIbi
    );

  bool anchoredMatch =
    anchoredPhys &&
    ibiCloseTo(
      anchoredIbi,
      reference,
      _cfg.trackingIbiTolerance
    );

  bool detectedMatch =
    ibiCloseTo(
      detectedIbi,
      reference,
      _cfg.trackingIbiTolerance
    );

  if (
    anchoredMatch ||
    detectedMatch
  ) {
    uint16_t acceptedIbi =
      anchoredMatch
        ? (uint16_t)anchoredIbi
        : detectedIbi;

    pushStableIbi(
      acceptedIbi
    );

    _lastAcceptedBeatMs =
      peakMs;

    _data.ibiMs =
      acceptedIbi;

    _data.ibiAccepted =
      true;

    _data.validBeatCount++;

    _consecutiveOutliers = 0;

    clearReacquisition();

    _data.robustIbiMs =
      robustAverageIbi();

    if (_data.robustIbiMs > 0) {
      _data.bpm =
        (uint16_t)(
          (
            60000UL +
            (_data.robustIbiMs / 2)
          ) /
          _data.robustIbiMs
        );
    }

    _data.validIbiCount =
      _ibiCount;

    _data.bpmStable =
      _ibiCount >=
      _cfg.acquisitionMinIbi;

    _data.measurementValid =
      _data.bpmStable &&
      _data.contactLikely &&
      !_data.motionDetected;

    return;
  }

  // --------------------------------------------------------
  // OUTLIER
  // --------------------------------------------------------

  _data.rejectedIbiCount++;
  _data.ibiAccepted = false;

  if (_consecutiveOutliers < 255) {
    _consecutiveOutliers++;
  }

  pushReacqIbi(
    detectedIbi
  );

  uint16_t newMedian = 0;

  if (
    acquisitionClusterReady(
      _reacqBuffer,
      _reacqCount,
      newMedian
    ) &&
    _reacqCount >=
      _cfg.reacquireMinIbi
  ) {
    clearStableHistory();

    // Sembrar historia con el cluster de reacquisicion.
    for (
      uint8_t i = 0;
      i < _reacqCount;
      i++
    ) {
      if (
        ibiCloseTo(
          _reacqBuffer[i],
          newMedian,
          _cfg.reacquireTolerance
        )
      ) {
        pushStableIbi(
          _reacqBuffer[i]
        );
      }
    }

    _lastAcceptedBeatMs =
      peakMs;

    _data.robustIbiMs =
      robustAverageIbi();

    if (_data.robustIbiMs > 0) {
      _data.bpm =
        (uint16_t)(
          (
            60000UL +
            (_data.robustIbiMs / 2)
          ) /
          _data.robustIbiMs
        );
    }

    _data.validIbiCount =
      _ibiCount;

    _data.bpmStable =
      _ibiCount >=
      _cfg.reacquireMinIbi;

    _data.reacquisitionCount++;

    _consecutiveOutliers = 0;

    clearReacquisition();

    _data.measurementValid =
      _data.bpmStable &&
      _data.contactLikely &&
      !_data.motionDetected;

    return;
  }

  if (
    _consecutiveOutliers >=
    _cfg.maxConsecutiveOutliers
  ) {
    enterState(
      PULSE_STATE_ACQUIRING,
      peakMs
    );
  }
}

bool PulseAPDS9008::acquisitionClusterReady(
  const uint16_t* values,
  uint8_t count,
  uint16_t& clusterMedian
) const {
  clusterMedian = 0;

  if (
    count <
    _cfg.acquisitionMinIbi
  ) {
    return false;
  }

  uint16_t median =
    medianOf(
      values,
      count
    );

  if (median == 0) {
    return false;
  }

  uint8_t coherent = 0;

  for (
    uint8_t i = 0;
    i < count;
    i++
  ) {
    if (
      ibiCloseTo(
        values[i],
        median,
        _cfg.acquisitionTolerance
      )
    ) {
      coherent++;
    }
  }

  uint8_t requiredCoherent =
    _cfg.acquisitionMinIbi;

  uint16_t candidateBpm =
    (uint16_t)(
      (
        60000UL +
        (median / 2)
      ) /
      median
    );

  if (
    candidateBpm >=
      _cfg.highRateBpmThreshold &&
    _cfg.highRateAcquisitionMinIbi >
      requiredCoherent
  ) {
    requiredCoherent =
      _cfg.highRateAcquisitionMinIbi;
  }

  if (
    count >= requiredCoherent &&
    coherent >= requiredCoherent
  ) {
    clusterMedian = median;
    return true;
  }

  return false;
}

void PulseAPDS9008::lockFromAcquisition(
  uint16_t clusterMedian,
  uint32_t peakMs
) {
  clearStableHistory();

  for (
    uint8_t i = 0;
    i < _acqCount;
    i++
  ) {
    if (
      ibiCloseTo(
        _acqBuffer[i],
        clusterMedian,
        _cfg.acquisitionTolerance
      )
    ) {
      pushStableIbi(
        _acqBuffer[i]
      );
    }
  }

  _lastAcceptedBeatMs =
    peakMs;

  _data.robustIbiMs =
    robustAverageIbi();

  if (_data.robustIbiMs > 0) {
    _data.bpm =
      (uint16_t)(
        (
          60000UL +
          (_data.robustIbiMs / 2)
        ) /
        _data.robustIbiMs
      );
  }

  _data.validIbiCount =
    _ibiCount;

  _data.bpmStable =
    _ibiCount >=
    _cfg.acquisitionMinIbi;

  _data.validBeatCount +=
    _ibiCount;

  clearAcquisition();
  clearReacquisition();

  _consecutiveOutliers = 0;

  enterState(
    PULSE_STATE_TRACKING,
    peakMs
  );

  _data.measurementValid =
    _data.bpmStable &&
    _data.contactLikely &&
    !_data.motionDetected;
}

void PulseAPDS9008::pushStableIbi(
  uint16_t ibi
) {
  _ibiBuffer[_ibiIndex] =
    ibi;

  _ibiIndex++;

  if (
    _ibiIndex >=
    IBI_BUFFER_SIZE
  ) {
    _ibiIndex = 0;
  }

  if (
    _ibiCount <
    IBI_BUFFER_SIZE
  ) {
    _ibiCount++;
  }
}

void PulseAPDS9008::pushAcqIbi(
  uint16_t ibi
) {
  if (
    _acqCount <
    ACQ_BUFFER_SIZE
  ) {
    _acqBuffer[_acqCount++] =
      ibi;

    return;
  }

  for (
    uint8_t i = 1;
    i < ACQ_BUFFER_SIZE;
    i++
  ) {
    _acqBuffer[i - 1] =
      _acqBuffer[i];
  }

  _acqBuffer[
    ACQ_BUFFER_SIZE - 1
  ] = ibi;
}

void PulseAPDS9008::pushReacqIbi(
  uint16_t ibi
) {
  if (
    _reacqCount <
    ACQ_BUFFER_SIZE
  ) {
    _reacqBuffer[
      _reacqCount++
    ] = ibi;

    return;
  }

  for (
    uint8_t i = 1;
    i < ACQ_BUFFER_SIZE;
    i++
  ) {
    _reacqBuffer[i - 1] =
      _reacqBuffer[i];
  }

  _reacqBuffer[
    ACQ_BUFFER_SIZE - 1
  ] = ibi;
}

uint16_t PulseAPDS9008::medianIbi()
const {
  return
    medianOf(
      _ibiBuffer,
      _ibiCount
    );
}

uint16_t PulseAPDS9008::robustAverageIbi()
const {
  if (_ibiCount == 0) {
    return 0;
  }

  uint16_t median =
    medianIbi();

  if (
    _ibiCount < 3 ||
    median == 0
  ) {
    uint32_t sum = 0;

    for (
      uint8_t i = 0;
      i < _ibiCount;
      i++
    ) {
      sum +=
        _ibiBuffer[i];
    }

    return
      (uint16_t)(
        sum / _ibiCount
      );
  }

  float tolerance =
    0.20f;

  uint32_t sum = 0;
  uint8_t count = 0;

  for (
    uint8_t i = 0;
    i < _ibiCount;
    i++
  ) {
    if (
      ibiCloseTo(
        _ibiBuffer[i],
        median,
        tolerance
      )
    ) {
      sum +=
        _ibiBuffer[i];

      count++;
    }
  }

  if (count == 0) {
    return median;
  }

  return
    (uint16_t)(
      sum / count
    );
}

uint16_t PulseAPDS9008::medianOf(
  const uint16_t* values,
  uint8_t count
) const {
  if (count == 0) {
    return 0;
  }

  uint16_t sorted[
    ACQ_BUFFER_SIZE > IBI_BUFFER_SIZE
      ? ACQ_BUFFER_SIZE
      : IBI_BUFFER_SIZE
  ];

  for (
    uint8_t i = 0;
    i < count;
    i++
  ) {
    sorted[i] =
      values[i];
  }

  sortUint16(
    sorted,
    count
  );

  if (
    (count & 1U) != 0
  ) {
    return
      sorted[count / 2];
  }

  uint8_t right =
    count / 2;

  uint8_t left =
    right - 1;

  return
    (uint16_t)(
      (
        (uint32_t)sorted[left] +
        (uint32_t)sorted[right]
      ) /
      2UL
    );
}

void PulseAPDS9008::clearStableHistory() {
  _ibiIndex = 0;
  _ibiCount = 0;

  for (
    uint8_t i = 0;
    i < IBI_BUFFER_SIZE;
    i++
  ) {
    _ibiBuffer[i] = 0;
  }

  _data.validIbiCount = 0;
}

void PulseAPDS9008::clearAcquisition() {
  _acqCount = 0;

  for (
    uint8_t i = 0;
    i < ACQ_BUFFER_SIZE;
    i++
  ) {
    _acqBuffer[i] = 0;
  }
}

void PulseAPDS9008::clearReacquisition() {
  _reacqCount = 0;

  for (
    uint8_t i = 0;
    i < ACQ_BUFFER_SIZE;
    i++
  ) {
    _reacqBuffer[i] = 0;
  }
}

void PulseAPDS9008::updateQuality(
  uint16_t raw
) {
  bool clipped =
    raw <= _cfg.contactAdcMin ||
    raw >= _cfg.contactAdcMax;

  float clippingNow =
    clipped ? 1.0f : 0.0f;

  _clipLevel +=
    0.05f *
    (clippingNow - _clipLevel);

  float ampRatio =
    _data.envelope /
    _cfg.minPulseAmplitude;

  // Sube hasta ~4x el piso; despues ya no se premia
  // aumentar mas la amplitud.
  float amplitudeScore =
    ampRatio / 4.0f;

  amplitudeScore =
    clampFloat(
      amplitudeScore,
      0.0f,
      1.0f
    );

  float snr =
    _data.envelope /
    (_data.noise + 0.5f);

  float snrScore =
    (snr - 1.2f) /
    5.0f;

  snrScore =
    clampFloat(
      snrScore,
      0.0f,
      1.0f
    );

  float clipScore =
    1.0f - _clipLevel;

  clipScore =
    clampFloat(
      clipScore,
      0.0f,
      1.0f
    );

  float motionPenalty =
    1.0f -
    clampFloat(
      _data.motionScore / 100.0f,
      0.0f,
      1.0f
    );

  float instantQuality =
    (
      0.40f * amplitudeScore +
      0.45f * snrScore +
      0.15f * clipScore
    ) *
    motionPenalty;

  if (!_data.contactLikely) {
    instantQuality *=
      0.35f;
  }

  if (_data.motionDetected) {
    instantQuality = 0.0f;
  }

  _qualityEma +=
    0.10f *
    (instantQuality - _qualityEma);

  _data.quality =
    (uint8_t)(
      clampFloat(
        _qualityEma,
        0.0f,
        1.0f
      ) *
      100.0f +
      0.5f
    );
}

void PulseAPDS9008::updateConfidence() {
  if (
    _data.state !=
      PULSE_STATE_TRACKING ||
    !_data.contactLikely ||
    _data.motionDetected ||
    _ibiCount == 0
  ) {
    _data.confidence = 0;
    _data.measurementValid = false;
    return;
  }

  uint16_t median =
    medianIbi();

  if (median == 0) {
    _data.confidence = 0;
    _data.measurementValid = false;
    return;
  }

  float mean = 0.0f;

  for (
    uint8_t i = 0;
    i < _ibiCount;
    i++
  ) {
    mean +=
      (float)_ibiBuffer[i];
  }

  mean /=
    (float)_ibiCount;

  float variance = 0.0f;

  for (
    uint8_t i = 0;
    i < _ibiCount;
    i++
  ) {
    float d =
      (float)_ibiBuffer[i] -
      mean;

    variance +=
      d * d;
  }

  variance /=
    (float)_ibiCount;

  // Evitamos sqrt() para mantenerlo ligero:
  // usamos desviacion absoluta media normalizada.
  float meanAbsDev = 0.0f;

  for (
    uint8_t i = 0;
    i < _ibiCount;
    i++
  ) {
    meanAbsDev +=
      absFloat(
        (float)_ibiBuffer[i] -
        (float)median
      );
  }

  meanAbsDev /=
    (float)_ibiCount;

  float variability =
    meanAbsDev /
    (float)median;

  float stabilityScore =
    1.0f -
    (variability / 0.18f);

  stabilityScore =
    clampFloat(
      stabilityScore,
      0.0f,
      1.0f
    );

  float historyScore =
    (float)_ibiCount / 5.0f;

  historyScore =
    clampFloat(
      historyScore,
      0.0f,
      1.0f
    );

  float qualityScore =
    (float)_data.quality /
    100.0f;

  float confidence =
    0.50f * stabilityScore +
    0.30f * qualityScore +
    0.20f * historyScore;

  confidence =
    clampFloat(
      confidence,
      0.0f,
      1.0f
    );

  _data.confidence =
    (uint8_t)(
      confidence *
      100.0f +
      0.5f
    );

  _data.measurementValid =
    _data.bpmStable &&
    _data.confidence >= 55;
}


// ============================================================
// ANALIZADOR HIBRIDO v0.5
// ============================================================

void PulseAPDS9008::configureHybridAnalyzer() {
  if (_cfg.hybridDecimation < 1) {
    _cfg.hybridDecimation = 1;
  }

  if (_cfg.hybridSpectralStepBpm < 1) {
    _cfg.hybridSpectralStepBpm = 1;
  }

  if (_cfg.hybridSpectralStepBpm > 4) {
    _cfg.hybridSpectralStepBpm = 4;
  }

  if (_cfg.hybridSliceBudgetUs < 250) {
    _cfg.hybridSliceBudgetUs = 250;
  }

  uint32_t hybridInterval =
    (uint32_t)_cfg.sampleIntervalMs *
    (uint32_t)_cfg.hybridDecimation;

  if (hybridInterval > 1000UL) {
    hybridInterval = 1000UL;
  }

  _hybridSampleIntervalMs =
    (uint16_t)hybridInterval;

  uint32_t target =
    (uint32_t)_cfg.hybridWindowMs /
    (uint32_t)_hybridSampleIntervalMs;

  if (target < 80) {
    target = 80;
  }

  if (target > HYBRID_BUFFER_MAX) {
    target = HYBRID_BUFFER_MAX;
  }

  _hybridTargetSamples =
    (uint16_t)target;

  uint32_t minimum =
    (uint32_t)_cfg.hybridMinWindowMs /
    (uint32_t)_hybridSampleIntervalMs;

  if (minimum < 60) {
    minimum = 60;
  }

  if (minimum > target) {
    minimum = target;
  }

  _hybridMinSamples =
    (uint16_t)minimum;

  float dt =
    (float)_cfg.sampleIntervalMs /
    1000.0f;

  const float PULSE_TWO_PI =
    6.28318530718f;

  if (_cfg.ppgHighpassHz < 0.05f) {
    _cfg.ppgHighpassHz = 0.05f;
  }

  if (_cfg.ppgLowpassHz < 0.5f) {
    _cfg.ppgLowpassHz = 0.5f;
  }

  float hpRc =
    1.0f /
    (
      PULSE_TWO_PI *
      _cfg.ppgHighpassHz
    );

  _ppgHighpassAlpha =
    hpRc /
    (hpRc + dt);

  float lpRc =
    1.0f /
    (
      PULSE_TWO_PI *
      _cfg.ppgLowpassHz
    );

  _ppgLowpassAlpha =
    dt /
    (lpRc + dt);
}

void PulseAPDS9008::resetHybridAnalyzer() {
  _hybridWrite = 0;
  _hybridCount = 0;
  _hybridDecimationCounter = 0;
  _lastHybridAnalysisMs = 0;

  _hybridWorkStage =
    HYBRID_WORK_IDLE;

  _hybridAnalysisInProgress = false;
  _analysisCount = 0;
  _analysisCpuAccumUs = 0;
  _analysisStartedMs = 0;

  _ppgInitialized = false;
  _ppgPreviousInput = 0.0f;
  _ppgHighpass = 0.0f;
  _ppgLowpass = 0.0f;

  _data.ppgFiltered = 0.0f;

  _data.bpmAutocorr = 0;
  _data.autocorrQuality = 0;
  _data.autocorrStrength = 0.0f;

  _data.bpmSpectral = 0;
  _data.bpmSpectralRawPeak = 0;
  _data.spectralQuality = 0;
  _data.spectralDominance = 0.0f;
  _data.spectralSecondHarmonicRatio = 0.0f;

  _data.bpmFused = 0;
  _data.fusionConfidence = 0;
  _data.methodsAgree = 0;
  _data.fusionScore = 0.0f;
  _data.ibiFusionRelation = 0;
  _data.autocorrFusionRelation = 0;
  _data.spectralFusionRelation = 0;

  _data.hybridReady = false;
  _data.hybridUpdated = false;
  _data.hybridBusy = false;
  _data.fusionValid = false;
  _data.harmonicSuspect = false;
  _data.harmonicResolved = false;
  _data.hybridSamples = 0;

  _data.hybridAnalysisUs = 0;
  _data.hybridSliceUs = 0;
  _data.hybridCycleElapsedMs = 0;
}

void PulseAPDS9008::updatePpgBandpass(
  float input
) {
  if (!_ppgInitialized) {
    _ppgPreviousInput = input;
    _ppgHighpass = 0.0f;
    _ppgLowpass = 0.0f;
    _data.ppgFiltered = 0.0f;
    _ppgInitialized = true;
    return;
  }

  _ppgHighpass =
    _ppgHighpassAlpha *
    (
      _ppgHighpass +
      input -
      _ppgPreviousInput
    );

  _ppgPreviousInput = input;

  _ppgLowpass +=
    _ppgLowpassAlpha *
    (
      _ppgHighpass -
      _ppgLowpass
    );

  _data.ppgFiltered =
    _ppgLowpass;
}

void PulseAPDS9008::pushHybridSample(
  float value
) {
  if (_hybridTargetSamples == 0) {
    return;
  }

  _hybridBuffer[
    _hybridWrite
  ] = value;

  _hybridWrite++;

  if (
    _hybridWrite >=
    _hybridTargetSamples
  ) {
    _hybridWrite = 0;
  }

  if (
    _hybridCount <
    _hybridTargetSamples
  ) {
    _hybridCount++;
  }

  _data.hybridSamples =
    _hybridCount;
}

float PulseAPDS9008::hybridSampleAt(
  uint16_t chronologicalIndex
) const {
  if (
    chronologicalIndex >=
    _hybridCount ||
    _hybridCount == 0
  ) {
    return 0.0f;
  }

  uint16_t oldest =
    (
      _hybridWrite +
      _hybridTargetSamples -
      _hybridCount
    ) %
    _hybridTargetSamples;

  uint16_t index =
    oldest +
    chronologicalIndex;

  if (
    index >=
    _hybridTargetSamples
  ) {
    index -=
      _hybridTargetSamples;
  }

  return
    _hybridBuffer[index];
}

void PulseAPDS9008::maybeRunHybridAnalysis(
  uint32_t nowMs
) {
  if (
    !_cfg.hybridEnabled ||
    _hybridCount <
      _hybridMinSamples
  ) {
    _data.hybridReady = false;
    return;
  }

  _data.hybridReady = true;

  if (!_hybridAnalysisInProgress) {
    bool due =
      _lastHybridAnalysisMs == 0 ||
      nowMs -
      _lastHybridAnalysisMs >=
        _cfg.hybridUpdateMs;

    if (due) {
      startHybridAnalysis(nowMs);
    }
  }

  if (_hybridAnalysisInProgress) {
    serviceHybridAnalysis(nowMs);
  }
}

void PulseAPDS9008::startHybridAnalysis(
  uint32_t nowMs
) {
  if (
    _hybridCount <
    _hybridMinSamples
  ) {
    return;
  }

  uint32_t startUs = micros();

  _analysisCount =
    _hybridCount;

  if (
    _analysisCount >
    HYBRID_BUFFER_MAX
  ) {
    _analysisCount =
      HYBRID_BUFFER_MAX;
  }

  float mean = 0.0f;

  for (
    uint16_t i = 0;
    i < _analysisCount;
    i++
  ) {
    float value =
      hybridSampleAt(i);

    _analysisBuffer[i] =
      value;

    mean += value;
  }

  mean /=
    (float)_analysisCount;

  for (
    uint16_t i = 0;
    i < _analysisCount;
    i++
  ) {
    _analysisBuffer[i] -=
      mean;
  }

  for (
    uint16_t i = 0;
    i <= CORR_LAG_MAX;
    i++
  ) {
    _corrScores[i] = 0.0f;
  }

  for (
    uint16_t i = 0;
    i <= SPECTRAL_BPM_MAX;
    i++
  ) {
    _spectralPowers[i] = 0.0f;
  }

  float sampleRate =
    1000.0f /
    (float)_hybridSampleIntervalMs;

  _corrMinLag =
    (uint16_t)(
      sampleRate *
      60.0f /
      (float)_cfg.maxBpm +
      0.5f
    );

  _corrMaxLag =
    (uint16_t)(
      sampleRate *
      60.0f /
      (float)_cfg.minBpm +
      0.5f
    );

  if (_corrMinLag < 2) {
    _corrMinLag = 2;
  }

  uint16_t maxAllowed =
    _analysisCount / 2;

  if (_corrMaxLag > maxAllowed) {
    _corrMaxLag = maxAllowed;
  }

  if (_corrMaxLag > CORR_LAG_MAX) {
    _corrMaxLag = CORR_LAG_MAX;
  }

  _corrCurrentLag =
    _corrMinLag;

  _corrGlobalBest = 0.0f;
  _corrGlobalLag = 0;

  _spectralMinBpm =
    _cfg.minBpm;

  if (_spectralMinBpm < 20) {
    _spectralMinBpm = 20;
  }

  _spectralMaxBpm =
    _cfg.maxBpm;

  if (
    _spectralMaxBpm >
    SPECTRAL_BPM_MAX
  ) {
    _spectralMaxBpm =
      SPECTRAL_BPM_MAX;
  }

  _spectralCurrentBpm =
    _spectralMinBpm;

  _spectralRawBestPower = 0.0f;
  _spectralRawBestBpm = 0;
  _spectralSumPower = 0.0f;
  _spectralPowerCount = 0;

  _hybridWorkStage =
    HYBRID_WORK_CORRELATION;

  _hybridAnalysisInProgress = true;
  _data.hybridBusy = true;
  _data.hybridUpdated = false;

  _analysisStartedMs =
    nowMs;

  _lastHybridAnalysisMs =
    nowMs;

  _analysisCpuAccumUs =
    micros() - startUs;

  uint32_t startupUs =
    _analysisCpuAccumUs;

  if (startupUs > 65535UL) {
    startupUs = 65535UL;
  }

  _data.hybridSliceUs =
    (uint16_t)startupUs;

  if (
    _data.hybridSliceUs >
    _data.hybridSliceMaxUs
  ) {
    _data.hybridSliceMaxUs =
      _data.hybridSliceUs;
  }
}

void PulseAPDS9008::serviceHybridAnalysis(
  uint32_t nowMs
) {
  if (!_hybridAnalysisInProgress) {
    return;
  }

  uint32_t sliceStartUs =
    micros();

  bool keepWorking = true;

  while (keepWorking) {
    switch (_hybridWorkStage) {
      case HYBRID_WORK_CORRELATION:
        if (
          _corrCurrentLag <=
          _corrMaxLag
        ) {
          processOneAutocorrelationLag();
        } else {
          finalizeAutocorrelation();
          _hybridWorkStage =
            HYBRID_WORK_PREPARE_SPECTRUM;
          _spectralCurrentBpm = 0;
        }
        break;

      case HYBRID_WORK_PREPARE_SPECTRUM:
        // Reutilizamos spectralCurrentBpm como indice de preparacion.
        if (
          _spectralCurrentBpm <
          _analysisCount
        ) {
          uint16_t i =
            _spectralCurrentBpm;

          float t =
            (float)i /
            (float)(
              _analysisCount - 1
            );

          float window =
            4.0f *
            t *
            (1.0f - t);

          _analysisBuffer[i] *=
            window;

          _spectralCurrentBpm++;
        } else {
          _spectralCurrentBpm =
            _spectralMinBpm;

          _hybridWorkStage =
            HYBRID_WORK_SPECTRUM;
        }
        break;

      case HYBRID_WORK_SPECTRUM:
        if (
          _spectralCurrentBpm <=
          _spectralMaxBpm
        ) {
          processOneSpectralBin();
        } else {
          finalizeSpectral();
          _hybridWorkStage =
            HYBRID_WORK_FINALIZE;
        }
        break;

      case HYBRID_WORK_FINALIZE:
        // Finalizamos despues de contabilizar la CPU de esta
        // ultima rebanada, para que la telemetria sea exacta.
        keepWorking = false;
        break;

      default:
        _hybridAnalysisInProgress = false;
        _data.hybridBusy = false;
        keepWorking = false;
        break;
    }

    if (!keepWorking) {
      break;
    }

    uint32_t elapsed =
      micros() -
      sliceStartUs;

    if (
      elapsed >=
      (uint32_t)_cfg.hybridSliceBudgetUs
    ) {
      break;
    }
  }

  uint32_t sliceUs =
    micros() -
    sliceStartUs;

  _analysisCpuAccumUs +=
    sliceUs;

  if (sliceUs > 65535UL) {
    sliceUs = 65535UL;
  }

  _data.hybridSliceUs =
    (uint16_t)sliceUs;

  if (
    _data.hybridSliceUs >
    _data.hybridSliceMaxUs
  ) {
    _data.hybridSliceMaxUs =
      _data.hybridSliceUs;
  }

  if (
    _hybridAnalysisInProgress &&
    _hybridWorkStage ==
      HYBRID_WORK_FINALIZE
  ) {
    finishHybridAnalysis(nowMs);
  }
}

void PulseAPDS9008::processOneAutocorrelationLag() {
  uint16_t lag =
    _corrCurrentLag;

  float sumXY = 0.0f;
  float sumXX = 0.0f;
  float sumYY = 0.0f;

  for (
    uint16_t i = lag;
    i < _analysisCount;
    i++
  ) {
    float x =
      _analysisBuffer[i];

    float y =
      _analysisBuffer[
        i - lag
      ];

    sumXY += x * y;
    sumXX += x * x;
    sumYY += y * y;
  }

  float score = 0.0f;

  if (
    sumXY > 0.0f &&
    sumXX > 0.0001f &&
    sumYY > 0.0001f
  ) {
    score =
      (
        sumXY *
        sumXY
      ) /
      (
        sumXX *
        sumYY
      );
  }

  score =
    clampFloat(
      score,
      0.0f,
      1.0f
    );

  _corrScores[lag] =
    score;

  if (
    score >
    _corrGlobalBest
  ) {
    _corrGlobalBest =
      score;

    _corrGlobalLag =
      lag;
  }

  _corrCurrentLag++;
}

void PulseAPDS9008::finalizeAutocorrelation() {
  _data.bpmAutocorr = 0;
  _data.autocorrQuality = 0;
  _data.autocorrStrength = 0.0f;

  if (
    _corrGlobalLag == 0 ||
    _corrGlobalBest <= 0.0f
  ) {
    return;
  }

  float localFloor =
    _corrGlobalBest *
    0.80f;

  const float MINIMUM_USEFUL =
    0.0625f; // 0.25^2

  if (
    localFloor <
    MINIMUM_USEFUL
  ) {
    localFloor =
      MINIMUM_USEFUL;
  }

  uint16_t selectedLag =
    _corrGlobalLag;

  // IMPORTANTE v0.5.3:
  // elegimos el primer pico local plausible. Ya NO existe la
  // antigua regla que reemplazaba T por 2T cuando 2T era mayor;
  // esa regla produjo ~43 BPM para una señal real ~85 BPM.
  for (
    uint16_t lag =
      _corrMinLag + 1;
    lag < _corrMaxLag;
    lag++
  ) {
    bool localPeak =
      _corrScores[lag] >=
        _corrScores[lag - 1] &&
      _corrScores[lag] >=
        _corrScores[lag + 1];

    if (
      localPeak &&
      _corrScores[lag] >=
        localFloor
    ) {
      selectedLag = lag;
      break;
    }
  }

  float selectedLagFloat =
    (float)selectedLag;

  if (
    selectedLag > _corrMinLag &&
    selectedLag < _corrMaxLag
  ) {
    float y1 =
      _corrScores[selectedLag - 1];

    float y2 =
      _corrScores[selectedLag];

    float y3 =
      _corrScores[selectedLag + 1];

    float denominator =
      y1 -
      2.0f * y2 +
      y3;

    if (
      absFloat(denominator) >
      0.000001f
    ) {
      float delta =
        0.5f *
        (y1 - y3) /
        denominator;

      if (
        delta >= -1.0f &&
        delta <= 1.0f
      ) {
        selectedLagFloat +=
          delta;
      }
    }
  }

  float strength =
    fastSqrt(
      _corrScores[selectedLag]
    );

  float bpmFloat =
    60000.0f /
    (
      selectedLagFloat *
      (float)_hybridSampleIntervalMs
    );

  if (
    bpmFloat <
      (float)_cfg.minBpm ||
    bpmFloat >
      (float)_cfg.maxBpm
  ) {
    return;
  }

  _data.bpmAutocorr =
    (uint16_t)(
      bpmFloat +
      0.5f
    );

  _data.autocorrStrength =
    strength;

  float q =
    (
      strength -
      0.25f
    ) /
    0.65f;

  q =
    clampFloat(
      q,
      0.0f,
      1.0f
    );

  float fill =
    (float)_analysisCount /
    (float)_hybridTargetSamples;

  fill =
    clampFloat(
      fill,
      0.0f,
      1.0f
    );

  q *= fill;

  _data.autocorrQuality =
    (uint8_t)(
      q *
      100.0f +
      0.5f
    );
}

float PulseAPDS9008::goertzelPowerAnalysis(
  uint16_t bpm
) const {
  if (
    _analysisCount < 2 ||
    bpm == 0
  ) {
    return 0.0f;
  }

  const float PULSE_TWO_PI =
    6.28318530718f;

  float omega =
    PULSE_TWO_PI *
    (float)bpm *
    (float)_hybridSampleIntervalMs /
    60000.0f;

  float coefficient =
    2.0f *
    fastCosSmall(omega);

  float s1 = 0.0f;
  float s2 = 0.0f;

  for (
    uint16_t i = 0;
    i < _analysisCount;
    i++
  ) {
    float s0 =
      _analysisBuffer[i] +
      coefficient *
      s1 -
      s2;

    s2 = s1;
    s1 = s0;
  }

  float power =
    s1 * s1 +
    s2 * s2 -
    coefficient *
    s1 *
    s2;

  if (power < 0.0f) {
    power = 0.0f;
  }

  return power;
}

void PulseAPDS9008::processOneSpectralBin() {
  uint16_t candidate =
    _spectralCurrentBpm;

  float power =
    goertzelPowerAnalysis(
      candidate
    );

  _spectralPowers[candidate] =
    power;

  _spectralSumPower +=
    power;

  _spectralPowerCount++;

  if (
    power >
    _spectralRawBestPower
  ) {
    _spectralRawBestPower =
      power;

    _spectralRawBestBpm =
      candidate;
  }

  uint16_t next =
    candidate +
    _cfg.hybridSpectralStepBpm;

  if (next <= candidate) {
    next = candidate + 1;
  }

  _spectralCurrentBpm =
    next;
}

float PulseAPDS9008::spectralPowerNear(
  uint16_t bpm
) const {
  if (
    bpm < _spectralMinBpm ||
    bpm > _spectralMaxBpm
  ) {
    return 0.0f;
  }

  uint8_t radius =
    _cfg.hybridSpectralStepBpm;

  if (radius < 1) {
    radius = 1;
  }

  float best = 0.0f;

  for (
    int16_t offset =
      -(int16_t)radius;
    offset <=
      (int16_t)radius;
    offset++
  ) {
    int16_t index =
      (int16_t)bpm +
      offset;

    if (
      index <
        (int16_t)_spectralMinBpm ||
      index >
        (int16_t)_spectralMaxBpm
    ) {
      continue;
    }

    float value =
      _spectralPowers[index];

    if (value > best) {
      best = value;
    }
  }

  return best;
}

void PulseAPDS9008::finalizeSpectral() {
  _data.bpmSpectral = 0;
  _data.bpmSpectralRawPeak = 0;
  _data.spectralQuality = 0;
  _data.spectralDominance = 0.0f;
  _data.spectralSecondHarmonicRatio = 0.0f;

  if (
    _spectralRawBestBpm == 0 ||
    _spectralRawBestPower <= 0.0f ||
    _spectralPowerCount == 0
  ) {
    return;
  }

  _data.bpmSpectralRawPeak =
    _spectralRawBestBpm;

  // El analizador espectral mantiene su propia salida, pero la
  // decision f/2, f o 2f se deja principalmente a updateFusion().
  // Aqui solo sumamos evidencia de familia armonica sin forzarla.
  float bestFamilyScore = -1.0f;
  uint16_t bestBpm =
    _spectralRawBestBpm;

  uint16_t step =
    _cfg.hybridSpectralStepBpm;

  if (step < 1) {
    step = 1;
  }

  for (
    uint16_t candidate =
      _spectralMinBpm;
    candidate <=
      _spectralMaxBpm;
    candidate += step
  ) {
    float fundamental =
      _spectralPowers[candidate];

    if (fundamental <= 0.0f) {
      continue;
    }

    // Piso mas permisivo que v0.5.2: una fundamental debil puede
    // coexistir con un segundo armonico dominante en PPG.
    if (
      fundamental <
      _spectralRawBestPower *
      0.08f
    ) {
      continue;
    }

    float score =
      fundamental;

    uint16_t h2 =
      candidate * 2;

    uint16_t h3 =
      candidate * 3;

    if (h2 <= _spectralMaxBpm) {
      score +=
        0.50f *
        spectralPowerNear(h2);
    }

    if (h3 <= _spectralMaxBpm) {
      score +=
        0.15f *
        spectralPowerNear(h3);
    }

    if (
      score >
      bestFamilyScore
    ) {
      bestFamilyScore = score;
      bestBpm = candidate;
    }

    if (
      _spectralMaxBpm - candidate < step
    ) {
      break;
    }
  }

  float bpmFloat =
    (float)bestBpm;

  // Interpolacion usando el paso real del barrido.
  if (
    bestBpm >=
      _spectralMinBpm + step &&
    bestBpm + step <=
      _spectralMaxBpm
  ) {
    float y1 =
      _spectralPowers[
        bestBpm - step
      ];

    float y2 =
      _spectralPowers[
        bestBpm
      ];

    float y3 =
      _spectralPowers[
        bestBpm + step
      ];

    float denominator =
      y1 -
      2.0f * y2 +
      y3;

    if (
      absFloat(denominator) >
      0.000001f
    ) {
      float deltaBins =
        0.5f *
        (y1 - y3) /
        denominator;

      if (
        deltaBins >= -1.0f &&
        deltaBins <= 1.0f
      ) {
        bpmFloat +=
          deltaBins *
          (float)step;
      }
    }
  }

  _data.bpmSpectral =
    (uint16_t)(
      bpmFloat +
      0.5f
    );

  float averagePower =
    _spectralSumPower /
    (float)_spectralPowerCount;

  float selectedPower =
    spectralPowerNear(
      bestBpm
    );

  _data.spectralDominance =
    selectedPower /
    (averagePower + 0.000001f);

  uint16_t second =
    bestBpm * 2;

  if (
    second <= _spectralMaxBpm &&
    selectedPower > 0.000001f
  ) {
    _data.spectralSecondHarmonicRatio =
      spectralPowerNear(second) /
      selectedPower;
  }

  float q =
    (
      _data.spectralDominance -
      2.0f
    ) /
    12.0f;

  q =
    clampFloat(
      q,
      0.0f,
      1.0f
    );

  float fill =
    (float)_analysisCount /
    (float)_hybridTargetSamples;

  fill =
    clampFloat(
      fill,
      0.0f,
      1.0f
    );

  q *= fill;

  _data.spectralQuality =
    (uint8_t)(
      q *
      100.0f +
      0.5f
    );
}

void PulseAPDS9008::finishHybridAnalysis(
  uint32_t nowMs
) {
  updateFusion();

  _hybridAnalysisInProgress = false;
  _hybridWorkStage =
    HYBRID_WORK_IDLE;

  _data.hybridBusy = false;
  _data.hybridUpdated = true;

  _data.hybridAnalysisUs =
    _analysisCpuAccumUs;

  if (
    _data.hybridAnalysisUs >
    _data.hybridAnalysisMaxUs
  ) {
    _data.hybridAnalysisMaxUs =
      _data.hybridAnalysisUs;
  }

  uint32_t elapsedMs =
    nowMs -
    _analysisStartedMs;

  if (elapsedMs > 65535UL) {
    elapsedMs = 65535UL;
  }

  _data.hybridCycleElapsedMs =
    (uint16_t)elapsedMs;
}

bool PulseAPDS9008::bpmAgreement(
  uint16_t a,
  uint16_t b
) const {
  if (
    a == 0 ||
    b == 0
  ) {
    return false;
  }

  float average =
    (
      (float)a +
      (float)b
    ) *
    0.5f;

  float tolerance =
    average *
    _cfg.fusionToleranceFraction;

  if (
    tolerance <
    (float)_cfg.fusionToleranceBpm
  ) {
    tolerance =
      (float)_cfg.fusionToleranceBpm;
  }

  return
    absFloat(
      (float)a -
      (float)b
    ) <= tolerance;
}

uint8_t PulseAPDS9008::relationToHypothesis(
  uint16_t observedBpm,
  uint16_t hypothesisBpm,
  float& supportFactor
) const {
  supportFactor = 0.0f;

  if (
    observedBpm == 0 ||
    hypothesisBpm == 0
  ) {
    return 0;
  }

  const float RELATION_FACTOR[3] = {
    0.5f,
    1.0f,
    2.0f
  };

  // Fundamental vale 100%; sub/segundo armonico conservan 82%.
  const float RELATION_WEIGHT[3] = {
    0.82f,
    1.00f,
    0.82f
  };

  uint8_t bestCode = 0;
  float bestSupport = 0.0f;

  for (
    uint8_t i = 0;
    i < 3;
    i++
  ) {
    float expected =
      (float)hypothesisBpm *
      RELATION_FACTOR[i];

    float tolerance =
      expected *
      _cfg.fusionToleranceFraction;

    if (
      tolerance <
      (float)_cfg.fusionToleranceBpm
    ) {
      tolerance =
        (float)_cfg.fusionToleranceBpm;
    }

    float difference =
      absFloat(
        (float)observedBpm -
        expected
      );

    if (difference > tolerance) {
      continue;
    }

    float closeness =
      1.0f -
      difference /
      tolerance;

    closeness =
      clampFloat(
        closeness,
        0.0f,
        1.0f
      );

    float support =
      RELATION_WEIGHT[i] *
      (
        0.65f +
        0.35f *
        closeness
      );

    if (support > bestSupport) {
      bestSupport = support;
      bestCode = i + 1;
    }
  }

  supportFactor =
    bestSupport;

  return bestCode;
}

void PulseAPDS9008::updateFusion() {
  _data.methodsAgree = 0;
  _data.fusionValid = false;
  _data.harmonicSuspect = false;
  _data.harmonicResolved = false;
  _data.fusionScore = 0.0f;
  _data.ibiFusionRelation = 0;
  _data.autocorrFusionRelation = 0;
  _data.spectralFusionRelation = 0;

  bool contextValid =
    _data.contactLikely &&
    !_data.motionDetected;

  if (!contextValid) {
    _data.bpmFused = 0;
    _data.fusionConfidence = 0;
    return;
  }

  bool ibiAvailable =
    _data.bpmIbi > 0 &&
    _data.ibiMethodQuality > 0;

  bool corrAvailable =
    _data.bpmAutocorr > 0 &&
    _data.autocorrQuality >=
      _cfg.autocorrMinQuality;

  bool specAvailable =
    _data.bpmSpectral > 0 &&
    _data.spectralQuality >=
      _cfg.spectralMinQuality;

  if (
    !ibiAvailable &&
    !corrAvailable &&
    !specAvailable
  ) {
    _data.bpmFused = 0;
    _data.fusionConfidence = 0;
    return;
  }

  // Maximo 3 metodos x {2x,1x,0.5x} = 9 hipotesis.
  uint16_t hypotheses[9] = {0};
  uint8_t hypothesisCount = 0;

  uint16_t observations[3] = {
    (uint16_t)(
      ibiAvailable
        ? _data.bpmIbi
        : 0
    ),
    (uint16_t)(
      corrAvailable
        ? _data.bpmAutocorr
        : 0
    ),
    (uint16_t)(
      specAvailable
        ? _data.bpmSpectral
        : 0
    )
  };

  for (
    uint8_t method = 0;
    method < 3;
    method++
  ) {
    uint16_t observed =
      observations[method];

    if (observed == 0) {
      continue;
    }

    uint16_t candidates[3] = {
      observed,
      (uint16_t)(
        observed <= 32767
          ? observed * 2
          : 65535
      ),
      (uint16_t)(
        observed / 2
      )
    };

    for (
      uint8_t c = 0;
      c < 3;
      c++
    ) {
      uint16_t candidate =
        candidates[c];

      if (
        candidate < _cfg.minBpm ||
        candidate > _cfg.maxBpm
      ) {
        continue;
      }

      bool duplicate = false;

      for (
        uint8_t i = 0;
        i < hypothesisCount;
        i++
      ) {
        if (
          absFloat(
            (float)hypotheses[i] -
            (float)candidate
          ) <= 1.0f
        ) {
          duplicate = true;
          break;
        }
      }

      if (
        !duplicate &&
        hypothesisCount < 9
      ) {
        hypotheses[
          hypothesisCount++
        ] = candidate;
      }
    }
  }

  uint8_t bestMethods = 0;
  uint8_t bestFundamentals = 0;
  float bestScore = -1.0f;
  uint16_t bestHypothesis = 0;
  uint8_t bestRelations[3] = {0,0,0};
  float bestSupports[3] = {0.0f,0.0f,0.0f};

  uint8_t qualities[3] = {
    _data.ibiMethodQuality,
    _data.autocorrQuality,
    _data.spectralQuality
  };

  bool available[3] = {
    ibiAvailable,
    corrAvailable,
    specAvailable
  };

  for (
    uint8_t h = 0;
    h < hypothesisCount;
    h++
  ) {
    uint16_t hypothesis =
      hypotheses[h];

    uint8_t supportMethods = 0;
    uint8_t fundamentalMethods = 0;
    float score = 0.0f;
    uint8_t relations[3] = {0,0,0};
    float supports[3] = {0.0f,0.0f,0.0f};

    for (
      uint8_t method = 0;
      method < 3;
      method++
    ) {
      if (!available[method]) {
        continue;
      }

      float supportFactor = 0.0f;

      uint8_t relation =
        relationToHypothesis(
          observations[method],
          hypothesis,
          supportFactor
        );

      if (relation == 0) {
        continue;
      }

      relations[method] = relation;
      supports[method] = supportFactor;
      supportMethods++;

      if (relation == 2) {
        fundamentalMethods++;
      }

      score +=
        (float)qualities[method] *
        supportFactor;
    }

    bool better = false;

    // Primero manda redundancia: 3 metodos > 2 > 1.
    if (supportMethods > bestMethods) {
      better = true;
    } else if (
      supportMethods == bestMethods &&
      score > bestScore + 0.01f
    ) {
      better = true;
    } else if (
      supportMethods == bestMethods &&
      absFloat(score - bestScore) <= 0.01f &&
      fundamentalMethods > bestFundamentals
    ) {
      better = true;
    }

    if (better) {
      bestMethods = supportMethods;
      bestFundamentals = fundamentalMethods;
      bestScore = score;
      bestHypothesis = hypothesis;

      for (
        uint8_t i = 0;
        i < 3;
        i++
      ) {
        bestRelations[i] =
          relations[i];

        bestSupports[i] =
          supports[i];
      }
    }
  }

  if (
    bestMethods < 2 ||
    bestHypothesis == 0
  ) {
    _data.bpmFused = 0;
    _data.fusionConfidence = 0;
    return;
  }

  // Convertimos cada observacion respaldante a la escala de la
  // hipotesis antes de promediar. Ej.: 43@0.5x -> 86 BPM.
  float weightedBpm = 0.0f;
  float weightSum = 0.0f;
  float confidenceSum = 0.0f;

  for (
    uint8_t method = 0;
    method < 3;
    method++
  ) {
    uint8_t relation =
      bestRelations[method];

    if (relation == 0) {
      continue;
    }

    float relationFactor = 1.0f;

    if (relation == 1) {
      relationFactor = 0.5f;
    } else if (relation == 3) {
      relationFactor = 2.0f;
    }

    float normalizedBpm =
      (float)observations[method] /
      relationFactor;

    float methodWeight =
      (float)qualities[method] *
      bestSupports[method];

    weightedBpm +=
      normalizedBpm *
      methodWeight;

    weightSum +=
      methodWeight;

    confidenceSum +=
      methodWeight;
  }

  if (weightSum <= 0.0f) {
    _data.bpmFused = 0;
    _data.fusionConfidence = 0;
    return;
  }

  float fusedFloat =
    weightedBpm /
    weightSum;

  _data.bpmFused =
    (uint16_t)(
      fusedFloat +
      0.5f
    );

  _data.methodsAgree =
    bestMethods;

  _data.fusionScore =
    bestScore;

  _data.ibiFusionRelation =
    bestRelations[0];

  _data.autocorrFusionRelation =
    bestRelations[1];

  _data.spectralFusionRelation =
    bestRelations[2];

  _data.harmonicSuspect =
    (
      bestRelations[0] != 0 &&
      bestRelations[0] != 2
    ) ||
    (
      bestRelations[1] != 0 &&
      bestRelations[1] != 2
    ) ||
    (
      bestRelations[2] != 0 &&
      bestRelations[2] != 2
    );

  float confidence =
    confidenceSum /
    (float)bestMethods;

  if (bestMethods == 3) {
    confidence += 10.0f;
  }

  confidence =
    clampFloat(
      confidence,
      0.0f,
      100.0f
    );

  _data.fusionConfidence =
    (uint8_t)(
      confidence +
      0.5f
    );

  _data.fusionValid =
    _data.fusionConfidence >=
      _cfg.fusionMinConfidence;

  _data.harmonicResolved =
    _data.fusionValid &&
    _data.harmonicSuspect;
}

void PulseAPDS9008::applyLedPower() {
  analogWrite(
    _cfg.ledPin,
    _data.ledPower
  );
}

float PulseAPDS9008::absFloat(
  float value
) {
  return
    value < 0.0f
      ? -value
      : value;
}

float PulseAPDS9008::fastSqrt(
  float value
) {
  if (value <= 0.0f) {
    return 0.0f;
  }

  float x =
    value >= 1.0f
      ? value
      : 1.0f;

  for (
    uint8_t i = 0;
    i < 7;
    i++
  ) {
    x =
      0.5f *
      (
        x +
        value / x
      );
  }

  return x;
}

float PulseAPDS9008::fastCosSmall(
  float radians
) {
  // Para nuestro rango fisiologico @100 Hz:
  // omega aprox 0.04..0.23 rad.
  // Incluso con otros sampleInterval razonables sigue siendo
  // suficientemente preciso para Goertzel.
  float x2 =
    radians *
    radians;

  float x4 =
    x2 *
    x2;

  float x6 =
    x4 *
    x2;

  return
    1.0f -
    x2 * 0.5f +
    x4 * 0.0416666667f -
    x6 * 0.0013888889f;
}

float PulseAPDS9008::clampFloat(
  float value,
  float minimum,
  float maximum
) {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}

void PulseAPDS9008::sortUint16(
  uint16_t* values,
  uint8_t count
) {
  for (
    uint8_t i = 1;
    i < count;
    i++
  ) {
    uint16_t key =
      values[i];

    int8_t j =
      (int8_t)i - 1;

    while (
      j >= 0 &&
      values[j] > key
    ) {
      values[j + 1] =
        values[j];

      j--;
    }

    values[j + 1] =
      key;
  }
}
