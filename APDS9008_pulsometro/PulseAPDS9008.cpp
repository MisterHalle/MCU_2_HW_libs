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

/*
  PulseAPDS9008.cpp
  Halle / ZATA - APDS-9008 Pulsometria v0.3.0
*/

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

  if (
    nowMs - _lastSampleMs <
    _cfg.sampleIntervalMs
  ) {
    return false;
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

  _cycleValley = 0.0f;

  cancelCandidate();

  clearStableHistory();
  clearAcquisition();
  clearReacquisition();

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

void PulseAPDS9008::processSample(
  uint16_t raw,
  uint32_t nowMs
) {
  _data.beat = false;
  _data.ibiAccepted = false;
  _data.raw = raw;

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

  if (
    refractoryFinished &&
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

  bool opticalVariation =
    _data.envelope >=
    contactEnvelopeFloor;

  bool cleanEnough =
    !_data.motionDetected &&
    _data.motionScore < 70.0f;

  bool evidence =
    adcUsable &&
    opticalVariation &&
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

    clearAcquisition();
    clearReacquisition();

    _consecutiveOutliers = 0;

    // La medicion anterior deja de ser publicable.
    _data.bpm = 0;
    _data.ibiMs = 0;
    _data.robustIbiMs = 0;
    _data.bpmStable = false;

    clearStableHistory();
  }

  if (
    newState ==
      PULSE_STATE_ACQUIRING
  ) {
    _lastDetectedBeatMs = 0;
    _lastAcceptedBeatMs = 0;

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

  _lastSignalEvidenceMs =
    peakMs;

  _data.contactLikely = true;

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

  if (
    coherent >=
    _cfg.acquisitionMinIbi
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