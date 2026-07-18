#include "PulseOximeter-MAX30102.h"
#include <math.h>

// MAX30102 WristFilter Rebased OpticalGate v1.1.3

bool PulseOximeter::begin(TwoWire& wirePort) {
  Config defaultCfg;
  return begin(defaultCfg, wirePort);
}

bool PulseOximeter::begin(const Config& cfg, TwoWire& wirePort) {
  wire = &wirePort;
  wire->setClock(400000);

  wire->beginTransmission(MAX30102_I2C_ADDR);
  if (wire->endTransmission() != 0) {
    return false;
  }

  if (readPartID() != MAX30102_EXPECTED_PART_ID) {
    return false;
  }

  if (!reset()) {
    return false;
  }

  (void)readStatus1();
  (void)readStatus2();

  if (!writeRegister(MAX30102_REG_INTR_ENABLE_1, 0x00)) return false;
  if (!writeRegister(MAX30102_REG_INTR_ENABLE_2, 0x00)) return false;

  if (!applyConfig(cfg)) {
    return false;
  }

  refreshFilterCoefficients();
  resetVitalsProcessing(true);
  return true;
}

bool PulseOximeter::reset() {
  if (!writeRegister(MAX30102_REG_MODE_CONFIG, 0x40)) {
    return false;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
    uint8_t value = 0;
    if (!readRegister(MAX30102_REG_MODE_CONFIG, value)) {
      return false;
    }

    if ((value & 0x40) == 0) {
      return clearFIFO();
    }

    delay(1);
  }

  return false;
}

bool PulseOximeter::applyConfig(const Config& cfg) {
  currentConfig = cfg;

  if (!setFIFOConfig(cfg.fifoAverage, cfg.fifoRollover, cfg.fifoAlmostFull)) {
    return false;
  }

  if (!writeRegister(MAX30102_REG_MODE_CONFIG, cfg.mode & 0x07)) {
    return false;
  }

  if (!setSpO2Config(cfg.adcRange, cfg.sampleRate, cfg.pulseWidth)) {
    return false;
  }

  if (!setLedPulseAmplitude(cfg.redLedPA, cfg.irLedPA)) {
    return false;
  }

  refreshFilterCoefficients();
  resetVitalsProcessing(false);
  return clearFIFO();
}

bool PulseOximeter::clearFIFO() {
  if (!writeRegister(MAX30102_REG_FIFO_WR_PTR, 0x00)) return false;
  if (!writeRegister(MAX30102_REG_OVF_COUNTER, 0x00)) return false;
  if (!writeRegister(MAX30102_REG_FIFO_RD_PTR, 0x00)) return false;

  sampleClockValid = false;
  return true;
}

uint8_t PulseOximeter::readPartID() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_PART_ID, value)) return 0;
  return value;
}

uint8_t PulseOximeter::readRevisionID() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_REV_ID, value)) return 0;
  return value;
}

uint8_t PulseOximeter::readStatus1() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_INTR_STATUS_1, value)) return 0;
  return value;
}

uint8_t PulseOximeter::readStatus2() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_INTR_STATUS_2, value)) return 0;
  return value;
}

uint8_t PulseOximeter::readOverflowCounter() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_OVF_COUNTER, value)) return 0;
  return value & 0x1F;
}

uint8_t PulseOximeter::availableSamples() {
  uint8_t wr = 0;
  uint8_t rd = 0;
  uint8_t ovf = 0;

  if (!readRegister(MAX30102_REG_FIFO_WR_PTR, wr)) return 0;
  if (!readRegister(MAX30102_REG_FIFO_RD_PTR, rd)) return 0;
  if (!readRegister(MAX30102_REG_OVF_COUNTER, ovf)) return 0;

  wr &= 0x1F;
  rd &= 0x1F;
  ovf &= 0x1F;

  if (wr == rd) {
    return (ovf > 0) ? 32 : 0;
  }

  return (wr - rd) & 0x1F;
}

bool PulseOximeter::readFIFORaw(uint32_t& red, uint32_t& ir) {
  wire->beginTransmission(MAX30102_I2C_ADDR);
  wire->write(MAX30102_REG_FIFO_DATA);
  if (wire->endTransmission(false) != 0) {
    return false;
  }

  if (wire->requestFrom((uint8_t)MAX30102_I2C_ADDR, (uint8_t)6) != 6) {
    return false;
  }

  red = ((uint32_t)wire->read() << 16);
  red |= ((uint32_t)wire->read() << 8);
  red |= (uint32_t)wire->read();

  ir = ((uint32_t)wire->read() << 16);
  ir |= ((uint32_t)wire->read() << 8);
  ir |= (uint32_t)wire->read();

  red &= 0x03FFFF;
  ir &= 0x03FFFF;
  return true;
}

bool PulseOximeter::readSample(Sample& out) {
  if (availableSamples() == 0) {
    return false;
  }

  uint32_t red = 0;
  uint32_t ir = 0;
  if (!readFIFORaw(red, ir)) {
    return false;
  }

  // Evita que un Sample reutilizado conserve flags de la iteración anterior.
  out = Sample();
  out.red = red;
  out.ir = ir;

  uint32_t periodUs = getSamplePeriodUs();
  uint32_t nowUs = micros();

  if (!sampleClockValid) {
    lastSampleTimestampUs = nowUs;
    sampleClockValid = true;
  } else {
    lastSampleTimestampUs += periodUs;

    // Re-sincroniza solo si la estimación se separó demasiado del reloj real.
    int32_t clockError = (int32_t)(nowUs - lastSampleTimestampUs);
    if (clockError > 1000000L || clockError < -1000000L) {
      lastSampleTimestampUs = nowUs;
    }
  }

  out.timestampMs = lastSampleTimestampUs / 1000UL;
  return true;
}

size_t PulseOximeter::readAvailableSamples(Sample* buffer, size_t maxSamples) {
  if (buffer == NULL || maxSamples == 0) {
    return 0;
  }

  uint8_t totalAvailable = availableSamples();
  if (totalAvailable == 0) return 0;

  uint8_t n = totalAvailable;
  if (n > maxSamples) n = (uint8_t)maxSamples;

  size_t count = 0;
  while (count < n) {
    if (!readFIFORaw(buffer[count].red, buffer[count].ir)) {
      break;
    }

    buffer[count].redFiltered = 0.0f;
    buffer[count].irFiltered = 0.0f;
    buffer[count].adaptiveThreshold = 0.0f;
    buffer[count].pulseQuality = 0.0f;
    buffer[count].beatDetected = false;
    count++;
  }

  if (count == 0) return 0;

  const uint32_t periodUs = getSamplePeriodUs();
  const uint32_t nowUs = micros();
  uint32_t firstTimestampUs = 0;

  if (!sampleClockValid) {
    // El primer elemento leído es el más antiguo del FIFO. Si quedaron más
    // muestras pendientes, también se incluyen al estimar su antigüedad.
    firstTimestampUs = nowUs - (uint32_t)(totalAvailable - 1) * periodUs;
    sampleClockValid = true;
  } else {
    firstTimestampUs = lastSampleTimestampUs + periodUs;

    uint32_t estimatedLastUs = firstTimestampUs + (uint32_t)(count - 1) * periodUs;
    int32_t clockError = (int32_t)(nowUs - estimatedLastUs);

    // Mantiene continuidad normal del FIFO, pero se recupera de pausas largas.
    if (clockError > 1000000L || clockError < -1000000L) {
      firstTimestampUs = nowUs - (uint32_t)(totalAvailable - 1) * periodUs;
    }
  }

  for (size_t i = 0; i < count; i++) {
    uint32_t tsUs = firstTimestampUs + (uint32_t)i * periodUs;
    buffer[i].timestampMs = tsUs / 1000UL;
  }

  lastSampleTimestampUs = firstTimestampUs + (uint32_t)(count - 1) * periodUs;
  return count;
}

bool PulseOximeter::setLedPulseAmplitude(uint8_t redPA, uint8_t irPA) {
  if (!writeRegister(MAX30102_REG_LED1_PA, redPA)) return false;
  if (!writeRegister(MAX30102_REG_LED2_PA, irPA)) return false;

  currentConfig.redLedPA = redPA;
  currentConfig.irLedPA = irPA;
  return true;
}

bool PulseOximeter::setSpO2Config(uint8_t adcRange, uint8_t sampleRate, uint8_t pulseWidth) {
  adcRange &= 0x03;
  sampleRate &= 0x07;
  pulseWidth &= 0x03;

  uint8_t value = (uint8_t)((adcRange << 5) | (sampleRate << 2) | pulseWidth);
  if (!writeRegister(MAX30102_REG_SPO2_CONFIG, value)) return false;

  currentConfig.adcRange = adcRange;
  currentConfig.sampleRate = sampleRate;
  currentConfig.pulseWidth = pulseWidth;
  refreshFilterCoefficients();
  sampleClockValid = false;
  return true;
}

bool PulseOximeter::setFIFOConfig(uint8_t fifoAverage, bool rollover, uint8_t almostFull) {
  fifoAverage &= 0x07;
  almostFull &= 0x0F;

  uint8_t value = (uint8_t)(fifoAverage << 5);
  if (rollover) value |= 0x10;
  value |= almostFull;

  if (!writeRegister(MAX30102_REG_FIFO_CONFIG, value)) return false;

  currentConfig.fifoAverage = fifoAverage;
  currentConfig.fifoRollover = rollover;
  currentConfig.fifoAlmostFull = almostFull;
  refreshFilterCoefficients();
  sampleClockValid = false;
  return true;
}

bool PulseOximeter::setSampleAveraging(uint8_t fifoAverage) {
  return setFIFOConfig(fifoAverage & 0x07,
                       currentConfig.fifoRollover,
                       currentConfig.fifoAlmostFull);
}

bool PulseOximeter::setFIFORollover(bool enable) {
  return setFIFOConfig(currentConfig.fifoAverage,
                       enable,
                       currentConfig.fifoAlmostFull);
}

bool PulseOximeter::setFIFOAlmostFull(uint8_t almostFull) {
  return setFIFOConfig(currentConfig.fifoAverage,
                       currentConfig.fifoRollover,
                       almostFull & 0x0F);
}

float PulseOximeter::getRawSampleRateHz() const {
  static const float rates[8] = {
    50.0f, 100.0f, 200.0f, 400.0f,
    800.0f, 1000.0f, 1600.0f, 3200.0f
  };
  return rates[currentConfig.sampleRate & 0x07];
}

float PulseOximeter::getEffectiveSampleRateHz() const {
  static const uint8_t averages[8] = {1, 2, 4, 8, 16, 32, 32, 32};
  float effective = getRawSampleRateHz() / (float)averages[currentConfig.fifoAverage & 0x07];
  if (effective < 1.0f) effective = 1.0f;
  return effective;
}

uint32_t PulseOximeter::getSamplePeriodUs() const {
  float fs = getEffectiveSampleRateHz();
  if (fs < 1.0f) fs = 1.0f;
  return (uint32_t)(1000000.0f / fs + 0.5f);
}

bool PulseOximeter::writeRegister(uint8_t reg, uint8_t value) {
  if (wire == NULL) return false;

  wire->beginTransmission(MAX30102_I2C_ADDR);
  wire->write(reg);
  wire->write(value);
  return (wire->endTransmission() == 0);
}

bool PulseOximeter::readRegister(uint8_t reg, uint8_t& value) {
  if (wire == NULL) return false;

  wire->beginTransmission(MAX30102_I2C_ADDR);
  wire->write(reg);
  if (wire->endTransmission(false) != 0) return false;

  if (wire->requestFrom((uint8_t)MAX30102_I2C_ADDR, (uint8_t)1) != 1) return false;

  value = wire->read();
  return true;
}

// -----------------------------------------------------------------------------
// Autoexposición
// -----------------------------------------------------------------------------

bool PulseOximeter::setAdcRange(uint8_t adcRange) {
  return setSpO2Config(adcRange & 0x03,
                       currentConfig.sampleRate,
                       currentConfig.pulseWidth);
}

void PulseOximeter::setExposureTargets(uint32_t lowCounts, uint32_t highCounts) {
  if (lowCounts < highCounts) {
    exposureLowTarget = lowCounts;
    exposureHighTarget = highCounts;
  }
}

void PulseOximeter::setFingerThreshold(uint32_t thresholdCounts) {
  fingerThreshold = thresholdCounts;
}

bool PulseOximeter::autoAdjustExposure(const Sample* samples, size_t count) {
  if (!autoExposureEnabled || samples == NULL || count == 0) return false;

  unsigned long now = millis();
  if (now < adjustLockUntilMs) return false;
  if (freezeAutoExposureWhenUsable && signalUsable) return false;
  if ((now - lastExposureChangeMs) < exposureSettleMs) return false;

  uint64_t sumRed = 0;
  uint64_t sumIr = 0;
  uint32_t maxRed = 0;
  uint32_t maxIr = 0;

  for (size_t i = 0; i < count; i++) {
    sumRed += samples[i].red;
    sumIr += samples[i].ir;
    if (samples[i].red > maxRed) maxRed = samples[i].red;
    if (samples[i].ir > maxIr) maxIr = samples[i].ir;
  }

  uint32_t meanRed = (uint32_t)(sumRed / count);
  uint32_t meanIr = (uint32_t)(sumIr / count);

  if (meanIr < fingerThreshold) return false;

  uint32_t meanLevel = (meanRed > meanIr) ? meanRed : meanIr;
  uint32_t peakLevel = (maxRed > maxIr) ? maxRed : maxIr;

  bool tooHigh = (peakLevel > 250000UL) || (meanLevel > exposureHighTarget);
  bool tooLow = (meanLevel < exposureLowTarget);
  bool inBand = !tooHigh && !tooLow;

  if (inBand) {
    badExposureWindows = 0;
    if (goodExposureWindows < 255) goodExposureWindows++;
  } else {
    goodExposureWindows = 0;
    if (badExposureWindows < 255) badExposureWindows++;
  }

  bool changed = false;

  if (!inBand) {
    if (badExposureWindows < 3) return false;

    if (tooHigh) {
      if (currentConfig.adcRange < 0x03) {
        changed = setAdcRange(currentConfig.adcRange + 1);
      } else {
        uint8_t newRed = (currentConfig.redLedPA > 0x02) ? (currentConfig.redLedPA - 0x02) : 0x00;
        uint8_t newIr = (currentConfig.irLedPA > 0x02) ? (currentConfig.irLedPA - 0x02) : 0x00;
        changed = setLedPulseAmplitude(newRed, newIr);
      }
    } else if (tooLow) {
      if (currentConfig.adcRange > 0x00) {
        changed = setAdcRange(currentConfig.adcRange - 1);
      } else {
        uint8_t newRed = (currentConfig.redLedPA < 0x3F) ? (currentConfig.redLedPA + 0x02) : 0x3F;
        uint8_t newIr = (currentConfig.irLedPA < 0x3F) ? (currentConfig.irLedPA + 0x02) : 0x3F;
        changed = setLedPulseAmplitude(newRed, newIr);
      }
    }
  } else {
    // Ahora sí puede ejecutarse la reducción lenta de corriente dentro de banda.
    bool enoughMargin = meanLevel > (exposureLowTarget + ledBackoffMargin);
    bool canLowerLed = (currentConfig.redLedPA > minLedPA) &&
                       (currentConfig.irLedPA > minLedPA);
    bool slowOptimizeReady = (now - lastExposureChangeMs) >= exposureOptimizeMs;

    if (goodExposureWindows >= 10 && enoughMargin && canLowerLed && slowOptimizeReady) {
      changed = setLedPulseAmplitude(currentConfig.redLedPA - 1,
                                     currentConfig.irLedPA - 1);
    }
  }

  if (changed) {
    clearFIFO();
    lastExposureChangeMs = now;
    adjustLockUntilMs = now + 400;
    stableSampleCount = 0;
    exposureStable = false;
    prevDCHistoryValid = false;
    dcSettled = false;
    badExposureWindows = 0;
    goodExposureWindows = 0;
    resetVitalsProcessing(false);
  }

  return changed;
}

bool PulseOximeter::processSamples(Sample* samples, size_t count) {
  if (samples == NULL || count == 0) return false;

  beatDetectedInLastProcess = false;

  bool adjusted = false;
  if (autoExposureEnabled) {
    adjusted = autoAdjustExposure(samples, count);
  }

  updateSignalIndicators(samples, count);

  // Las muestras que provocaron un cambio de exposición pertenecen a la
  // configuración anterior. No se usan para detectar latidos.
  if (adjusted) {
    for (size_t i = 0; i < count; i++) {
      samples[i].redFiltered = 0.0f;
      samples[i].irFiltered = 0.0f;
      samples[i].adaptiveThreshold = 0.0f;
      samples[i].pulseQuality = 0.0f;
      samples[i].candidateDetected = false;
      samples[i].beatDetected = false;
      samples[i].beatIntervalMs = 0;
      samples[i].pulsePolarity = 0;
      samples[i].opticalSignalUsable = false;
      samples[i].rhythmSignalUsable = false;
      samples[i].motionArtifact = false;
      samples[i].rejectCode = REJECT_OPTICAL;
    }
    return true;
  }

  if (vitalsEnabled) {
    updateVitals(samples, count);
  }

  return adjusted;
}

void PulseOximeter::setLedBackoffMargin(uint32_t counts) {
  ledBackoffMargin = counts;
}

void PulseOximeter::setMinLedPulseAmplitude(uint8_t paCode) {
  minLedPA = paCode & 0x3F;
}

// -----------------------------------------------------------------------------
// Calidad base de señal
// -----------------------------------------------------------------------------

void PulseOximeter::setFingerThresholds(uint32_t onCounts, uint32_t offCounts) {
  if (offCounts < onCounts) {
    fingerOnThreshold = onCounts;
    fingerOffThreshold = offCounts;
  }
}

void PulseOximeter::setSaturationThreshold(uint32_t counts) {
  saturationThreshold = counts;
}

void PulseOximeter::updateSignalIndicators(const Sample* samples, size_t count) {
  if (samples == NULL || count == 0) return;

  uint64_t sumRed = 0;
  uint64_t sumIr = 0;
  uint32_t maxRed = 0;
  uint32_t maxIr = 0;

  for (size_t i = 0; i < count; i++) {
    sumRed += samples[i].red;
    sumIr += samples[i].ir;
    if (samples[i].red > maxRed) maxRed = samples[i].red;
    if (samples[i].ir > maxIr) maxIr = samples[i].ir;
  }

  redDC = (uint32_t)(sumRed / count);
  irDC = (uint32_t)(sumIr / count);
  redPeak = maxRed;
  irPeak = maxIr;

  if (!fingerPresent) {
    if (irDC >= fingerOnThreshold) fingerPresent = true;
  } else {
    if (irDC <= fingerOffThreshold) fingerPresent = false;
  }

  uint8_t st1 = readStatus1();
  alcOverflow = ((st1 & 0x20) != 0);
  saturated = (redPeak >= saturationThreshold) || (irPeak >= saturationThreshold);
  fifoOverflow = (readOverflowCounter() != 0);

  bool baseValid = fingerPresent && !alcOverflow && !saturated && !fifoOverflow;

  uint32_t meanLevel = (redDC > irDC) ? redDC : irDC;
  exposureInRange = (meanLevel >= exposureLowTarget) && (meanLevel <= exposureHighTarget);

  if (!prevDCHistoryValid) {
    dcSettled = false;
    prevDCHistoryValid = true;
  } else {
    uint32_t redDiff = (redDC > prevRedDC) ? (redDC - prevRedDC) : (prevRedDC - redDC);
    uint32_t irDiff = (irDC > prevIrDC) ? (irDC - prevIrDC) : (prevIrDC - irDC);
    dcSettled = (redDiff <= redDCSettleTol) && (irDiff <= irDCSettleTol);
  }

  prevRedDC = redDC;
  prevIrDC = irDC;

  unsigned long now = millis();
  bool exposureQuiet = (!autoExposureEnabled) ||
                       ((now - lastExposureChangeMs) >= exposureSettleMs);

  bool configChanged =
      (currentConfig.adcRange != lastStableAdcRange) ||
      (currentConfig.redLedPA != lastStableRedPA) ||
      (currentConfig.irLedPA != lastStableIrPA);

  bool stableConditions =
      baseValid &&
      exposureQuiet &&
      exposureInRange &&
      dcSettled &&
      !configChanged;

  if (!stableConditions) {
    stableSampleCount = 0;
    exposureStable = false;

    lastStableAdcRange = currentConfig.adcRange;
    lastStableRedPA = currentConfig.redLedPA;
    lastStableIrPA = currentConfig.irLedPA;
  } else {
    if (stableSampleCount < 1000000UL) {
      stableSampleCount += (uint32_t)count;
    }
    exposureStable = (stableSampleCount >= usableStableSamples);
  }

  signalUsable = stableConditions && exposureStable;
}

void PulseOximeter::setUsableStableSamples(uint32_t samples) {
  if (samples > 0) usableStableSamples = samples;
}

void PulseOximeter::setDCSettleTolerance(uint32_t redTol, uint32_t irTol) {
  redDCSettleTol = redTol;
  irDCSettleTol = irTol;
}

// -----------------------------------------------------------------------------
// Filtro PPG y métricas de calidad
// -----------------------------------------------------------------------------

void PulseOximeter::setPulseFilter(float highPassHz, float lowPassHz, float dcTrackHz) {
  float fs = getEffectiveSampleRateHz();
  float nyquist = fs * 0.5f;

  if (highPassHz > 0.05f && highPassHz < nyquist) {
    filterHighPassHz = highPassHz;
  }
  if (lowPassHz > filterHighPassHz && lowPassHz < nyquist) {
    filterLowPassHz = lowPassHz;
  }
  if (dcTrackHz > 0.01f && dcTrackHz < filterHighPassHz) {
    filterDCTrackHz = dcTrackHz;
  }

  refreshFilterCoefficients();
  resetVitalsProcessing(false);
}

void PulseOximeter::setPulseQualityThreshold(float minimumQuality) {
  pulseQualityMinimum = clampFloat(minimumQuality, 0.0f, 100.0f);
}

void PulseOximeter::setMinimumPulseRms(float minimumRmsPpm) {
  if (minimumRmsPpm > 0.0f) minimumPulseRmsPpm = minimumRmsPpm;
}

void PulseOximeter::setBeatRefractory(unsigned long refractoryMs) {
  if (refractoryMs >= 250 && refractoryMs <= 600) {
    beatRefractoryMs = refractoryMs;
  }
}

void PulseOximeter::setBeatHysteresis(float rearmFactor) {
  if (rearmFactor >= 0.10f && rearmFactor <= 0.90f) {
    beatRearmFactor = rearmFactor;
  }
}

void PulseOximeter::setRhythmAcquisition(uint8_t requiredIntervals,
                                         float tolerancePercent) {
  if (requiredIntervals >= 2 && requiredIntervals <= 4) {
    acquisitionIntervalsRequired = requiredIntervals;
  }

  float tolerance = tolerancePercent;
  if (tolerancePercent > 1.0f) tolerance = tolerancePercent / 100.0f;
  if (tolerance >= 0.10f && tolerance <= 0.40f) {
    rhythmIntervalTolerance = tolerance;
  }
}

void PulseOximeter::setMinimumRhythmConfidence(float confidence) {
  minimumRhythmConfidence = clampFloat(confidence, 40.0f, 98.0f);
}

void PulseOximeter::setMinimumMorphologyConfidence(float confidence) {
  minimumMorphologyConfidence = clampFloat(confidence, 20.0f, 95.0f);
}

void PulseOximeter::setProvisionalRhythm(float confidence,
                                         uint8_t requiredIntervals,
                                         unsigned long referenceMemoryMs) {
  provisionalConfidenceMinimum = clampFloat(confidence, 50.0f, 99.0f);
  if (requiredIntervals >= 2 && requiredIntervals <= 3) {
    provisionalIntervalsRequired = requiredIntervals;
  }
  if (referenceMemoryMs >= 3000UL && referenceMemoryMs <= 60000UL) {
    rhythmReferenceMemoryMs = referenceMemoryMs;
  }
}

void PulseOximeter::setMissedBeatRecovery(uint8_t maximumMultiple,
                                           float tolerancePercent) {
  // v1.1.3 restringe la recuperación a un único pulso omitido.
  if (maximumMultiple >= 1 && maximumMultiple <= 2) {
    maximumMissedBeatMultiple = maximumMultiple;
  }
  float tolerance = tolerancePercent;
  if (tolerancePercent > 1.0f) tolerance = tolerancePercent / 100.0f;
  if (tolerance >= 0.08f && tolerance <= 0.30f) {
    missedBeatRecoveryTolerance = tolerance;
  }
}

void PulseOximeter::setFastReacquireThreshold(float shortWindowSeconds,
                                               float spanMultiplier) {
  if (shortWindowSeconds >= 0.50f && shortWindowSeconds <= 1.20f) {
    fastThresholdWindowSeconds = shortWindowSeconds;
  }
  if (spanMultiplier >= 1.10f && spanMultiplier <= 1.80f) {
    fastThresholdSpanMultiplier = spanMultiplier;
  }
}

void PulseOximeter::enableOpticalMotionGate(bool enable) {
  opticalMotionGateEnabled = enable;
  uint32_t now = lastProcessedSampleTimestampMs;
  if (now == 0) now = millis();
  motionEvidenceCount = 0;
  quietCandidateSinceMs = 0;
  if (enable) {
    acquisitionState = STATE_SETTLING;
    quietStable = false;
    stateChangedMs = now;
    resetRhythmLock(true, true);
  } else {
    motionArtifact = false;
    quietStable = true;
    acquisitionState = (pulsePolarity != 0) ? STATE_TRACKING : STATE_ACQUIRING;
  }
}

void PulseOximeter::setOpticalMotionGate(float redMotionPercent,
                                          float irMotionPercent,
                                          float redQuietPercent,
                                          float irQuietPercent,
                                          unsigned long settleMs,
                                          float windowSeconds) {
  if (redMotionPercent >= 0.30f && redMotionPercent <= 5.0f) {
    redMotionThresholdPercent = redMotionPercent;
  }
  if (irMotionPercent >= 0.30f && irMotionPercent <= 8.0f) {
    irMotionThresholdPercent = irMotionPercent;
  }
  if (redQuietPercent >= 0.10f && redQuietPercent < redMotionThresholdPercent) {
    redQuietThresholdPercent = redQuietPercent;
  }
  if (irQuietPercent >= 0.10f && irQuietPercent < irMotionThresholdPercent) {
    irQuietThresholdPercent = irQuietPercent;
  }
  if (settleMs >= 300UL && settleMs <= 2000UL) quietSettleMs = settleMs;
  if (windowSeconds >= 0.50f && windowSeconds <= 1.50f) {
    motionWindowSeconds = windowSeconds;
  }
}

void PulseOximeter::setHeartRateRange(float minimumBpm, float maximumBpm) {
  if (minimumBpm >= 30.0f && maximumBpm <= 240.0f &&
      minimumBpm < maximumBpm) {
    minimumHeartRateBpm = minimumBpm;
    maximumHeartRateBpm = maximumBpm;
  }
}

void PulseOximeter::refreshFilterCoefficients() {
  float fs = getEffectiveSampleRateHz();
  if (fs < 1.0f) fs = 1.0f;

  const float twoPi = 6.28318530718f;
  float dt = 1.0f / fs;

  float rcHP = 1.0f / (twoPi * filterHighPassHz);
  hpAlpha = rcHP / (rcHP + dt);

  float rcLP = 1.0f / (twoPi * filterLowPassHz);
  lpAlpha = dt / (rcLP + dt);

  dcAlpha = 1.0f - expf(-twoPi * filterDCTrackHz / fs);
  dcAlpha = clampFloat(dcAlpha, 0.0001f, 1.0f);
}

float PulseOximeter::median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

float PulseOximeter::clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float PulseOximeter::filterChannel(ChannelFilterState& state, uint32_t raw) {
  float rawF = (float)raw;
  float medianValue = rawF;

  if (!state.medianReady) {
    state.median1 = rawF;
    state.median2 = rawF;
    state.medianReady = true;
  } else {
    medianValue = median3(state.median2, state.median1, rawF);
    state.median2 = state.median1;
    state.median1 = rawF;
  }

  if (!state.dcReady) {
    state.dc = medianValue;
    state.dcReady = true;
    state.hpPrevX = 0.0f;
    state.hpPrevY = 0.0f;
    state.lp1 = 0.0f;
    state.lp2 = 0.0f;
    return 0.0f;
  }

  state.dc += dcAlpha * (medianValue - state.dc);
  float safeDC = (state.dc > 1.0f) ? state.dc : 1.0f;

  // AC normalizada en partes por millón. Reduce dependencia de presión/LED.
  float normalizedPpm = ((medianValue - state.dc) / safeDC) * 1000000.0f;

  float hp = hpAlpha * (state.hpPrevY + normalizedPpm - state.hpPrevX);
  state.hpPrevX = normalizedPpm;
  state.hpPrevY = hp;

  // Dos pasa-bajos de primer orden en cascada: filtrado suave y económico.
  state.lp1 += lpAlpha * (hp - state.lp1);
  state.lp2 += lpAlpha * (state.lp1 - state.lp2);

  return state.lp2;
}

void PulseOximeter::resetVitalsProcessing(bool clearDisplayedValues) {
  redFilter = ChannelFilterState();
  irFilter = ChannelFilterState();

  lastFilteredRed = 0.0f;
  lastFilteredIR = 0.0f;
  filteredPrev2 = 0.0f;
  filteredPrev1 = 0.0f;
  filteredPrev2Ms = 0;
  filteredPrev1Ms = 0;
  filteredHistoryCount = 0;

  adaptiveThreshold = 0.0f;
  pulseQuality = 0.0f;
  pulseRmsPpm = 0.0f;
  perfusionIndexPercent = 0.0f;
  redIrCorrelation = 0.0f;
  redIrGain = 0.0f;
  redIrGainBaseline = 0.0f;
  redIrGainDeviation = 0.0f;
  redIrGainReady = false;
  morphologyConfidence = 0.0f;

  opticalSignalUsable = false;
  rhythmSignalUsable = false;
  pulseSignalUsable = false;
  motionArtifact = false;
  motionArtifactUntilMs = 0;
  acquisitionStartedMs = 0;
  lastCandidateActivityMs = 0;
  redMotionRangePercent = 0.0f;
  irMotionRangePercent = 0.0f;
  quietCandidateSinceMs = 0;
  motionEvidenceCount = 0;
  lastProcessedSampleTimestampMs = 0;
  quietStable = !opticalMotionGateEnabled;
  acquisitionState = opticalMotionGateEnabled ? STATE_SETTLING : STATE_ACQUIRING;
  stateChangedMs = 0;

  beatDetectedInLastProcess = false;
  pulsePolarity = 0;
  rhythmConfidence = 0.0f;
  rhythmRejectStreak = 0;
  lastRejectCode = REJECT_NONE;
  lastBeatIntervalMs = 0;
  lastRecoveredBeatMultiple = 1;
  previousAcceptedBeatWasRecovered = false;
  positiveTracker = PeakTracker();
  negativeTracker = PeakTracker();

  vitalsHead = 0;
  vitalsCount = 0;
  lastBeatMs = 0;
  rejectedBeatCount = 0;
  beatIntervalCount = 0;
  beatIntervalIndex = 0;
  for (uint8_t i = 0; i < BEAT_INTERVAL_BUF_SIZE; i++) beatIntervalBuf[i] = 0;

  prevSignalUsable = false;
  hrFastBeatsRemaining = 3;
  heartRateValid = false;
  heartRateHeld = false;
  heartRateProvisional = false;
  provisionalHeartRateBpm = 0.0f;
  lastProvisionalHeartRateMs = 0;

  if (clearDisplayedValues) {
    heartRateBpm = 0.0f;
    spo2Percent = 0.0f;
    rValue = 0.0f;
    lastValidHeartRateMs = 0;
    referenceIntervalMs = 0;
    lastReliableRhythmMs = 0;
    lastReliableHeartRateBpm = 0.0f;
  }
}

void PulseOximeter::updatePulseMetrics() {
  if (vitalsCount < 16) {
    pulseQuality = 0.0f;
    morphologyConfidence = 0.0f;
    opticalSignalUsable = false;
    rhythmSignalUsable = false;
    pulseSignalUsable = false;
    return;
  }

  uint16_t window = vitalsCount;
  uint16_t maxWindow = (uint16_t)getEffectiveSampleRateHz() * 2U;
  if (maxWindow < 32) maxWindow = 32;
  if (maxWindow > VITALS_BUF_SIZE) maxWindow = VITALS_BUF_SIZE;
  if (window > maxWindow) window = maxWindow;

  uint16_t shortWindow = (uint16_t)(getEffectiveSampleRateHz() *
                                    fastThresholdWindowSeconds + 0.5f);
  if (shortWindow < 20) shortWindow = 20;
  if (shortWindow > window) shortWindow = window;

  uint16_t motionWindow = (uint16_t)(getEffectiveSampleRateHz() *
                                     motionWindowSeconds + 0.5f);
  if (motionWindow < 20) motionWindow = 20;
  if (motionWindow > window) motionWindow = window;

  float minIr = 1.0e30f;
  float maxIr = -1.0e30f;
  float minIrShort = 1.0e30f;
  float maxIrShort = -1.0e30f;
  uint32_t minRawRed = 0xFFFFFFFFUL;
  uint32_t maxRawRed = 0;
  uint32_t minRawIr = 0xFFFFFFFFUL;
  uint32_t maxRawIr = 0;
  uint64_t sumRawRedMotion = 0;
  uint64_t sumRawIrMotion = 0;
  double sumIr = 0.0;
  double sumRed = 0.0;
  double sumIr2 = 0.0;
  double sumRed2 = 0.0;
  double sumCross = 0.0;
  double sumDiff2 = 0.0;
  float previousIr = 0.0f;
  bool previousValid = false;
  uint64_t sumRawIr = 0;

  for (uint16_t i = 0; i < window; i++) {
    uint16_t idx = (vitalsHead + VITALS_BUF_SIZE - window + i) % VITALS_BUF_SIZE;
    float ir = irFilteredBuf[idx];
    float red = redFilteredBuf[idx];

    if (ir < minIr) minIr = ir;
    if (ir > maxIr) maxIr = ir;
    if (i >= (uint16_t)(window - shortWindow)) {
      if (ir < minIrShort) minIrShort = ir;
      if (ir > maxIrShort) maxIrShort = ir;
    }

    if (i >= (uint16_t)(window - motionWindow)) {
      uint32_t rawRed = redBuf[idx];
      uint32_t rawIr = irBuf[idx];
      if (rawRed < minRawRed) minRawRed = rawRed;
      if (rawRed > maxRawRed) maxRawRed = rawRed;
      if (rawIr < minRawIr) minRawIr = rawIr;
      if (rawIr > maxRawIr) maxRawIr = rawIr;
      sumRawRedMotion += rawRed;
      sumRawIrMotion += rawIr;
    }

    sumIr += ir;
    sumRed += red;
    sumIr2 += (double)ir * ir;
    sumRed2 += (double)red * red;
    sumCross += (double)ir * red;
    sumRawIr += irBuf[idx];

    if (previousValid) {
      float d = ir - previousIr;
      sumDiff2 += (double)d * d;
    }
    previousIr = ir;
    previousValid = true;
  }

  float meanIr = (float)(sumIr / window);
  float meanRed = (float)(sumRed / window);
  float varIr = (float)(sumIr2 / window - (double)meanIr * meanIr);
  float varRed = (float)(sumRed2 / window - (double)meanRed * meanRed);
  if (varIr < 0.0f) varIr = 0.0f;
  if (varRed < 0.0f) varRed = 0.0f;

  float rmsIr = sqrtf(varIr);
  pulseRmsPpm = rmsIr;

  float covariance = (float)(sumCross / window - (double)meanIr * meanRed);
  float correlationDen = sqrtf(varIr * varRed);
  redIrCorrelation = (correlationDen > 0.001f) ? (covariance / correlationDen) : 0.0f;
  redIrCorrelation = clampFloat(redIrCorrelation, -1.0f, 1.0f);

  redIrGain = (varIr > 0.001f) ? (covariance / varIr) : 0.0f;
  if (!redIrGainReady && window >= 32) {
    redIrGainBaseline = redIrGain;
    redIrGainDeviation = 0.0f;
    redIrGainReady = true;
  } else if (redIrGainReady) {
    float gainError = fabsf(redIrGain - redIrGainBaseline);
    redIrGainBaseline += 0.02f * (redIrGain - redIrGainBaseline);
    redIrGainDeviation += 0.04f * (gainError - redIrGainDeviation);
  }

  float rawIrMean = (float)sumRawIr / window;
  perfusionIndexPercent = (rawIrMean > 1.0f) ? (rmsIr / 10000.0f) : 0.0f;

  float diffRms = 0.0f;
  if (window > 1) diffRms = sqrtf((float)(sumDiff2 / (window - 1)));
  float roughness = diffRms / (rmsIr + 1.0f);

  float amplitudeScore = clampFloat((rmsIr - 20.0f) / 480.0f, 0.0f, 1.0f) * 40.0f;
  float correlationScore = clampFloat(fabsf(redIrCorrelation), 0.0f, 1.0f) * 35.0f;
  float smoothnessScore = clampFloat((1.8f - roughness) / 1.3f, 0.0f, 1.0f) * 25.0f;
  pulseQuality = amplitudeScore + correlationScore + smoothnessScore;
  updateMorphologyConfidence();

  float rawRedMean = (motionWindow > 0)
                       ? (float)sumRawRedMotion / motionWindow : 0.0f;
  float rawIrMotionMean = (motionWindow > 0)
                            ? (float)sumRawIrMotion / motionWindow : 0.0f;
  redMotionRangePercent = (rawRedMean > 1.0f)
                            ? 100.0f * (float)(maxRawRed - minRawRed) / rawRedMean
                            : 0.0f;
  irMotionRangePercent = (rawIrMotionMean > 1.0f)
                           ? 100.0f * (float)(maxRawIr - minRawIr) / rawIrMotionMean
                           : 0.0f;

  float spanLong = maxIr - minIr;
  float spanForThreshold = spanLong;
  // La ventana corta solo se habilita cuando la compuerta ya confirmó quietud.
  if (acquisitionState == STATE_ACQUIRING && quietStable) {
    float spanShort = maxIrShort - minIrShort;
    float limited = spanShort * fastThresholdSpanMultiplier;
    if (limited > 0.0f && limited < spanForThreshold) spanForThreshold = limited;
  }
  adaptiveThreshold = hbPeakFactor * spanForThreshold;
  if (adaptiveThreshold < hbPeakMin) adaptiveThreshold = hbPeakMin;

  opticalSignalUsable = signalUsable &&
                        (pulseRmsPpm >= minimumPulseRmsPpm) &&
                        (pulseQuality >= pulseQualityMinimum);

  uint32_t now = lastProcessedSampleTimestampMs;
  if (now == 0) now = millis();
  updateOpticalMotionGate(now);

  rhythmSignalUsable = opticalSignalUsable &&
                       (pulsePolarity != 0) &&
                       heartRateValid &&
                       (beatIntervalCount >= acquisitionIntervalsRequired) &&
                       (rhythmConfidence >= minimumRhythmConfidence) &&
                       (morphologyConfidence >= minimumMorphologyConfidence) &&
                       !motionArtifact &&
                       (!opticalMotionGateEnabled || acquisitionState == STATE_TRACKING);

  pulseSignalUsable = opticalSignalUsable && rhythmSignalUsable;
}

// -----------------------------------------------------------------------------
// Pulsómetro / oxímetro
// -----------------------------------------------------------------------------

void PulseOximeter::setSpO2Coefficients(float a, float b, float c) {
  spo2A = a;
  spo2B = b;
  spo2C = c;
}

void PulseOximeter::updateVitals(Sample* samples, size_t count) {
  if (samples == NULL || count == 0) return;

  uint32_t now = samples[count - 1].timestampMs;
  if (now == 0) now = millis();

  if (!fingerPresent) {
    resetVitalsProcessing(true);
    return;
  }

  if (!signalUsable) {
    updateHeartRateState(now);
    opticalSignalUsable = false;
    rhythmSignalUsable = false;
    pulseSignalUsable = false;

    for (size_t i = 0; i < count; i++) {
      samples[i].redFiltered = 0.0f;
      samples[i].irFiltered = 0.0f;
      samples[i].adaptiveThreshold = adaptiveThreshold;
      samples[i].pulseQuality = pulseQuality;
      samples[i].candidateDetected = false;
      samples[i].beatDetected = false;
      samples[i].beatIntervalMs = 0;
      samples[i].pulsePolarity = pulsePolarity;
      samples[i].opticalSignalUsable = false;
      samples[i].rhythmSignalUsable = false;
      samples[i].motionArtifact = motionArtifact;
      samples[i].quietStable = quietStable;
      samples[i].heartRateProvisional = heartRateProvisional;
      samples[i].acquisitionState = (uint8_t)acquisitionState;
      samples[i].recoveredBeatMultiple = 1;
      samples[i].temporalConfidence = rhythmConfidence;
      samples[i].morphologyConfidence = morphologyConfidence;
      samples[i].perfusionIndexPercent = perfusionIndexPercent;
      samples[i].redIrCorrelation = redIrCorrelation;
      samples[i].redIrGain = redIrGain;
      samples[i].redIrGainDeviation = redIrGainDeviation;
      samples[i].redMotionRangePercent = redMotionRangePercent;
      samples[i].irMotionRangePercent = irMotionRangePercent;
      samples[i].heartRateBpm = heartRateBpm;
      samples[i].provisionalHeartRateBpm = provisionalHeartRateBpm;
      samples[i].heartRateValid = heartRateValid;
      samples[i].heartRateHeld = heartRateHeld;
      samples[i].referenceBeatIntervalMs = (uint16_t)referenceIntervalMs;
      samples[i].rejectCode = REJECT_OPTICAL;
    }

    prevSignalUsable = false;
    return;
  }

  if (!prevSignalUsable) {
    redFilter = ChannelFilterState();
    irFilter = ChannelFilterState();
    vitalsHead = 0;
    vitalsCount = 0;
    filteredHistoryCount = 0;
    resetRhythmLock(true, true);
    acquisitionStartedMs = now;
    if (opticalMotionGateEnabled) {
      acquisitionState = STATE_SETTLING;
      quietStable = false;
      quietCandidateSinceMs = now;
    }
  }

  prevSignalUsable = true;

  for (size_t i = 0; i < count; i++) processVitalSample(samples[i]);

  updatePulseMetrics();
  updateHeartRateState(now);

  rhythmSignalUsable = opticalSignalUsable &&
                       (pulsePolarity != 0) &&
                       heartRateValid &&
                       (beatIntervalCount >= acquisitionIntervalsRequired) &&
                       (rhythmConfidence >= minimumRhythmConfidence) &&
                       (morphologyConfidence >= minimumMorphologyConfidence) &&
                       !motionArtifact &&
                       (!opticalMotionGateEnabled || acquisitionState == STATE_TRACKING);
  pulseSignalUsable = opticalSignalUsable && rhythmSignalUsable;

  if (rhythmSignalUsable) computeSpO2FromBuffer();
}

void PulseOximeter::processVitalSample(Sample& sample) {
  sample.candidateDetected = false;
  sample.beatDetected = false;
  sample.beatIntervalMs = 0;
  sample.recoveredBeatMultiple = 1;
  sample.rejectCode = REJECT_NONE;

  lastProcessedSampleTimestampMs = sample.timestampMs;
  float redFiltered = filterChannel(redFilter, sample.red);
  float irFiltered = filterChannel(irFilter, sample.ir);

  lastFilteredRed = redFiltered;
  lastFilteredIR = irFiltered;

  redBuf[vitalsHead] = sample.red;
  irBuf[vitalsHead] = sample.ir;
  redFilteredBuf[vitalsHead] = redFiltered;
  irFilteredBuf[vitalsHead] = irFiltered;

  vitalsHead = (vitalsHead + 1) % VITALS_BUF_SIZE;
  if (vitalsCount < VITALS_BUF_SIZE) vitalsCount++;

  updatePulseMetrics();
  bool beat = detectBeat(irFiltered, sample.timestampMs, sample);

  sample.redFiltered = redFiltered;
  sample.irFiltered = irFiltered;
  sample.adaptiveThreshold = adaptiveThreshold;
  sample.pulseQuality = pulseQuality;
  sample.beatDetected = beat;
  sample.pulsePolarity = pulsePolarity;
  sample.opticalSignalUsable = opticalSignalUsable;
  sample.rhythmSignalUsable = rhythmSignalUsable;
  sample.motionArtifact = motionArtifact;
  sample.quietStable = quietStable;
  sample.heartRateProvisional = heartRateProvisional;
  sample.acquisitionState = (uint8_t)acquisitionState;
  sample.temporalConfidence = rhythmConfidence;
  sample.morphologyConfidence = morphologyConfidence;
  sample.redMotionRangePercent = redMotionRangePercent;
  sample.irMotionRangePercent = irMotionRangePercent;
  sample.perfusionIndexPercent = perfusionIndexPercent;
  sample.redIrCorrelation = redIrCorrelation;
  sample.redIrGain = redIrGain;
  sample.redIrGainDeviation = redIrGainDeviation;
  sample.heartRateBpm = heartRateBpm;
  sample.provisionalHeartRateBpm = provisionalHeartRateBpm;
  sample.heartRateValid = heartRateValid;
  sample.heartRateHeld = heartRateHeld;
  sample.referenceBeatIntervalMs = (uint16_t)referenceIntervalMs;

  if (beat) beatDetectedInLastProcess = true;
}

bool PulseOximeter::updatePeakTracker(PeakTracker& tracker, int8_t polarity,
                                      float x0, float x1, float x2,
                                      float highThreshold,
                                      float lowThreshold) {
  float y0 = (float)polarity * x0;
  float y1 = (float)polarity * x1;
  float y2 = (float)polarity * x2;

  bool candidate = tracker.armed &&
                   (y1 > y0) &&
                   (y1 >= y2) &&
                   (y1 >= highThreshold);

  if (candidate) {
    tracker.armed = false;
    return true;
  }

  if (!tracker.armed && y2 <= lowThreshold) {
    tracker.armed = true;
  }

  return false;
}

bool PulseOximeter::detectBeat(float currentFiltered,
                               uint32_t currentTimestampMs,
                               Sample& sample) {
  if (filteredHistoryCount < 2) {
    if (filteredHistoryCount == 0) {
      filteredPrev1 = currentFiltered;
      filteredPrev1Ms = currentTimestampMs;
      filteredHistoryCount = 1;
    } else {
      filteredPrev2 = filteredPrev1;
      filteredPrev2Ms = filteredPrev1Ms;
      filteredPrev1 = currentFiltered;
      filteredPrev1Ms = currentTimestampMs;
      filteredHistoryCount = 2;
    }
    return false;
  }

  float x0 = filteredPrev2;
  float x1 = filteredPrev1;
  float x2 = currentFiltered;
  uint32_t candidateTimestampMs = filteredPrev1Ms;

  filteredPrev2 = filteredPrev1;
  filteredPrev2Ms = filteredPrev1Ms;
  filteredPrev1 = currentFiltered;
  filteredPrev1Ms = currentTimestampMs;

  if (!opticalSignalUsable) {
    sample.rejectCode = REJECT_OPTICAL;
    lastRejectCode = REJECT_OPTICAL;
    return false;
  }

  if (opticalMotionGateEnabled && acquisitionState == STATE_MOVING) {
    sample.rejectCode = REJECT_MOTION;
    lastRejectCode = REJECT_MOTION;
    return false;
  }
  if (opticalMotionGateEnabled && acquisitionState == STATE_SETTLING) {
    sample.rejectCode = REJECT_SETTLING;
    lastRejectCode = REJECT_SETTLING;
    return false;
  }

  float highThreshold = adaptiveThreshold;
  float lowThreshold = adaptiveThreshold * beatRearmFactor;
  bool positiveCandidate = false;
  bool negativeCandidate = false;

  if (pulsePolarity == 0 || pulsePolarity > 0) {
    positiveCandidate = updatePeakTracker(positiveTracker, 1,
                                          x0, x1, x2,
                                          highThreshold, lowThreshold);
  }
  if (pulsePolarity == 0 || pulsePolarity < 0) {
    negativeCandidate = updatePeakTracker(negativeTracker, -1,
                                          x0, x1, x2,
                                          highThreshold, lowThreshold);
  }
  if (!positiveCandidate && !negativeCandidate) return false;

  sample.candidateDetected = true;
  lastCandidateActivityMs = candidateTimestampMs;

  if (pulsePolarity == 0) {
    bool accepted = false;
    if (positiveCandidate) {
      accepted = processAcquisitionCandidate(positiveTracker, 1,
                                             candidateTimestampMs, sample);
    }
    if (!accepted && pulsePolarity == 0 && negativeCandidate) {
      accepted = processAcquisitionCandidate(negativeTracker, -1,
                                             candidateTimestampMs, sample);
    }
    return accepted;
  }

  return processLockedCandidate(candidateTimestampMs, sample);
}

bool PulseOximeter::processAcquisitionCandidate(PeakTracker& tracker,
                                                int8_t polarity,
                                                uint32_t candidateTimestampMs,
                                                Sample& sample) {
  if (acquisitionStartedMs == 0) acquisitionStartedMs = candidateTimestampMs;

  if (tracker.lastCandidateMs == 0) {
    tracker.lastCandidateMs = candidateTimestampMs;
    sample.rejectCode = REJECT_REACQUIRE;
    lastRejectCode = REJECT_REACQUIRE;
    return false;
  }

  unsigned long intervalMs = candidateTimestampMs - tracker.lastCandidateMs;
  sample.beatIntervalMs = (intervalMs <= 65535UL) ? (uint16_t)intervalMs : 0;

  unsigned long minimumInterval = (unsigned long)(60000.0f / maximumHeartRateBpm);
  unsigned long maximumInterval = (unsigned long)(60000.0f / minimumHeartRateBpm);
  if (minimumInterval < beatRefractoryMs) minimumInterval = beatRefractoryMs;

  if (intervalMs < minimumInterval) {
    // Un hombro secundario no debe reemplazar la referencia temporal.
    sample.rejectCode = REJECT_REFRACTORY;
    lastRejectCode = REJECT_REFRACTORY;
    return false;
  }
  tracker.lastCandidateMs = candidateTimestampMs;
  if (intervalMs > maximumInterval) {
    tracker.acquisitionCount = 0;
    sample.rejectCode = REJECT_INTERVAL_RANGE;
    lastRejectCode = REJECT_INTERVAL_RANGE;
    return false;
  }

  bool coherent = true;
  if (tracker.acquisitionCount > 0) {
    unsigned long reference = medianValues(tracker.acquisitionIntervals,
                                           tracker.acquisitionCount);
    if (reference > 0) {
      float error = fabsf((float)intervalMs - (float)reference) /
                    (float)reference;
      coherent = (error <= rhythmIntervalTolerance);
    }
  }

  if (!coherent) {
    tracker.acquisitionCount = 0;
    tracker.acquisitionIntervals[tracker.acquisitionCount++] = intervalMs;
    heartRateProvisional = false;
    provisionalHeartRateBpm = 0.0f;
    sample.rejectCode = REJECT_INCONSISTENT;
    lastRejectCode = REJECT_INCONSISTENT;
    return false;
  }

  if (tracker.acquisitionCount < 4) {
    tracker.acquisitionIntervals[tracker.acquisitionCount++] = intervalMs;
  } else {
    for (uint8_t i = 1; i < 4; i++) {
      tracker.acquisitionIntervals[i - 1] = tracker.acquisitionIntervals[i];
    }
    tracker.acquisitionIntervals[3] = intervalMs;
  }

  updateProvisionalHeartRate(tracker, candidateTimestampMs);

  if (tracker.acquisitionCount < acquisitionIntervalsRequired) {
    sample.rejectCode = REJECT_REACQUIRE;
    lastRejectCode = REJECT_REACQUIRE;
    return false;
  }

  float acquisitionConfidence = computeIntervalConfidence(
      tracker.acquisitionIntervals, tracker.acquisitionCount);
  if (acquisitionConfidence < minimumRhythmConfidence ||
      morphologyConfidence < minimumMorphologyConfidence) {
    sample.rejectCode = REJECT_LOW_CONFIDENCE;
    lastRejectCode = REJECT_LOW_CONFIDENCE;
    return false;
  }

  lockRhythm(polarity, tracker, candidateTimestampMs, sample);
  return true;
}

void PulseOximeter::lockRhythm(int8_t polarity,
                               const PeakTracker& tracker,
                               uint32_t beatTimestampMs,
                               Sample& sample) {
  pulsePolarity = polarity;
  beatIntervalCount = 0;
  beatIntervalIndex = 0;
  uint8_t count = tracker.acquisitionCount;
  if (count > acquisitionIntervalsRequired) count = acquisitionIntervalsRequired;

  for (uint8_t i = 0; i < count; i++) {
    beatIntervalBuf[beatIntervalIndex] = tracker.acquisitionIntervals[i];
    beatIntervalIndex = (beatIntervalIndex + 1) % BEAT_INTERVAL_BUF_SIZE;
    beatIntervalCount++;
  }

  lastBeatMs = beatTimestampMs;
  lastBeatIntervalMs = (uint16_t)medianBeatInterval();
  updateRhythmConfidence();
  unsigned long interval = medianBeatInterval();
  if (interval > 0) heartRateBpm = 60000.0f / (float)interval;

  heartRateValid = true;
  heartRateHeld = false;
  heartRateProvisional = false;
  provisionalHeartRateBpm = 0.0f;
  lastProvisionalHeartRateMs = 0;
  lastValidHeartRateMs = beatTimestampMs;
  referenceIntervalMs = interval;
  lastReliableRhythmMs = beatTimestampMs;
  lastReliableHeartRateBpm = heartRateBpm;
  previousAcceptedBeatWasRecovered = false;
  lastRecoveredBeatMultiple = 1;
  rhythmRejectStreak = 0;
  enterTrackingState(beatTimestampMs);

  rhythmSignalUsable = opticalSignalUsable &&
                       (rhythmConfidence >= minimumRhythmConfidence) &&
                       (morphologyConfidence >= minimumMorphologyConfidence) &&
                       !motionArtifact;
  pulseSignalUsable = opticalSignalUsable && rhythmSignalUsable;

  sample.beatDetected = true;
  sample.beatIntervalMs = lastBeatIntervalMs;
  sample.recoveredBeatMultiple = 1;
  sample.rejectCode = REJECT_NONE;
  lastRejectCode = REJECT_NONE;
}

bool PulseOximeter::processLockedCandidate(uint32_t candidateTimestampMs,
                                           Sample& sample) {
  if (lastBeatMs == 0) {
    lastBeatMs = candidateTimestampMs;
    sample.rejectCode = REJECT_REACQUIRE;
    lastRejectCode = REJECT_REACQUIRE;
    return false;
  }

  unsigned long intervalMs = candidateTimestampMs - lastBeatMs;
  sample.beatIntervalMs = (intervalMs <= 65535UL) ? (uint16_t)intervalMs : 0;
  unsigned long expected = medianBeatInterval();
  if (expected == 0) expected = intervalMs;

  unsigned long minimumInterval = (unsigned long)(60000.0f / maximumHeartRateBpm);
  unsigned long maximumInterval = (unsigned long)(60000.0f / minimumHeartRateBpm);
  if (minimumInterval < beatRefractoryMs) minimumInterval = beatRefractoryMs;

  if (intervalMs < minimumInterval) {
    sample.rejectCode = REJECT_SECONDARY_PEAK;
    lastRejectCode = REJECT_SECONDARY_PEAK;
    return false;
  }

  unsigned long intervalUsed = intervalMs;
  uint8_t recoveredMultiple = 1;
  float normalError = (expected > 0)
                        ? fabsf((float)intervalMs - (float)expected) /
                          (float)expected : 0.0f;
  bool accepted = (normalError <= rhythmIntervalTolerance);

  // Solo una recuperación x2, únicamente en seguimiento quieto y nunca dos
  // recuperaciones consecutivas.
  if (!accepted && maximumMissedBeatMultiple >= 2 &&
      acquisitionState == STATE_TRACKING && quietStable &&
      !previousAcceptedBeatWasRecovered && expected > 0) {
    float ratio = (float)intervalMs / (float)expected;
    if (ratio >= 1.70f && ratio <= 2.30f) {
      unsigned long corrected = intervalMs / 2UL;
      float error = fabsf((float)corrected - (float)expected) /
                    (float)expected;
      if (error <= missedBeatRecoveryTolerance) {
        intervalUsed = corrected;
        recoveredMultiple = 2;
        accepted = true;
      }
    }
  }

  if (intervalUsed < minimumInterval || intervalUsed > maximumInterval) {
    accepted = false;
    sample.rejectCode = REJECT_INTERVAL_RANGE;
    lastRejectCode = REJECT_INTERVAL_RANGE;
  }

  if (!accepted) {
    rhythmRejectStreak++;
    sample.rejectCode = REJECT_INCONSISTENT;
    lastRejectCode = REJECT_INCONSISTENT;
    if (rhythmRejectStreak >= 3) {
      resetRhythmLock(true, true);
      enterAcquiringState(candidateTimestampMs);
    }
    return false;
  }

  rhythmRejectStreak = 0;
  lastBeatMs = candidateTimestampMs;
  lastBeatIntervalMs = (intervalUsed <= 65535UL) ? (uint16_t)intervalUsed : 0;
  lastRecoveredBeatMultiple = recoveredMultiple;
  previousAcceptedBeatWasRecovered = (recoveredMultiple > 1);
  acceptBeatInterval(intervalUsed, candidateTimestampMs);
  updateRhythmConfidence();

  if (rhythmConfidence < minimumRhythmConfidence ||
      morphologyConfidence < minimumMorphologyConfidence ||
      motionArtifact || !quietStable) {
    heartRateValid = false;
    heartRateHeld = (lastReliableHeartRateBpm > 0.0f);
    sample.rejectCode = REJECT_LOW_CONFIDENCE;
    lastRejectCode = REJECT_LOW_CONFIDENCE;
    return false;
  }

  heartRateValid = true;
  heartRateHeld = false;
  heartRateProvisional = false;
  provisionalHeartRateBpm = 0.0f;
  lastValidHeartRateMs = candidateTimestampMs;
  referenceIntervalMs = medianBeatInterval();
  lastReliableRhythmMs = candidateTimestampMs;
  lastReliableHeartRateBpm = heartRateBpm;

  rhythmSignalUsable = true;
  pulseSignalUsable = opticalSignalUsable;
  sample.beatDetected = true;
  sample.beatIntervalMs = lastBeatIntervalMs;
  sample.recoveredBeatMultiple = recoveredMultiple;
  sample.rejectCode = REJECT_NONE;
  lastRejectCode = REJECT_NONE;
  return true;
}

void PulseOximeter::resetAcquisitionTrackers() {
  positiveTracker = PeakTracker();
  negativeTracker = PeakTracker();
  filteredPrev2 = 0.0f;
  filteredPrev1 = 0.0f;
  filteredPrev2Ms = 0;
  filteredPrev1Ms = 0;
  filteredHistoryCount = 0;
  lastCandidateActivityMs = 0;
}

void PulseOximeter::resetRhythmLock(bool keepDisplayedValue,
                                    bool keepReference) {
  pulsePolarity = 0;
  resetAcquisitionTrackers();
  lastBeatMs = 0;
  rhythmRejectStreak = 0;
  beatIntervalCount = 0;
  beatIntervalIndex = 0;
  for (uint8_t i = 0; i < BEAT_INTERVAL_BUF_SIZE; i++) beatIntervalBuf[i] = 0;
  rhythmConfidence = 0.0f;
  rhythmSignalUsable = false;
  pulseSignalUsable = false;
  acquisitionStartedMs = lastProcessedSampleTimestampMs;
  previousAcceptedBeatWasRecovered = false;
  lastRecoveredBeatMultiple = 1;
  heartRateValid = false;
  heartRateProvisional = false;
  provisionalHeartRateBpm = 0.0f;
  lastProvisionalHeartRateMs = 0;

  if (!keepDisplayedValue) {
    heartRateBpm = 0.0f;
    heartRateHeld = false;
    lastValidHeartRateMs = 0;
  } else {
    if (lastReliableHeartRateBpm > 0.0f) heartRateBpm = lastReliableHeartRateBpm;
    heartRateHeld = (heartRateBpm > 0.0f);
  }

  if (!keepReference) {
    referenceIntervalMs = 0;
    lastReliableRhythmMs = 0;
    lastReliableHeartRateBpm = 0.0f;
  }
}

void PulseOximeter::markMotionArtifact(uint32_t nowMs) {
  enterMotionState(nowMs);
}

void PulseOximeter::updateHeartRateState(uint32_t nowMs) {
  if (lastReliableRhythmMs > 0 &&
      (nowMs - lastReliableRhythmMs) > rhythmReferenceMemoryMs) {
    referenceIntervalMs = 0;
    lastReliableRhythmMs = 0;
    lastReliableHeartRateBpm = 0.0f;
  }

  if (heartRateProvisional &&
      (nowMs - lastProvisionalHeartRateMs) > 1800UL) {
    heartRateProvisional = false;
    provisionalHeartRateBpm = 0.0f;
    if (lastReliableHeartRateBpm > 0.0f) heartRateBpm = lastReliableHeartRateBpm;
  }

  if (lastValidHeartRateMs == 0 || heartRateBpm <= 0.0f) {
    heartRateValid = false;
    heartRateHeld = false;
    return;
  }

  unsigned long elapsed = nowMs - lastValidHeartRateMs;
  bool lockStillValid = (pulsePolarity != 0) &&
                        (acquisitionState == STATE_TRACKING) &&
                        quietStable && !motionArtifact &&
                        (rhythmConfidence >= minimumRhythmConfidence) &&
                        (morphologyConfidence >= minimumMorphologyConfidence);

  if (elapsed <= hbTimeoutMs && lockStillValid) {
    heartRateValid = true;
    heartRateHeld = false;
  } else if (elapsed <= hbHoldTimeoutMs && lastReliableHeartRateBpm > 0.0f) {
    heartRateValid = false;
    heartRateHeld = !heartRateProvisional;
    if (!heartRateProvisional) heartRateBpm = lastReliableHeartRateBpm;
  } else {
    heartRateBpm = 0.0f;
    heartRateValid = false;
    heartRateHeld = false;
    heartRateProvisional = false;
    provisionalHeartRateBpm = 0.0f;
    lastValidHeartRateMs = 0;
    // La referencia interna se conserva hasta rhythmReferenceMemoryMs.
  }
}

void PulseOximeter::updateRhythmConfidence() {
  rhythmConfidence = computeIntervalConfidence(beatIntervalBuf,
                                               beatIntervalCount);
}

void PulseOximeter::updateMorphologyConfidence() {
  float corr = fabsf(redIrCorrelation);
  float correlationPart = clampFloat((corr - 0.20f) / 0.75f,
                                     0.0f, 1.0f) * 50.0f;
  float qualityPart = clampFloat(pulseQuality / 100.0f,
                                 0.0f, 1.0f) * 30.0f;
  float gainPart = clampFloat(1.0f - redIrGainDeviation / 0.30f,
                              0.0f, 1.0f) * 20.0f;
  morphologyConfidence = correlationPart + qualityPart + gainPart;
}

float PulseOximeter::computeIntervalConfidence(
    const unsigned long* values, uint8_t count) const {
  if (values == NULL || count < 2) return 0.0f;
  if (count > BEAT_INTERVAL_BUF_SIZE) count = BEAT_INTERVAL_BUF_SIZE;
  float mean = 0.0f;
  for (uint8_t i = 0; i < count; i++) mean += (float)values[i];
  mean /= count;
  float variance = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    float d = (float)values[i] - mean;
    variance += d * d;
  }
  variance /= count;
  float cv = (mean > 1.0f) ? sqrtf(variance) / mean : 1.0f;
  return 100.0f * clampFloat(1.0f - cv / rhythmIntervalTolerance,
                             0.0f, 1.0f);
}

bool PulseOximeter::referenceRhythmAvailable(uint32_t nowMs) const {
  return referenceIntervalMs > 0 && lastReliableRhythmMs > 0 &&
         (nowMs - lastReliableRhythmMs) <= rhythmReferenceMemoryMs;
}

void PulseOximeter::updateProvisionalHeartRate(const PeakTracker& tracker,
                                               uint32_t nowMs) {
  if (tracker.acquisitionCount < provisionalIntervalsRequired ||
      !referenceRhythmAvailable(nowMs) || !quietStable || motionArtifact) {
    return;
  }

  float confidence = computeIntervalConfidence(tracker.acquisitionIntervals,
                                               tracker.acquisitionCount);
  unsigned long interval = medianValues(tracker.acquisitionIntervals,
                                        tracker.acquisitionCount);
  if (interval == 0) return;
  float refError = fabsf((float)interval - (float)referenceIntervalMs) /
                   (float)referenceIntervalMs;
  if (confidence < provisionalConfidenceMinimum ||
      refError > rhythmIntervalTolerance ||
      morphologyConfidence < minimumMorphologyConfidence) {
    return;
  }

  provisionalHeartRateBpm = 60000.0f / (float)interval;
  heartRateBpm = provisionalHeartRateBpm;
  heartRateProvisional = true;
  heartRateValid = false;
  heartRateHeld = false;
  lastProvisionalHeartRateMs = nowMs;
}

void PulseOximeter::updateOpticalMotionGate(uint32_t nowMs) {
  if (!opticalMotionGateEnabled) {
    quietStable = true;
    motionArtifact = false;
    if (pulsePolarity != 0) acquisitionState = STATE_TRACKING;
    else if (acquisitionState != STATE_ACQUIRING) acquisitionState = STATE_ACQUIRING;
    return;
  }

  bool motionEvidence = redMotionRangePercent >= redMotionThresholdPercent ||
                        irMotionRangePercent >= irMotionThresholdPercent;
  bool quietEvidence = redMotionRangePercent <= redQuietThresholdPercent &&
                       irMotionRangePercent <= irQuietThresholdPercent;

  if (motionEvidence) {
    if (motionEvidenceCount < 3) motionEvidenceCount++;
  } else if (motionEvidenceCount > 0) {
    motionEvidenceCount--;
  }

  if (motionEvidenceCount >= 2) {
    if (acquisitionState != STATE_MOVING) enterMotionState(nowMs);
    quietCandidateSinceMs = 0;
    return;
  }

  if (acquisitionState == STATE_MOVING) {
    if (quietEvidence) enterSettlingState(nowMs);
    return;
  }

  if (acquisitionState == STATE_SETTLING) {
    if (!quietEvidence) {
      quietCandidateSinceMs = 0;
      quietStable = false;
      return;
    }
    if (quietCandidateSinceMs == 0) quietCandidateSinceMs = nowMs;
    if ((nowMs - quietCandidateSinceMs) >= quietSettleMs) {
      enterAcquiringState(nowMs);
    }
  }
}

void PulseOximeter::enterMotionState(uint32_t nowMs) {
  acquisitionState = STATE_MOVING;
  stateChangedMs = nowMs;
  quietStable = false;
  motionArtifact = true;
  motionArtifactUntilMs = nowMs + 500UL;
  quietCandidateSinceMs = 0;
  resetRhythmLock(true, true);
}

void PulseOximeter::enterSettlingState(uint32_t nowMs) {
  acquisitionState = STATE_SETTLING;
  stateChangedMs = nowMs;
  quietStable = false;
  motionArtifact = true;
  quietCandidateSinceMs = nowMs;
  resetRhythmLock(true, true);
}

void PulseOximeter::enterAcquiringState(uint32_t nowMs) {
  acquisitionState = STATE_ACQUIRING;
  stateChangedMs = nowMs;
  quietStable = true;
  motionArtifact = false;
  motionEvidenceCount = 0;
  quietCandidateSinceMs = 0;
  resetRhythmLock(true, true);
  acquisitionStartedMs = nowMs;
}

void PulseOximeter::enterTrackingState(uint32_t nowMs) {
  acquisitionState = STATE_TRACKING;
  stateChangedMs = nowMs;
  quietStable = true;
  motionArtifact = false;
  motionEvidenceCount = 0;
}

void PulseOximeter::acceptBeatInterval(unsigned long intervalMs,
                                       uint32_t beatTimestampMs) {
  (void)beatTimestampMs;
  beatIntervalBuf[beatIntervalIndex] = intervalMs;
  beatIntervalIndex = (beatIntervalIndex + 1) % BEAT_INTERVAL_BUF_SIZE;
  if (beatIntervalCount < BEAT_INTERVAL_BUF_SIZE) beatIntervalCount++;

  unsigned long intervalUsed = medianBeatInterval();
  if (intervalUsed == 0) intervalUsed = intervalMs;
  float bpmFiltered = 60000.0f / (float)intervalUsed;

  float smoothingBase = heartRateBpm;
  if (!heartRateValid && lastReliableHeartRateBpm > 0.0f) {
    smoothingBase = lastReliableHeartRateBpm;
  }
  if (smoothingBase <= 0.0f) {
    heartRateBpm = bpmFiltered;
    if (hrFastBeatsRemaining > 0) hrFastBeatsRemaining--;
    return;
  }

  float alpha = hbAlphaSlow;
  if (hrFastBeatsRemaining > 0) {
    alpha = hbAlphaFast;
    hrFastBeatsRemaining--;
  }
  float diff = fabsf(bpmFiltered - smoothingBase);
  if (diff >= hbChangeThresholdBpm) alpha = hbAlphaFast;
  heartRateBpm = (1.0f - alpha) * smoothingBase + alpha * bpmFiltered;
}

unsigned long PulseOximeter::medianValues(const unsigned long* values,
                                          uint8_t count) {
  if (values == NULL || count == 0) return 0;
  if (count > BEAT_INTERVAL_BUF_SIZE) count = BEAT_INTERVAL_BUF_SIZE;

  unsigned long sorted[BEAT_INTERVAL_BUF_SIZE];
  for (uint8_t i = 0; i < count; i++) sorted[i] = values[i];

  for (uint8_t i = 1; i < count; i++) {
    unsigned long key = sorted[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  return sorted[count / 2];
}

unsigned long PulseOximeter::medianBeatInterval() const {
  return medianValues(beatIntervalBuf, beatIntervalCount);
}

bool PulseOximeter::computeSpO2FromBuffer() {
  if (!rhythmSignalUsable || !heartRateValid || vitalsCount < 64) return false;

  uint16_t window = vitalsCount;
  if (window > VITALS_BUF_SIZE) window = VITALS_BUF_SIZE;

  double sumRedRaw = 0.0;
  double sumIrRaw = 0.0;
  double sumRedAc2 = 0.0;
  double sumIrAc2 = 0.0;

  for (uint16_t i = 0; i < window; i++) {
    uint16_t idx = (vitalsHead + VITALS_BUF_SIZE - window + i) % VITALS_BUF_SIZE;
    sumRedRaw += redBuf[idx];
    sumIrRaw += irBuf[idx];
    sumRedAc2 += (double)redFilteredBuf[idx] * redFilteredBuf[idx];
    sumIrAc2 += (double)irFilteredBuf[idx] * irFilteredBuf[idx];
  }

  float dcRed = (float)(sumRedRaw / window);
  float dcIr = (float)(sumIrRaw / window);
  float acRed = sqrtf((float)(sumRedAc2 / window));
  float acIr = sqrtf((float)(sumIrAc2 / window));

  if (dcRed < 1.0f || dcIr < 1.0f || acRed < 1.0f || acIr < 1.0f) {
    return false;
  }

  // Los filtros ya normalizan cada canal por su DC. La relación AC normalizada
  // es equivalente al ratio-of-ratios utilizado por el cálculo clásico.
  rValue = acRed / acIr;

  float spo2 = spo2A * rValue * rValue + spo2B * rValue + spo2C;
  if (spo2 < 70.0f) spo2 = 70.0f;
  if (spo2 > 100.0f) spo2 = 100.0f;

  if (spo2Percent <= 0.0f) {
    spo2Percent = spo2;
  } else {
    spo2Percent = 0.8f * spo2Percent + 0.2f * spo2;
  }

  return true;
}

void PulseOximeter::setHeartRateSmoothing(float alphaFast, float alphaSlow,
                                           float changeThresholdBpm) {
  if (alphaFast > 0.0f && alphaFast <= 1.0f) hbAlphaFast = alphaFast;
  if (alphaSlow > 0.0f && alphaSlow <= 1.0f) hbAlphaSlow = alphaSlow;
  if (changeThresholdBpm > 0.0f) hbChangeThresholdBpm = changeThresholdBpm;
}

void PulseOximeter::setHeartRateTimeout(unsigned long timeoutMs) {
  if (timeoutMs >= 1000) hbTimeoutMs = timeoutMs;
}

void PulseOximeter::setHeartRatePeakDetection(float peakFactor, float peakMin) {
  if (peakFactor > 0.05f && peakFactor < 1.0f) hbPeakFactor = peakFactor;
  if (peakMin > 0.0f) hbPeakMin = peakMin;
}

void PulseOximeter::setHeartRateHoldTimeout(unsigned long timeoutMs) {
  if (timeoutMs >= 1000) hbHoldTimeoutMs = timeoutMs;
}

void PulseOximeter::setHeartRateJumpLimit(float bpm) {
  if (bpm > 1.0f) hbMaxInstantJumpBpm = bpm;
}