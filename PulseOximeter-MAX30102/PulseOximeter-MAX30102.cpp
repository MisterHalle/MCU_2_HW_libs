#include "PulseOximeter-MAX30102.h"

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

  // Limpia flags pendientes
  (void)readStatus1();
  (void)readStatus2();

  // Deshabilita interrupciones en esta v1 básica
  if (!writeRegister(MAX30102_REG_INTR_ENABLE_1, 0x00)) return false;
  if (!writeRegister(MAX30102_REG_INTR_ENABLE_2, 0x00)) return false;

  if (!applyConfig(cfg)) {
    return false;
  }

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

  return clearFIFO();
}

bool PulseOximeter::clearFIFO() {
  if (!writeRegister(MAX30102_REG_FIFO_WR_PTR, 0x00)) return false;
  if (!writeRegister(MAX30102_REG_OVF_COUNTER, 0x00)) return false;
  if (!writeRegister(MAX30102_REG_FIFO_RD_PTR, 0x00)) return false;
  return true;
}

uint8_t PulseOximeter::readPartID() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_PART_ID, value)) {
    return 0;
  }
  return value;
}

uint8_t PulseOximeter::readRevisionID() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_REV_ID, value)) {
    return 0;
  }
  return value;
}

uint8_t PulseOximeter::readStatus1() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_INTR_STATUS_1, value)) {
    return 0;
  }
  return value;
}

uint8_t PulseOximeter::readStatus2() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_INTR_STATUS_2, value)) {
    return 0;
  }
  return value;
}

uint8_t PulseOximeter::readOverflowCounter() {
  uint8_t value = 0;
  if (!readRegister(MAX30102_REG_OVF_COUNTER, value)) {
    return 0;
  }
  return value & 0x1F;
}

uint8_t PulseOximeter::availableSamples() {
  uint8_t wr = 0;
  uint8_t rd = 0;
  uint8_t ovf = 0;

  if (!readRegister(MAX30102_REG_FIFO_WR_PTR, wr)) {
    return 0;
  }
  if (!readRegister(MAX30102_REG_FIFO_RD_PTR, rd)) {
    return 0;
  }
  if (!readRegister(MAX30102_REG_OVF_COUNTER, ovf)) {
    return 0;
  }

  wr &= 0x1F;
  rd &= 0x1F;
  ovf &= 0x1F;

  if (wr == rd) {
    return (ovf > 0) ? 32 : 0;
  }

  return (wr - rd) & 0x1F;
}

bool PulseOximeter::readSample(Sample& out) {
  if (availableSamples() == 0) {
    return false;
  }

  wire->beginTransmission(MAX30102_I2C_ADDR);
  wire->write(MAX30102_REG_FIFO_DATA);
  if (wire->endTransmission(false) != 0) {
    return false;
  }

  if (wire->requestFrom((uint8_t)MAX30102_I2C_ADDR, (uint8_t)6) != 6) {
    return false;
  }

  uint32_t red = ((uint32_t)wire->read() << 16);
  red |= ((uint32_t)wire->read() << 8);
  red |= (uint32_t)wire->read();

  uint32_t ir = ((uint32_t)wire->read() << 16);
  ir |= ((uint32_t)wire->read() << 8);
  ir |= (uint32_t)wire->read();

  out.red = red & 0x03FFFF;
  out.ir = ir & 0x03FFFF;
  out.timestampMs = millis();

  return true;
}

size_t PulseOximeter::readAvailableSamples(Sample* buffer, size_t maxSamples) {
  if (buffer == NULL || maxSamples == 0) {
    return 0;
  }

  size_t count = 0;
  uint8_t n = availableSamples();
  if (n > maxSamples) {
    n = (uint8_t)maxSamples;
  }

  while (count < n) {
    if (!readSample(buffer[count])) {
      break;
    }
    count++;
  }

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

  if (!writeRegister(MAX30102_REG_SPO2_CONFIG, value)) {
    return false;
  }

  currentConfig.adcRange = adcRange;
  currentConfig.sampleRate = sampleRate;
  currentConfig.pulseWidth = pulseWidth;
  return true;
}

bool PulseOximeter::setFIFOConfig(uint8_t fifoAverage, bool rollover, uint8_t almostFull) {
  fifoAverage &= 0x07;
  almostFull &= 0x0F;

  uint8_t value = (uint8_t)(fifoAverage << 5);
  if (rollover) {
    value |= 0x10;
  }
  value |= almostFull;

  if (!writeRegister(MAX30102_REG_FIFO_CONFIG, value)) {
    return false;
  }

  currentConfig.fifoAverage = fifoAverage;
  currentConfig.fifoRollover = rollover;
  currentConfig.fifoAlmostFull = almostFull;
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

bool PulseOximeter::writeRegister(uint8_t reg, uint8_t value) {
  if (wire == NULL) {
    return false;
  }

  wire->beginTransmission(MAX30102_I2C_ADDR);
  wire->write(reg);
  wire->write(value);
  return (wire->endTransmission() == 0);
}

bool PulseOximeter::readRegister(uint8_t reg, uint8_t& value) {
  if (wire == NULL) {
    return false;
  }

  wire->beginTransmission(MAX30102_I2C_ADDR);
  wire->write(reg);
  if (wire->endTransmission(false) != 0) {
    return false;
  }

  if (wire->requestFrom((uint8_t)MAX30102_I2C_ADDR, (uint8_t)1) != 1) {
    return false;
  }

  value = wire->read();
  return true;
}

//----------------------  Funciones de autoexposure
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
  if (!autoExposureEnabled || samples == NULL || count == 0) {
    return false;
  }

  unsigned long now = millis();

  if (now < adjustLockUntilMs) {
    return false;
  }

  if (freezeAutoExposureWhenUsable && signalUsable) {
    return false;
  }

  //unsigned long now = millis();
  if ((now - lastExposureChangeMs) < exposureSettleMs) {
    return false;
  }

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
  uint32_t meanIr  = (uint32_t)(sumIr / count);

  // No ajustar si no parece haber dedo
  if (meanIr < fingerThreshold) {
    return false;
  }

  uint32_t meanLevel = (meanRed > meanIr) ? meanRed : meanIr;
  uint32_t peakLevel = (maxRed > maxIr) ? maxRed : maxIr;

  bool tooHigh = (peakLevel > 250000UL) || (meanLevel > exposureHighTarget);
  bool tooLow  = (meanLevel < exposureLowTarget);
  bool inBand  = !tooHigh && !tooLow;

  if (inBand) {
    badExposureWindows = 0;
    goodExposureWindows++;
  } else {
    goodExposureWindows = 0;
    if (badExposureWindows < 255) {
      badExposureWindows++;
    }
  }

  // No cambies nada hasta ver 3 ventanas malas seguidas
  if (!tooHigh && !tooLow) {
    return false;
  }

  if (badExposureWindows < 3) {
    return false;
  }

  bool changed = false;

  // 1) Protección rápida contra saturación
  if (peakLevel > 250000 || meanLevel > exposureHighTarget) {
    if (currentConfig.adcRange < 0x03) {
      changed = setAdcRange(currentConfig.adcRange + 1);
    } else {
      uint8_t newRed = (currentConfig.redLedPA > 0x02) ? (currentConfig.redLedPA - 0x02) : 0x00;
      uint8_t newIr  = (currentConfig.irLedPA  > 0x02) ? (currentConfig.irLedPA  - 0x02) : 0x00;
      changed = setLedPulseAmplitude(newRed, newIr);
    }
  }
  // 2) Señal demasiado baja
  else if (meanLevel < exposureLowTarget) {
    if (currentConfig.adcRange > 0x00) {
      changed = setAdcRange(currentConfig.adcRange - 1);
    } else {
      uint8_t newRed = (currentConfig.redLedPA < 0x3F) ? (currentConfig.redLedPA + 0x02) : 0x3F;
      uint8_t newIr  = (currentConfig.irLedPA  < 0x3F) ? (currentConfig.irLedPA  + 0x02) : 0x3F;
      changed = setLedPulseAmplitude(newRed, newIr);
    }
  }
  // 3) Señal dentro de banda: optimizar corriente lentamente
  else {
    bool enoughMargin = meanLevel > (exposureLowTarget + ledBackoffMargin);
    bool canLowerLed = (currentConfig.redLedPA > minLedPA) && (currentConfig.irLedPA > minLedPA);
    bool slowOptimizeReady = (now - lastExposureChangeMs) >= exposureOptimizeMs;

    if (enoughMargin && canLowerLed && slowOptimizeReady) {
      uint8_t newRed = currentConfig.redLedPA - 0x01;
      uint8_t newIr  = currentConfig.irLedPA  - 0x01;
      changed = setLedPulseAmplitude(newRed, newIr);
    }
  }

  if (changed) {
    clearFIFO();
    lastExposureChangeMs = now;
    adjustLockUntilMs = now + 400;   // prueba 400 ms, luego ajustas
    stableSampleCount = 0;
    exposureStable = false;
    prevDCHistoryValid = false;
    dcSettled = false;
    badExposureWindows = 0;
    goodExposureWindows = 0;
  }

  return changed;
}

bool PulseOximeter::processSamples(const Sample* samples, size_t count) {
  if (samples == NULL || count == 0) {
    return false;
  }

  bool adjusted = false;

  if (autoExposureEnabled) {
    adjusted = autoAdjustExposure(samples, count);
  }

  updateSignalIndicators(samples, count);

  if (vitalsEnabled) {
    updateVitals(samples, count);
  }

  return adjusted;
}

//-------------- LED power optimization

void PulseOximeter::setLedBackoffMargin(uint32_t counts) {
  ledBackoffMargin = counts;
}

void PulseOximeter::setMinLedPulseAmplitude(uint8_t paCode) {
  minLedPA = paCode & 0x3F;
}

//-------------- Signal Queality functions
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
  if (samples == NULL || count == 0) {
    return;
  }

  uint64_t sumRed = 0;
  uint64_t sumIr = 0;
  uint32_t maxRed = 0;
  uint32_t maxIr = 0;

  for (size_t i = 0; i < count; i++) {
    sumRed += samples[i].red;
    sumIr += samples[i].ir;

    if (samples[i].red > maxRed) maxRed = samples[i].red;
    if (samples[i].ir  > maxIr)  maxIr  = samples[i].ir;
  }

  redDC = (uint32_t)(sumRed / count);
  irDC  = (uint32_t)(sumIr / count);

  redPeak = maxRed;
  irPeak = maxIr;

  // 1) Finger present con histéresis sobre IR DC
  if (!fingerPresent) {
    if (irDC >= fingerOnThreshold) {
      fingerPresent = true;
    }
  } else {
    if (irDC <= fingerOffThreshold) {
      fingerPresent = false;
    }
  }

  // 2) Flags base de invalidez
  uint8_t st1 = readStatus1();
  alcOverflow = ((st1 & 0x20) != 0);   // ALC_OVF

  saturated = (redPeak >= saturationThreshold) || (irPeak >= saturationThreshold);
  fifoOverflow = (readOverflowCounter() != 0);

  bool baseValid = fingerPresent && !alcOverflow && !saturated && !fifoOverflow;

  // 3) La exposición debe estar dentro de la banda objetivo
  uint32_t meanLevel = (redDC > irDC) ? redDC : irDC;
  exposureInRange = (meanLevel >= exposureLowTarget) && (meanLevel <= exposureHighTarget);

  // 4) La DC debe estar asentada
  if (!prevDCHistoryValid) {
    dcSettled = false;
    prevDCHistoryValid = true;
  } else {
    uint32_t redDiff = (redDC > prevRedDC) ? (redDC - prevRedDC) : (prevRedDC - redDC);
    uint32_t irDiff  = (irDC  > prevIrDC)  ? (irDC  - prevIrDC)  : (prevIrDC  - irDC);

    dcSettled = (redDiff <= redDCSettleTol) && (irDiff <= irDCSettleTol);
  }

  prevRedDC = redDC;
  prevIrDC = irDC;

  // 5) Exposición quieta: sin cambios recientes del autoajuste
  unsigned long now = millis();
  bool exposureQuiet = (!autoExposureEnabled) || ((now - lastExposureChangeMs) >= exposureSettleMs);

  // 6) Configuración quieta
  bool configChanged =
      (currentConfig.adcRange != lastStableAdcRange) ||
      (currentConfig.redLedPA != lastStableRedPA) ||
      (currentConfig.irLedPA  != lastStableIrPA);

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

  // 7) usable final
  signalUsable = stableConditions && exposureStable;
}

void PulseOximeter::setUsableStableSamples(uint32_t samples) {
  if (samples > 0) {
    usableStableSamples = samples;
  }
}

void PulseOximeter::setDCSettleTolerance(uint32_t redTol, uint32_t irTol) {
  redDCSettleTol = redTol;
  irDCSettleTol = irTol;
}

//--------------- Pulsometer-Oximeter functions
void PulseOximeter::setSpO2Coefficients(float a, float b, float c) {
  spo2A = a;
  spo2B = b;
  spo2C = c;
}

void PulseOximeter::updateVitals(const Sample* samples, size_t count) {
  if (!signalUsable) {
    heartRateBpm = 0.0f;
    heartRateValid = false;
    spo2Percent = 0.0f;
    rValue = 0.0f;
    vitalsCount = 0;
    vitalsHead = 0;
    lastBeatMs = 0;
    hrFastBeatsRemaining = 0;
    prevSignalUsable = false;
    return;
  }

  if (!prevSignalUsable && signalUsable) {
    lastBeatMs = 0;
    hrFastBeatsRemaining = 3;
  }

  prevSignalUsable = true;

  pushVitalSamples(samples, count);

  computeHeartRateFromBuffer();
  computeSpO2FromBuffer();
}

void PulseOximeter::pushVitalSamples(const Sample* samples, size_t count) {
  for (size_t i = 0; i < count; i++) {
    redBuf[vitalsHead] = samples[i].red;
    irBuf[vitalsHead] = samples[i].ir;

    vitalsHead = (vitalsHead + 1) % VITALS_BUF_SIZE;

    if (vitalsCount < VITALS_BUF_SIZE) {
      vitalsCount++;
    }
  }
}

bool PulseOximeter::computeHeartRateFromBuffer() {
  if (vitalsCount < 32) {
    return false;
  }

  unsigned long now = millis();

  // Si pasa mucho tiempo sin latido válido, no borrar BPM.
  // Solo marcar que el siguiente debe re-adquirirse.
  if (lastBeatMs != 0 && (now - lastBeatMs) > hbHoldTimeoutMs) {
    lastBeatMs = 0;
    hrFastBeatsRemaining = 2;
    heartRateValid = false;
    beatIntervalCount = 0;
    beatIntervalIndex = 0;
    return false;
  }

  uint64_t sumIr = 0;
  uint32_t minIr = 0xFFFFFFFF;
  uint32_t maxIr = 0;

  for (uint16_t i = 0; i < vitalsCount; i++) {
    uint16_t idx = (vitalsHead + VITALS_BUF_SIZE - vitalsCount + i) % VITALS_BUF_SIZE;
    uint32_t v = irBuf[idx];
    sumIr += v;
    if (v < minIr) minIr = v;
    if (v > maxIr) maxIr = v;
  }

  float dcIr = (float)sumIr / vitalsCount;
  float acThresh = (float)(maxIr - minIr) * hbPeakFactor;
  if (acThresh < hbPeakMin) acThresh = hbPeakMin;

  uint16_t idx2 = (vitalsHead + VITALS_BUF_SIZE - 1) % VITALS_BUF_SIZE;
  uint16_t idx1 = (vitalsHead + VITALS_BUF_SIZE - 2) % VITALS_BUF_SIZE;
  uint16_t idx0 = (vitalsHead + VITALS_BUF_SIZE - 3) % VITALS_BUF_SIZE;

  float x0 = (float)irBuf[idx0] - dcIr;
  float x1 = (float)irBuf[idx1] - dcIr;
  float x2 = (float)irBuf[idx2] - dcIr;

  bool isPeak = (x1 > x0) && (x1 > x2) && (x1 > acThresh);

  if (!isPeak) {
    return false;
  }

  if (lastBeatMs == 0) {
    lastBeatMs = now;
    return true;
  }

  unsigned long dt = now - lastBeatMs;

  // rango fisiológico amplio: ~35 a 220 bpm
  if (dt < 273 || dt > 1715) {
    return false;
  }

  float bpmInstant = 60000.0f / (float)dt;

  // Rechazo de cambio instantáneo demasiado brusco si ya tenemos HB válido
  if (heartRateValid && heartRateBpm > 0.0f) {
    float diff = fabsf(bpmInstant - heartRateBpm);
    if (diff > hbMaxInstantJumpBpm) {
      // ignorar este latido espurio
      return false;
    }
  }

  lastBeatMs = now;

  // Guardar intervalo y usar mediana de 3 para robustez frente a un falso pico único
  beatIntervalBuf[beatIntervalIndex] = dt;
  beatIntervalIndex = (beatIntervalIndex + 1) % 3;
  if (beatIntervalCount < 3) {
    beatIntervalCount++;
  }

  unsigned long dtUsed = dt;

  if (beatIntervalCount == 3) {
    unsigned long a = beatIntervalBuf[0];
    unsigned long b = beatIntervalBuf[1];
    unsigned long c = beatIntervalBuf[2];

    // mediana de 3
    if (a > b) { unsigned long t = a; a = b; b = t; }
    if (b > c) { unsigned long t = b; b = c; c = t; }
    if (a > b) { unsigned long t = a; a = b; b = t; }

    dtUsed = b;
  }

  float bpmFiltered = 60000.0f / (float)dtUsed;

  if (heartRateBpm <= 0.0f) {
    heartRateBpm = bpmFiltered;
    heartRateValid = true;
    if (hrFastBeatsRemaining > 0) {
      hrFastBeatsRemaining--;
    }
    return true;
  }

  float alpha = hbAlphaSlow;

  if (hrFastBeatsRemaining > 0) {
    alpha = hbAlphaFast;
    hrFastBeatsRemaining--;
  }

  float diff = fabsf(bpmFiltered - heartRateBpm);
  if (diff >= hbChangeThresholdBpm) {
    alpha = hbAlphaFast;
  }

  heartRateBpm = (1.0f - alpha) * heartRateBpm + alpha * bpmFiltered;
  heartRateValid = true;
  return true;
}

bool PulseOximeter::computeSpO2FromBuffer() {
  if (vitalsCount < 64) {
    return false;
  }

  uint64_t sumRed = 0;
  uint64_t sumIr = 0;
  uint32_t minRed = 0xFFFFFFFF;
  uint32_t minIr  = 0xFFFFFFFF;
  uint32_t maxRed = 0;
  uint32_t maxIr  = 0;

  for (uint16_t i = 0; i < vitalsCount; i++) {
    uint16_t idx = (vitalsHead + VITALS_BUF_SIZE - vitalsCount + i) % VITALS_BUF_SIZE;

    uint32_t red = redBuf[idx];
    uint32_t ir  = irBuf[idx];

    sumRed += red;
    sumIr += ir;

    if (red < minRed) minRed = red;
    if (red > maxRed) maxRed = red;
    if (ir < minIr) minIr = ir;
    if (ir > maxIr) maxIr = ir;
  }

  float dcRed = (float)sumRed / vitalsCount;
  float dcIr  = (float)sumIr / vitalsCount;

  if (dcRed < 1.0f || dcIr < 1.0f) {
    return false;
  }

  float acRed = (float)(maxRed - minRed);
  float acIr  = (float)(maxIr - minIr);

  if (acRed < 1.0f || acIr < 1.0f) {
    return false;
  }

  rValue = (acRed / dcRed) / (acIr / dcIr);

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

void PulseOximeter::setHeartRateSmoothing(float alphaFast, float alphaSlow, float changeThresholdBpm) {
  if (alphaFast > 0.0f && alphaFast <= 1.0f) hbAlphaFast = alphaFast;
  if (alphaSlow > 0.0f && alphaSlow <= 1.0f) hbAlphaSlow = alphaSlow;
  if (changeThresholdBpm > 0.0f) hbChangeThresholdBpm = changeThresholdBpm;
}

void PulseOximeter::setHeartRateTimeout(unsigned long timeoutMs) {
  hbTimeoutMs = timeoutMs;
}

void PulseOximeter::setHeartRatePeakDetection(float peakFactor, float peakMin) {
  if (peakFactor > 0.05f && peakFactor < 1.0f) {
    hbPeakFactor = peakFactor;
  }
  if (peakMin > 0.0f) {
    hbPeakMin = peakMin;
  }
}

void PulseOximeter::setHeartRateHoldTimeout(unsigned long timeoutMs) {
  hbHoldTimeoutMs = timeoutMs;
}

void PulseOximeter::setHeartRateJumpLimit(float bpm) {
  if (bpm > 1.0f) {
    hbMaxInstantJumpBpm = bpm;
  }
}