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
  }

  return changed;
}

//-------------- LED power optimization

void PulseOximeter::setLedBackoffMargin(uint32_t counts) {
  ledBackoffMargin = counts;
}

void PulseOximeter::setMinLedPulseAmplitude(uint8_t paCode) {
  minLedPA = paCode & 0x3F;
}
