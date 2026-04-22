#ifndef PULSE_OXIMETER_H
#define PULSE_OXIMETER_H

#include <Arduino.h>
#include <Wire.h>

// MAX30102 registers
#define MAX30102_REG_INTR_STATUS_1   0x00
#define MAX30102_REG_INTR_STATUS_2   0x01
#define MAX30102_REG_INTR_ENABLE_1   0x02
#define MAX30102_REG_INTR_ENABLE_2   0x03
#define MAX30102_REG_FIFO_WR_PTR     0x04
#define MAX30102_REG_OVF_COUNTER     0x05
#define MAX30102_REG_FIFO_RD_PTR     0x06
#define MAX30102_REG_FIFO_DATA       0x07
#define MAX30102_REG_FIFO_CONFIG     0x08
#define MAX30102_REG_MODE_CONFIG     0x09
#define MAX30102_REG_SPO2_CONFIG     0x0A
#define MAX30102_REG_LED1_PA         0x0C
#define MAX30102_REG_LED2_PA         0x0D
#define MAX30102_REG_TEMP_INT        0x1F
#define MAX30102_REG_TEMP_FRAC       0x20
#define MAX30102_REG_TEMP_CONFIG     0x21
#define MAX30102_REG_REV_ID          0xFE
#define MAX30102_REG_PART_ID         0xFF

#define MAX30102_I2C_ADDR            0x57
#define MAX30102_EXPECTED_PART_ID    0x15

class PulseOximeter {
public:
  struct Config {
    uint8_t fifoAverage;
    bool fifoRollover;
    uint8_t fifoAlmostFull;

    uint8_t mode;
    uint8_t adcRange;
    uint8_t sampleRate;
    uint8_t pulseWidth;

    uint8_t redLedPA;
    uint8_t irLedPA;

    Config()
      : fifoAverage(0x02),     // 4 muestras promedio
        fifoRollover(false),
        fifoAlmostFull(0x0F),
        mode(0x03),            // SpO2 mode = Red + IR
        adcRange(0x01),        // 4096nA
        sampleRate(0x01),      // 100 sps
        pulseWidth(0x03),      // 411us, 18-bit
        redLedPA(0x0F),
        irLedPA(0x0F) {}
  };

  struct Sample {
    uint32_t red;
    uint32_t ir;
    uint32_t timestampMs;

    Sample() : red(0), ir(0), timestampMs(0) {}
  };

  bool begin(TwoWire& wirePort = Wire);
  bool begin(const Config& cfg, TwoWire& wirePort = Wire);

  bool reset();
  bool applyConfig(const Config& cfg);
  bool clearFIFO();

  uint8_t readPartID();
  uint8_t readRevisionID();
  uint8_t readStatus1();
  uint8_t readStatus2();
  uint8_t readOverflowCounter();
  uint8_t availableSamples();

  bool readSample(Sample& out);
  size_t readAvailableSamples(Sample* buffer, size_t maxSamples);

  bool setLedPulseAmplitude(uint8_t redPA, uint8_t irPA);
  bool setSpO2Config(uint8_t adcRange, uint8_t sampleRate, uint8_t pulseWidth);
  bool setFIFOConfig(uint8_t fifoAverage, bool rollover, uint8_t almostFull);
  bool setSampleAveraging(uint8_t fifoAverage);
  bool setFIFORollover(bool enable);
  bool setFIFOAlmostFull(uint8_t almostFull);

  const Config& getConfig() const { return currentConfig; }

  uint8_t getSampleAveraging() const { return currentConfig.fifoAverage; }
  bool getFIFORollover() const { return currentConfig.fifoRollover; }
  uint8_t getFIFOAlmostFull() const { return currentConfig.fifoAlmostFull; }

  //Exposure adjust
  bool setAdcRange(uint8_t adcRange);
  uint8_t getAdcRange() const { return currentConfig.adcRange; }

  void enableAutoExposure(bool enable) { autoExposureEnabled = enable; }
  bool isAutoExposureEnabled() const { return autoExposureEnabled; }

  void setExposureTargets(uint32_t lowCounts, uint32_t highCounts);
  void setFingerThreshold(uint32_t thresholdCounts);
  bool autoAdjustExposure(const Sample* samples, size_t count);

  uint32_t getExposureLowTarget() const { return exposureLowTarget; }
  uint32_t getExposureHighTarget() const { return exposureHighTarget; }
  uint32_t getFingerThreshold() const { return fingerThreshold; }

  //Led power optimization
  void setLedBackoffMargin(uint32_t counts);
  void setMinLedPulseAmplitude(uint8_t paCode);
  uint32_t getLedBackoffMargin() const { return ledBackoffMargin; }
  uint8_t getMinLedPulseAmplitude() const { return minLedPA; }

private:
  TwoWire* wire;
  Config currentConfig;

  bool autoExposureEnabled = true;
  uint32_t exposureLowTarget = 70000;
  uint32_t exposureHighTarget = 180000;
  uint32_t fingerThreshold = 5000;

  unsigned long lastExposureChangeMs = 0;
  unsigned long exposureSettleMs = 250;

  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t& value);

  uint32_t ledBackoffMargin = 20000;   // margen sobre exposureLowTarget
  uint8_t minLedPA = 0x04;             // no bajar de aquí
  unsigned long exposureOptimizeMs = 1000; // optimización lenta
};

#endif