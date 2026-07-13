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
        sampleRate(0x02),      // 200 sps
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

  bool processSamples(const Sample* samples, size_t count);

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
  
  //Signal Quality Measuring
  void updateSignalIndicators(const Sample* samples, size_t count);

  bool isFingerPresent() const { return fingerPresent; }
  bool isSignalUsable() const { return signalUsable; }
  bool getALCOverflow() const { return alcOverflow; }
  bool isSaturated() const { return saturated; }
  bool hasFIFOOverflow() const { return fifoOverflow; }

  uint32_t getRedDC() const { return redDC; }
  uint32_t getIrDC() const { return irDC; }
  uint32_t getRedPeak() const { return redPeak; }
  uint32_t getIrPeak() const { return irPeak; }

  void setFingerThresholds(uint32_t onCounts, uint32_t offCounts);
  void setSaturationThreshold(uint32_t counts);

  bool isExposureStable() const { return exposureStable; }
  uint32_t getStableSampleCount() const { return stableSampleCount; }

  void setUsableStableSamples(uint32_t samples);
  uint32_t getUsableStableSamples() const { return usableStableSamples; }

  bool isExposureInRange() const { return exposureInRange; }
  bool isDCSettled() const { return dcSettled; }

  void setDCSettleTolerance(uint32_t redTol, uint32_t irTol);

  //Pulsometer/Oximeter
  float getHeartRateBpm() const { return heartRateBpm; }
  float getSpO2() const { return spo2Percent; }
  float getRValue() const { return rValue; }

  void setSpO2Coefficients(float a, float b, float c);
  void enableVitals(bool enable) { vitalsEnabled = enable; }
  bool isVitalsEnabled() const { return vitalsEnabled; }

  void updateVitals(const Sample* samples, size_t count);

  bool isHeartRateValid() const { return heartRateValid; }

  void setHeartRatePeakDetection(float peakFactor, float peakMin);
  void setHeartRateHoldTimeout(unsigned long timeoutMs);

  void setHeartRateSmoothing(float alphaFast, float alphaSlow, float changeThresholdBpm);
  void setHeartRateTimeout(unsigned long timeoutMs);
  void setHeartRateJumpLimit(float bpm);

private:
  TwoWire* wire;
  Config currentConfig;

  bool autoExposureEnabled = true;
  uint32_t exposureLowTarget = 70000;
  uint32_t exposureHighTarget = 180000;
  uint32_t fingerThreshold = 5000;

  unsigned long lastExposureChangeMs = 0;
  unsigned long exposureSettleMs = 50;

  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t& value);

  uint32_t ledBackoffMargin = 20000;   // margen sobre exposureLowTarget
  uint8_t minLedPA = 0x04;             // no bajar de aquí
  unsigned long exposureOptimizeMs = 1000; // optimización lenta

  //Signal Quality
  bool fingerPresent = false;
  bool signalUsable = false;
  bool alcOverflow = false;
  bool saturated = false;
  bool fifoOverflow = false;

  uint32_t redDC = 0;
  uint32_t irDC = 0;
  uint32_t redPeak = 0;
  uint32_t irPeak = 0;

  uint32_t fingerOnThreshold = 10000;
  uint32_t fingerOffThreshold = 5000;
  uint32_t saturationThreshold = 250000;

  bool exposureStable = false;
  uint32_t stableSampleCount = 0;
  uint32_t usableStableSamples = 24;   // ~1s con 25 sps efectivos
  unsigned long adjustLockUntilMs = 0;
  uint8_t badExposureWindows = 0;
  uint8_t goodExposureWindows = 0;
  bool freezeAutoExposureWhenUsable = true;

  uint8_t lastStableAdcRange = 0xFF;
  uint8_t lastStableRedPA = 0xFF;
  uint8_t lastStableIrPA = 0xFF;

  bool exposureInRange = false;
  bool dcSettled = false;

  uint32_t prevRedDC = 0;
  uint32_t prevIrDC = 0;
  bool prevDCHistoryValid = false;

  uint32_t redDCSettleTol = 3000;
  uint32_t irDCSettleTol = 3000;

  //PulsometerOximeter
  bool vitalsEnabled = true;

  float heartRateBpm = 0.0f;
  float spo2Percent = 0.0f;
  float rValue = 0.0f;

  float spo2A = 0.0f;
  float spo2B = -25.0f;
  float spo2C = 110.0f;   // solo provisional para pruebas

  static const uint16_t VITALS_BUF_SIZE = 128;
  uint32_t redBuf[VITALS_BUF_SIZE];
  uint32_t irBuf[VITALS_BUF_SIZE];
  uint16_t vitalsHead = 0;
  uint16_t vitalsCount = 0;

  float irPrev1 = 0.0f;
  float irPrev2 = 0.0f;
  unsigned long lastBeatMs = 0;

  bool prevSignalUsable = false;
  uint8_t hrFastBeatsRemaining = 0;

  float hbAlphaFast = 0.40f;
  float hbAlphaSlow = 0.18f;
  float hbChangeThresholdBpm = 8.0f;

  unsigned long hbTimeoutMs = 2500;

  bool heartRateValid = false;
  unsigned long hbHoldTimeoutMs = 4000;

  float hbMaxInstantJumpBpm = 20.0f;

  unsigned long beatIntervalBuf[3] = {0, 0, 0};
  uint8_t beatIntervalCount = 0;
  uint8_t beatIntervalIndex = 0;

  float hbPeakFactor = 0.25f;   // antes 0.35f
  float hbPeakMin = 25.0f;      // antes 50.0f

  void pushVitalSamples(const Sample* samples, size_t count);
  bool computeHeartRateFromBuffer();
  bool computeSpO2FromBuffer();

};

#endif