#ifndef PULSE_OXIMETER_H
#define PULSE_OXIMETER_H

// MAX30102 WristFilter Rebased OpticalGate v1.1.3

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
        adcRange(0x01),        // 4096 nA
        sampleRate(0x02),      // 200 sps antes del promedio FIFO
        pulseWidth(0x03),      // 411 us, 18-bit
        redLedPA(0x0F),
        irLedPA(0x0F) {}
  };

  enum BeatRejectCode : uint8_t {
    REJECT_NONE = 0,
    REJECT_OPTICAL = 1,
    REJECT_REFRACTORY = 2,
    REJECT_INTERVAL_RANGE = 3,
    REJECT_INCONSISTENT = 4,
    REJECT_MOTION = 5,
    REJECT_REACQUIRE = 6,
    REJECT_SECONDARY_PEAK = 7,
    REJECT_LOW_CONFIDENCE = 8,
    REJECT_SETTLING = 9
  };

  enum AcquisitionState : uint8_t {
    STATE_MOVING = 0,
    STATE_SETTLING = 1,
    STATE_ACQUIRING = 2,
    STATE_TRACKING = 3
  };

  struct Sample {
    uint32_t red;
    uint32_t ir;
    uint32_t timestampMs;

    // Señal AC normalizada en ppm respecto de la DC.
    float redFiltered;
    float irFiltered;
    float adaptiveThreshold;
    float pulseQuality;

    bool candidateDetected;
    bool beatDetected;
    uint16_t beatIntervalMs;
    int8_t pulsePolarity;
    bool opticalSignalUsable;
    bool rhythmSignalUsable;
    bool motionArtifact;
    bool quietStable;
    bool heartRateProvisional;
    uint8_t acquisitionState;
    uint8_t recoveredBeatMultiple;
    float temporalConfidence;
    float morphologyConfidence;
    float perfusionIndexPercent;
    float redIrCorrelation;
    float redIrGain;
    float redIrGainDeviation;
    float redMotionRangePercent;
    float irMotionRangePercent;
    float heartRateBpm;
    float provisionalHeartRateBpm;
    bool heartRateValid;
    bool heartRateHeld;
    uint16_t referenceBeatIntervalMs;
    uint8_t rejectCode;

    Sample()
      : red(0), ir(0), timestampMs(0),
        redFiltered(0.0f), irFiltered(0.0f),
        adaptiveThreshold(0.0f), pulseQuality(0.0f),
        candidateDetected(false), beatDetected(false),
        beatIntervalMs(0), pulsePolarity(0),
        opticalSignalUsable(false), rhythmSignalUsable(false),
        motionArtifact(false), quietStable(false),
        heartRateProvisional(false), acquisitionState(STATE_ACQUIRING),
        recoveredBeatMultiple(1), temporalConfidence(0.0f),
        morphologyConfidence(0.0f), perfusionIndexPercent(0.0f),
        redIrCorrelation(0.0f), redIrGain(0.0f), redIrGainDeviation(0.0f),
        redMotionRangePercent(0.0f), irMotionRangePercent(0.0f),
        heartRateBpm(0.0f), provisionalHeartRateBpm(0.0f),
        heartRateValid(false), heartRateHeld(false),
        referenceBeatIntervalMs(0), rejectCode(REJECT_NONE) {}
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

  float getRawSampleRateHz() const;
  float getEffectiveSampleRateHz() const;
  uint32_t getSamplePeriodUs() const;

  // Autoexposición
  bool setAdcRange(uint8_t adcRange);
  uint8_t getAdcRange() const { return currentConfig.adcRange; }

  void enableAutoExposure(bool enable) { autoExposureEnabled = enable; }
  bool isAutoExposureEnabled() const { return autoExposureEnabled; }

  bool processSamples(Sample* samples, size_t count);

  void setExposureTargets(uint32_t lowCounts, uint32_t highCounts);
  void setFingerThreshold(uint32_t thresholdCounts);
  bool autoAdjustExposure(const Sample* samples, size_t count);

  uint32_t getExposureLowTarget() const { return exposureLowTarget; }
  uint32_t getExposureHighTarget() const { return exposureHighTarget; }
  uint32_t getFingerThreshold() const { return fingerThreshold; }

  // Optimización de potencia LED
  void setLedBackoffMargin(uint32_t counts);
  void setMinLedPulseAmplitude(uint8_t paCode);
  uint32_t getLedBackoffMargin() const { return ledBackoffMargin; }
  uint8_t getMinLedPulseAmplitude() const { return minLedPA; }

  // Calidad base de señal/exposición
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

  // Filtrado PPG para dedo/muñeca
  void setPulseFilter(float highPassHz, float lowPassHz, float dcTrackHz = 0.20f);
  void setPulseQualityThreshold(float minimumQuality);
  void setMinimumPulseRms(float minimumRmsPpm);
  void setBeatRefractory(unsigned long refractoryMs);
  void setBeatHysteresis(float rearmFactor);
  void setRhythmAcquisition(uint8_t requiredIntervals, float tolerancePercent);
  void setMinimumRhythmConfidence(float confidence);
  void setMinimumMorphologyConfidence(float confidence);
  void setProvisionalRhythm(float confidence, uint8_t requiredIntervals,
                            unsigned long referenceMemoryMs);
  void setMissedBeatRecovery(uint8_t maximumMultiple, float tolerancePercent);
  void setFastReacquireThreshold(float shortWindowSeconds, float spanMultiplier);
  void enableOpticalMotionGate(bool enable);
  void setOpticalMotionGate(float redMotionPercent, float irMotionPercent,
                            float redQuietPercent, float irQuietPercent,
                            unsigned long settleMs,
                            float windowSeconds = 1.0f);
  void setHeartRateRange(float minimumBpm, float maximumBpm);

  float getFilteredRed() const { return lastFilteredRed; }
  float getFilteredIR() const { return lastFilteredIR; }
  float getAdaptiveThreshold() const { return adaptiveThreshold; }
  float getPulseQuality() const { return pulseQuality; }
  float getPerfusionIndex() const { return perfusionIndexPercent; }
  float getRedIrCorrelation() const { return redIrCorrelation; }
  float getPulseRms() const { return pulseRmsPpm; }
  bool isOpticalSignalUsable() const { return opticalSignalUsable; }
  bool isRhythmSignalUsable() const { return rhythmSignalUsable; }
  bool isPulseSignalUsable() const { return pulseSignalUsable; }
  bool hasMotionArtifact() const { return motionArtifact; }
  int8_t getPulsePolarity() const { return pulsePolarity; }
  uint16_t getLastBeatIntervalMs() const { return lastBeatIntervalMs; }
  uint8_t getLastRejectCode() const { return lastRejectCode; }
  float getRhythmConfidence() const { return rhythmConfidence; }
  float getTemporalConfidence() const { return rhythmConfidence; }
  float getMorphologyConfidence() const { return morphologyConfidence; }
  float getMinimumRhythmConfidence() const { return minimumRhythmConfidence; }
  float getMinimumMorphologyConfidence() const { return minimumMorphologyConfidence; }
  bool isQuietStable() const { return quietStable; }
  bool isOpticalMotionGateEnabled() const { return opticalMotionGateEnabled; }
  AcquisitionState getAcquisitionState() const { return acquisitionState; }
  float getRedMotionRangePercent() const { return redMotionRangePercent; }
  float getIrMotionRangePercent() const { return irMotionRangePercent; }
  bool isHeartRateProvisional() const { return heartRateProvisional; }
  float getProvisionalHeartRateBpm() const { return provisionalHeartRateBpm; }
  uint16_t getReferenceBeatIntervalMs() const { return (uint16_t)referenceIntervalMs; }
  uint8_t getLastRecoveredBeatMultiple() const { return lastRecoveredBeatMultiple; }
  float getRedIrGain() const { return redIrGain; }
  float getRedIrGainDeviation() const { return redIrGainDeviation; }
  bool wasBeatDetected() const { return beatDetectedInLastProcess; }

  // Pulsómetro/oxímetro
  float getHeartRateBpm() const { return heartRateBpm; }
  float getSpO2() const { return spo2Percent; }
  float getRValue() const { return rValue; }

  void setSpO2Coefficients(float a, float b, float c);
  void enableVitals(bool enable) { vitalsEnabled = enable; }
  bool isVitalsEnabled() const { return vitalsEnabled; }

  void updateVitals(Sample* samples, size_t count);

  bool isHeartRateValid() const { return heartRateValid; }
  bool isHeartRateHeld() const { return heartRateHeld; }

  void setHeartRatePeakDetection(float peakFactor, float peakMin);
  void setHeartRateHoldTimeout(unsigned long timeoutMs);

  void setHeartRateSmoothing(float alphaFast, float alphaSlow, float changeThresholdBpm);
  void setHeartRateTimeout(unsigned long timeoutMs);
  void setHeartRateJumpLimit(float bpm);

private:
  struct ChannelFilterState {
    float median1;
    float median2;
    bool medianReady;

    float dc;
    bool dcReady;

    float hpPrevX;
    float hpPrevY;
    float lp1;
    float lp2;

    ChannelFilterState()
      : median1(0.0f), median2(0.0f), medianReady(false),
        dc(0.0f), dcReady(false), hpPrevX(0.0f), hpPrevY(0.0f),
        lp1(0.0f), lp2(0.0f) {}
  };

  struct PeakTracker {
    bool armed;
    uint32_t lastCandidateMs;
    unsigned long acquisitionIntervals[4];
    uint8_t acquisitionCount;

    PeakTracker()
      : armed(true), lastCandidateMs(0),
        acquisitionIntervals{0, 0, 0, 0}, acquisitionCount(0) {}
  };

  TwoWire* wire = NULL;
  Config currentConfig;

  bool autoExposureEnabled = true;
  uint32_t exposureLowTarget = 70000;
  uint32_t exposureHighTarget = 180000;
  uint32_t fingerThreshold = 5000;

  unsigned long lastExposureChangeMs = 0;
  unsigned long exposureSettleMs = 50;

  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t& value);
  bool readFIFORaw(uint32_t& red, uint32_t& ir);

  uint32_t ledBackoffMargin = 20000;
  uint8_t minLedPA = 0x04;
  unsigned long exposureOptimizeMs = 1000;

  // Reconstrucción temporal FIFO
  bool sampleClockValid = false;
  uint32_t lastSampleTimestampUs = 0;

  // Calidad base de señal
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
  uint32_t usableStableSamples = 50;   // ~1 s a 50 Hz efectivos
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

  // Filtro PPG
  ChannelFilterState redFilter;
  ChannelFilterState irFilter;

  float filterHighPassHz = 0.50f;
  float filterLowPassHz = 4.00f;
  float filterDCTrackHz = 0.20f;

  float hpAlpha = 0.0f;
  float lpAlpha = 0.0f;
  float dcAlpha = 0.0f;

  float lastFilteredRed = 0.0f;
  float lastFilteredIR = 0.0f;

  float filteredPrev2 = 0.0f;
  float filteredPrev1 = 0.0f;
  uint32_t filteredPrev2Ms = 0;
  uint32_t filteredPrev1Ms = 0;
  uint8_t filteredHistoryCount = 0;

  float adaptiveThreshold = 0.0f;
  float pulseQuality = 0.0f;
  float pulseQualityMinimum = 20.0f;
  float pulseRmsPpm = 0.0f;
  float minimumPulseRmsPpm = 35.0f;
  float perfusionIndexPercent = 0.0f;
  float redIrCorrelation = 0.0f;
  float redIrGain = 0.0f;
  float redIrGainBaseline = 0.0f;
  float redIrGainDeviation = 0.0f;
  bool redIrGainReady = false;

  bool opticalSignalUsable = false;
  bool rhythmSignalUsable = false;
  bool pulseSignalUsable = false;
  bool motionArtifact = false;
  uint32_t motionArtifactUntilMs = 0;
  uint32_t acquisitionStartedMs = 0;
  uint32_t lastCandidateActivityMs = 0;

  bool beatDetectedInLastProcess = false;
  int8_t pulsePolarity = 0;
  float rhythmConfidence = 0.0f;
  float morphologyConfidence = 0.0f;
  float minimumRhythmConfidence = 65.0f;
  float minimumMorphologyConfidence = 45.0f;
  float provisionalConfidenceMinimum = 75.0f;
  uint8_t provisionalIntervalsRequired = 2;
  unsigned long rhythmReferenceMemoryMs = 15000UL;
  unsigned long referenceIntervalMs = 0;
  unsigned long lastReliableRhythmMs = 0;
  float lastReliableHeartRateBpm = 0.0f;
  float provisionalHeartRateBpm = 0.0f;
  bool heartRateProvisional = false;
  unsigned long lastProvisionalHeartRateMs = 0;
  uint8_t lastRecoveredBeatMultiple = 1;
  bool previousAcceptedBeatWasRecovered = false;
  uint8_t maximumMissedBeatMultiple = 2;
  float missedBeatRecoveryTolerance = 0.18f;
  float fastThresholdWindowSeconds = 0.80f;
  float fastThresholdSpanMultiplier = 1.35f;
  uint8_t rhythmRejectStreak = 0;
  uint8_t lastRejectCode = REJECT_NONE;
  uint16_t lastBeatIntervalMs = 0;

  PeakTracker positiveTracker;
  PeakTracker negativeTracker;
  float beatRearmFactor = 0.45f;
  uint8_t acquisitionIntervalsRequired = 3;
  float rhythmIntervalTolerance = 0.25f;
  float minimumHeartRateBpm = 45.0f;
  float maximumHeartRateBpm = 200.0f;

  // Compuerta óptica de movimiento, ajustada para uso en muñeca.
  bool opticalMotionGateEnabled = false;
  AcquisitionState acquisitionState = STATE_ACQUIRING;
  bool quietStable = true;
  float redMotionRangePercent = 0.0f;
  float irMotionRangePercent = 0.0f;
  float redMotionThresholdPercent = 0.95f;
  float irMotionThresholdPercent = 1.15f;
  float redQuietThresholdPercent = 0.65f;
  float irQuietThresholdPercent = 0.80f;
  float motionWindowSeconds = 1.0f;
  unsigned long quietSettleMs = 650UL;
  unsigned long quietCandidateSinceMs = 0;
  unsigned long stateChangedMs = 0;
  uint8_t motionEvidenceCount = 0;
  uint32_t lastProcessedSampleTimestampMs = 0;

  // Pulsómetro/Oxímetro
  bool vitalsEnabled = true;

  float heartRateBpm = 0.0f;
  float spo2Percent = 0.0f;
  float rValue = 0.0f;

  float spo2A = 0.0f;
  float spo2B = -25.0f;
  float spo2C = 110.0f;

  static const uint16_t VITALS_BUF_SIZE = 128;
  uint32_t redBuf[VITALS_BUF_SIZE];
  uint32_t irBuf[VITALS_BUF_SIZE];
  float redFilteredBuf[VITALS_BUF_SIZE];
  float irFilteredBuf[VITALS_BUF_SIZE];
  uint16_t vitalsHead = 0;
  uint16_t vitalsCount = 0;

  unsigned long lastBeatMs = 0;
  unsigned long lastValidHeartRateMs = 0;
  uint8_t rejectedBeatCount = 0;

  bool prevSignalUsable = false;
  uint8_t hrFastBeatsRemaining = 0;

  float hbAlphaFast = 0.40f;
  float hbAlphaSlow = 0.18f;
  float hbChangeThresholdBpm = 8.0f;

  unsigned long hbTimeoutMs = 2500;

  bool heartRateValid = false;
  bool heartRateHeld = false;
  unsigned long hbHoldTimeoutMs = 5000;

  float hbMaxInstantJumpBpm = 20.0f;
  unsigned long beatRefractoryMs = 320;

  static const uint8_t BEAT_INTERVAL_BUF_SIZE = 5;
  unsigned long beatIntervalBuf[BEAT_INTERVAL_BUF_SIZE] = {0, 0, 0, 0, 0};
  uint8_t beatIntervalCount = 0;
  uint8_t beatIntervalIndex = 0;

  float hbPeakFactor = 0.20f;
  float hbPeakMin = 35.0f;

  void refreshFilterCoefficients();
  void resetVitalsProcessing(bool clearDisplayedValues);
  float filterChannel(ChannelFilterState& state, uint32_t raw);
  static float median3(float a, float b, float c);
  static float clampFloat(float value, float low, float high);

  void processVitalSample(Sample& sample);
  void updatePulseMetrics();
  bool detectBeat(float currentFiltered, uint32_t currentTimestampMs, Sample& sample);
  bool updatePeakTracker(PeakTracker& tracker, int8_t polarity,
                         float x0, float x1, float x2,
                         float highThreshold, float lowThreshold);
  bool processAcquisitionCandidate(PeakTracker& tracker, int8_t polarity,
                                   uint32_t candidateTimestampMs, Sample& sample);
  bool processLockedCandidate(uint32_t candidateTimestampMs, Sample& sample);
  void lockRhythm(int8_t polarity, const PeakTracker& tracker,
                  uint32_t beatTimestampMs, Sample& sample);
  void resetRhythmLock(bool keepDisplayedValue, bool keepReference = true);
  void resetAcquisitionTrackers();
  void updateHeartRateState(uint32_t nowMs);
  void markMotionArtifact(uint32_t nowMs);
  void updateRhythmConfidence();
  void updateMorphologyConfidence();
  float computeIntervalConfidence(const unsigned long* values, uint8_t count) const;
  bool referenceRhythmAvailable(uint32_t nowMs) const;
  void updateProvisionalHeartRate(const PeakTracker& tracker, uint32_t nowMs);
  void updateOpticalMotionGate(uint32_t nowMs);
  void enterMotionState(uint32_t nowMs);
  void enterSettlingState(uint32_t nowMs);
  void enterAcquiringState(uint32_t nowMs);
  void enterTrackingState(uint32_t nowMs);
  void acceptBeatInterval(unsigned long intervalMs, uint32_t beatTimestampMs);
  unsigned long medianBeatInterval() const;
  static unsigned long medianValues(const unsigned long* values, uint8_t count);
  bool computeSpO2FromBuffer();
};

#endif