#ifndef DW1000_RANGE_H
#define DW1000_RANGE_H

#include <Arduino.h>
#include "dw1000_driver.h"

enum Dw1000Role : uint8_t {
  DW1000_ROLE_TAG = 1,
  DW1000_ROLE_ANCHOR = 2
};

struct Dw1000RangeConfig {
  Dw1000Role role = DW1000_ROLE_TAG;

  Dw1000Pins pins;
  Dw1000Config radio;

  uint32_t rangePeriodMs = 300;
  uint32_t timeoutMs = 150;

  uint32_t respDelayUus = 5000;
  uint32_t finalDelayUus = 5000;
  uint32_t resultDelayUus = 3000;
};

struct Dw1000Reading {
  bool valid = false;

  float rawCm = 0.0f;
  float correctedCm = 0.0f;
  float rawM = 0.0f;
  float rxPowerDbm = NAN;

  uint8_t sequence = 0;
  uint32_t timestampMs = 0;

  Dw1000RxQuality quality;
};

struct Dw1000RangeDebug {
  const char* status = "unknown";

  bool irqLevel = false;
  uint32_t lastSysStatus = 0;
  uint64_t sysState = 0;

  uint32_t txDoneCount = 0;
  uint32_t rxDoneCount = 0;
  uint32_t rxErrorCount = 0;
  uint32_t txErrorCount = 0;
  uint32_t timeoutCount = 0;

  uint32_t pollTxCount = 0;
  uint32_t pollRxCount = 0;
  uint32_t respTxCount = 0;
  uint32_t respRxCount = 0;
  uint32_t finalTxCount = 0;
  uint32_t finalRxCount = 0;
  uint32_t resultTxCount = 0;
  uint32_t resultRxCount = 0;
};

class Dw1000Range {
public:
  bool begin(const Dw1000RangeConfig& config, SPIClass& spi = SPI);
  void update();

  bool available() const;
  bool read(Dw1000Reading& out);

  float getDistanceCm() const;
  float getDistanceM() const;
  float getRawDistanceCm() const;
  float getRxPowerDbm() const;

  void setCalibration(float offsetCm, float scale = 1.0f);

  void setAntennaDelay(uint16_t delay);
  uint16_t getAntennaDelay() const;
  void getLastRxQuality(Dw1000RxQuality& out);

  bool isFresh(uint32_t maxAgeMs = 500) const;

  void recover();
  const char* getStatus() const;

  uint32_t getDeviceId();
  uint32_t getLastDeviceId() const;

  uint8_t getSpiMode() const;
  uint32_t getSpiHz() const;

  void getDebugInfo(Dw1000RangeDebug& out);

private:
  enum TagState : uint8_t {
    TAG_IDLE,
    TAG_WAIT_POLL_TX,
    TAG_WAIT_RESP,
    TAG_WAIT_FINAL_TX,
    TAG_WAIT_RESULT
  };

  enum AnchorState : uint8_t {
    ANCHOR_WAIT_POLL,
    ANCHOR_WAIT_RESP_TX,
    ANCHOR_WAIT_FINAL,
    ANCHOR_WAIT_RESULT_TX
  };

  Dw1000RangeConfig _cfg;
  Dw1000Driver _dw;

  bool _started = false;
  bool _newReading = false;

  Dw1000Reading _last;

  float _offsetCm = 0.0f;
  float _scale = 1.0f;

  const char* _status = "not_started";

  uint8_t _seq = 0;
  uint8_t _currentSeq = 0;

  uint32_t _stateMs = 0;
  uint32_t _lastRangeMs = 0;

  TagState _tagState = TAG_IDLE;
  AnchorState _anchorState = ANCHOR_WAIT_POLL;

  uint64_t _pollTxTs = 0;
  uint64_t _respRxTs = 0;

  uint64_t _savedPollRxTs = 0;
  uint64_t _savedRespTxTs = 0;
  uint8_t _savedSeq = 0;

  uint32_t _lastSysStatus = 0;

  uint32_t _txDoneCount = 0;
  uint32_t _rxDoneCount = 0;
  uint32_t _rxErrorCount = 0;
  uint32_t _txErrorCount = 0;
  uint32_t _timeoutCount = 0;

  uint32_t _pollTxCount = 0;
  uint32_t _pollRxCount = 0;
  uint32_t _respTxCount = 0;
  uint32_t _respRxCount = 0;
  uint32_t _finalTxCount = 0;
  uint32_t _finalRxCount = 0;
  uint32_t _resultTxCount = 0;
  uint32_t _resultRxCount = 0;

  Dw1000RxQuality _lastRxQuality;

  void updateTag(const Dw1000Events& ev);
  void updateAnchor(const Dw1000Events& ev);

  void sendPoll();

  void processTagRx(uint64_t rxTs, const uint8_t* frame, uint16_t len);
  void processAnchorRx(uint64_t rxTs, const uint8_t* frame, uint16_t len);

  void sendResult(uint8_t seq, double distanceM, uint64_t finalRxTs);
  void storeDistance(uint8_t seq, float distanceM);

  static uint32_t uusToDw1000Ticks(uint32_t uus);
  static uint64_t diff40(uint64_t later, uint64_t earlier);

  static void makeHeader(uint8_t* frame, uint8_t type, uint8_t seq);
  static bool validHeader(const uint8_t* frame, uint16_t len);

  static void writeTs40(uint8_t* dst, uint64_t value);
  static uint64_t readTs40(const uint8_t* src);

  static void writeI32(uint8_t* dst, int32_t value);
  static int32_t readI32(const uint8_t* src);

  static double calculateDsTwrDistanceM(uint64_t pollTx,
                                        uint64_t pollRx,
                                        uint64_t respTx,
                                        uint64_t respRx,
                                        uint64_t finalTx,
                                        uint64_t finalRx);
};

#endif