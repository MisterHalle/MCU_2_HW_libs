#include "dw1000_range.h"

static constexpr uint64_t DW1000_TIME40_MASK_LOCAL = 0xFFFFFFFFFFULL;

// 1 UWB microsecond = 512 / 499.2 MHz.
// En ticks DW1000 se usa aprox. 63898.
static constexpr uint32_t UUS_TO_DW1000_TIME = 63898UL;

// 1 tick DW1000 equivale aprox. a 15.65 ps.
// Distancia = tiempo_vuelo * velocidad_luz.
// Valor aproximado en metros por tick.
static constexpr double DISTANCE_PER_UWB_TICK_M = 0.0046917639786159;

static constexpr uint8_t MAGIC0 = 0x55;
static constexpr uint8_t MAGIC1 = 0xAA;

static constexpr uint8_t MSG_POLL   = 0x01;
static constexpr uint8_t MSG_RESP   = 0x02;
static constexpr uint8_t MSG_FINAL  = 0x03;
static constexpr uint8_t MSG_RESULT = 0x04;

// =====================================================
// Public API
// =====================================================
bool Dw1000Range::begin(const Dw1000RangeConfig& config, SPIClass& spi) {
  _cfg = config;

  _newReading = false;
  _last = Dw1000Reading();

  _lastSysStatus = 0;

  _txDoneCount = 0;
  _rxDoneCount = 0;
  _rxErrorCount = 0;
  _txErrorCount = 0;
  _timeoutCount = 0;

  _pollTxCount = 0;
  _pollRxCount = 0;
  _respTxCount = 0;
  _respRxCount = 0;
  _finalTxCount = 0;
  _finalRxCount = 0;
  _resultTxCount = 0;
  _resultRxCount = 0;

  _status = "begin";

  if (!_dw.begin(_cfg.pins, _cfg.radio, spi)) {
    _started = false;
    _status = "dw1000_begin_failed";
    return false;
  }

  _started = true;
  _lastRangeMs = millis() - _cfg.rangePeriodMs;
  _stateMs = millis();

  if (_cfg.role == DW1000_ROLE_TAG) {
    _tagState = TAG_IDLE;
    _status = "tag_ready";
  } else {
    _anchorState = ANCHOR_WAIT_POLL;
    _dw.startReceive();
    _status = "anchor_listening";
  }

  return true;
}

void Dw1000Range::update() {
  if (!_started) {
    return;
  }

  // En v0.2.2 leemos SYS_STATUS siempre.
  // Esto evita depender 100% del nivel instantaneo del pin IRQ.
  Dw1000Events ev = _dw.handleIrq();

  if (ev.rawStatus != 0) {
    _lastSysStatus = ev.rawStatus;
  }

  if (ev.txDone) {
    _txDoneCount++;
  }

  if (ev.rxDone) {
    _rxDoneCount++;
  }

  if (ev.rxError) {
    _rxErrorCount++;
    _status = "rx_error_recover";
    _dw.startReceive();

    if (_cfg.role == DW1000_ROLE_ANCHOR) {
      _anchorState = ANCHOR_WAIT_POLL;
    }

    if (_cfg.role == DW1000_ROLE_TAG && _tagState != TAG_IDLE) {
      _tagState = TAG_IDLE;
    }
  }

  if (ev.txError) {
    _txErrorCount++;
    _status = "tx_error_recover";
    _dw.startReceive();

    if (_cfg.role == DW1000_ROLE_ANCHOR) {
      _anchorState = ANCHOR_WAIT_POLL;
    }

    if (_cfg.role == DW1000_ROLE_TAG) {
      _tagState = TAG_IDLE;
    }
  }

  if (_cfg.role == DW1000_ROLE_TAG) {
    updateTag(ev);
  } else {
    updateAnchor(ev);
  }
}

bool Dw1000Range::available() const {
  return _newReading;
}

bool Dw1000Range::read(Dw1000Reading& out) {
  if (!_newReading) {
    return false;
  }

  out = _last;
  _newReading = false;

  return true;
}

float Dw1000Range::getDistanceCm() const {
  return _last.correctedCm;
}

float Dw1000Range::getDistanceM() const {
  return _last.correctedCm / 100.0f;
}

float Dw1000Range::getRawDistanceCm() const {
  return _last.rawCm;
}

float Dw1000Range::getRxPowerDbm() const {
  return _last.rxPowerDbm;
}

void Dw1000Range::setCalibration(float offsetCm, float scale) {
  _offsetCm = offsetCm;
  _scale = scale;
}

void Dw1000Range::setAntennaDelay(uint16_t delay) {
  _cfg.radio.antennaDelay = delay;
  _dw.setAntennaDelay(delay);
}

uint16_t Dw1000Range::getAntennaDelay() const {
  return _dw.getAntennaDelay();
}

bool Dw1000Range::isFresh(uint32_t maxAgeMs) const {
  if (!_last.valid) {
    return false;
  }

  return millis() - _last.timestampMs <= maxAgeMs;
}

void Dw1000Range::recover() {
  _dw.softResetRadio();
  _newReading = false;

  if (_cfg.role == DW1000_ROLE_TAG) {
    _tagState = TAG_IDLE;
    _status = "tag_recovered";
  } else {
    _anchorState = ANCHOR_WAIT_POLL;
    _dw.startReceive();
    _status = "anchor_recovered";
  }
}

const char* Dw1000Range::getStatus() const {
  return _status;
}

uint32_t Dw1000Range::getDeviceId() {
  return _dw.getDeviceId();
}

uint32_t Dw1000Range::getLastDeviceId() const {
  return _dw.getLastDeviceId();
}

uint8_t Dw1000Range::getSpiMode() const {
  return _dw.getSpiMode();
}

uint32_t Dw1000Range::getSpiHz() const {
  return _dw.getSpiHz();
}

// =====================================================
// TAG state machine
// =====================================================
void Dw1000Range::updateTag(const Dw1000Events& ev) {
  if (_tagState == TAG_IDLE && millis() - _lastRangeMs >= _cfg.rangePeriodMs) {
    _lastRangeMs = millis();
    sendPoll();
    return;
  }

  if (ev.txDone) {
    if (_tagState == TAG_WAIT_POLL_TX) {
      _pollTxTs = _dw.getTransmitTimestamp();
      _dw.startReceive();

      _tagState = TAG_WAIT_RESP;
      _stateMs = millis();
      _status = "tag_wait_resp";

      return;
    }

    if (_tagState == TAG_WAIT_FINAL_TX) {
      _dw.startReceive();

      _tagState = TAG_WAIT_RESULT;
      _stateMs = millis();
      _status = "tag_wait_result";

      return;
    }
  }

  if (ev.rxDone) {
    uint8_t rxBuffer[128];
    uint16_t rxLen = 0;
    uint64_t rxTs = _dw.getReceiveTimestamp();

    if (_dw.readReceivedData(rxBuffer, rxLen, sizeof(rxBuffer))) {
      processTagRx(rxTs, rxBuffer, rxLen);
    }
  }

  if ((_tagState == TAG_WAIT_RESP ||
      _tagState == TAG_WAIT_RESULT ||
      _tagState == TAG_WAIT_FINAL_TX) &&
      millis() - _stateMs > _cfg.timeoutMs) {
    _timeoutCount++;

    _dw.forceTrxOff();
    _dw.clearAllStatus();
    _dw.startReceive();

    _tagState = TAG_IDLE;
    _status = "tag_timeout";
  }
}

// =====================================================
// ANCHOR state machine
// =====================================================
void Dw1000Range::updateAnchor(const Dw1000Events& ev) {
  if (ev.txDone) {
    if (_anchorState == ANCHOR_WAIT_RESP_TX) {
      _dw.startReceive();

      _anchorState = ANCHOR_WAIT_FINAL;
      _stateMs = millis();
      _status = "anchor_wait_final";

      return;
    }

    if (_anchorState == ANCHOR_WAIT_RESULT_TX) {
      _dw.startReceive();

      _anchorState = ANCHOR_WAIT_POLL;
      _status = "anchor_listening";

      return;
    }
  }

  if (ev.rxDone) {
    uint8_t rxBuffer[128];
    uint16_t rxLen = 0;
    uint64_t rxTs = _dw.getReceiveTimestamp();

    if (_dw.readReceivedData(rxBuffer, rxLen, sizeof(rxBuffer))) {
      processAnchorRx(rxTs, rxBuffer, rxLen);
    }
  }

  if (_anchorState == ANCHOR_WAIT_FINAL && millis() - _stateMs > _cfg.timeoutMs) {
    _timeoutCount++;

    _dw.forceTrxOff();
    _dw.clearAllStatus();
    _dw.startReceive();

    _anchorState = ANCHOR_WAIT_POLL;
    _status = "anchor_final_timeout";
  }
}

// =====================================================
// Frame flow
// =====================================================
void Dw1000Range::sendPoll() {
  uint8_t frame[12] = {0};

  _currentSeq = ++_seq;

  makeHeader(frame, MSG_POLL, _currentSeq);

  // Padding identificable para debug.
  frame[4] = 'P';
  frame[5] = 'O';
  frame[6] = 'L';
  frame[7] = 'L';

  if (_dw.transmit(frame, sizeof(frame))) {
    _pollTxCount++;
    _tagState = TAG_WAIT_POLL_TX;
    _stateMs = millis();
    _status = "tag_poll_sent";
  } else {
    _tagState = TAG_IDLE;
    _status = "tag_poll_failed";
  }
}

void Dw1000Range::processTagRx(uint64_t rxTs, const uint8_t* frame, uint16_t len) {
  _dw.readRxQuality(_lastRxQuality);

  if (!validHeader(frame, len)) {
    return;
  }

  uint8_t type = frame[2];
  uint8_t seq = frame[3];

  if (_tagState == TAG_WAIT_RESP &&
      type == MSG_RESP &&
      seq == _currentSeq &&
      len >= 14) {
    _respRxCount++;

    _respRxTs = rxTs;

    uint64_t finalTxTs = _dw.calculateDelayedTransmitTimestamp(
      _respRxTs,
      uusToDw1000Ticks(_cfg.finalDelayUus)
    );

    uint8_t finalFrame[24] = {0};

    makeHeader(finalFrame, MSG_FINAL, _currentSeq);

    writeTs40(finalFrame + 4, _pollTxTs);
    writeTs40(finalFrame + 9, _respRxTs);
    writeTs40(finalFrame + 14, finalTxTs);

    finalFrame[19] = 'F';
    finalFrame[20] = 'I';
    finalFrame[21] = 'N';

    if (_dw.transmitDelayedAt(finalFrame, sizeof(finalFrame), finalTxTs)) {
      _finalTxCount++;

      _tagState = TAG_WAIT_FINAL_TX;
      _stateMs = millis();
      _status = "tag_final_scheduled";
    } else {
      _tagState = TAG_IDLE;
      _dw.startReceive();
      _status = "tag_final_failed";
    }

    return;
  }

  if (_tagState == TAG_WAIT_RESULT &&
      type == MSG_RESULT &&
      seq == _currentSeq &&
      len >= 8) {
    _resultRxCount++;
    int32_t distanceMm = readI32(frame + 4);

    storeDistance(seq, (float)distanceMm / 1000.0f);

    _tagState = TAG_IDLE;
    _status = "tag_result_ok";

    return;
  }
}

void Dw1000Range::processAnchorRx(uint64_t rxTs, const uint8_t* frame, uint16_t len) {
  _dw.readRxQuality(_lastRxQuality);

  if (!validHeader(frame, len)) {
    return;
  }

  uint8_t type = frame[2];
  uint8_t seq = frame[3];

  if (_anchorState == ANCHOR_WAIT_POLL &&
      type == MSG_POLL &&
      len >= 4) {
    _pollRxCount++;
    _savedSeq = seq;
    _savedPollRxTs = rxTs;

    _savedRespTxTs = _dw.calculateDelayedTransmitTimestamp(
      _savedPollRxTs,
      uusToDw1000Ticks(_cfg.respDelayUus)
    );

    uint8_t respFrame[16] = {0};

    makeHeader(respFrame, MSG_RESP, _savedSeq);

    writeTs40(respFrame + 4, _savedPollRxTs);
    writeTs40(respFrame + 9, _savedRespTxTs);

    respFrame[14] = 'R';
    respFrame[15] = 'S';

    if (_dw.transmitDelayedAt(respFrame, sizeof(respFrame), _savedRespTxTs)) {
      _respTxCount++;
      _anchorState = ANCHOR_WAIT_RESP_TX;
      _stateMs = millis();
      _status = "anchor_resp_scheduled";
    } else {
      _anchorState = ANCHOR_WAIT_POLL;
      _dw.startReceive();
      _status = "anchor_resp_failed";
    }

    return;
  }

  if (_anchorState == ANCHOR_WAIT_FINAL &&
      type == MSG_FINAL &&
      seq == _savedSeq &&
      len >= 19) {
    _finalRxCount++;
    uint64_t finalRxTs = rxTs;

    uint64_t initiatorPollTxTs = readTs40(frame + 4);
    uint64_t initiatorRespRxTs = readTs40(frame + 9);
    uint64_t initiatorFinalTxTs = readTs40(frame + 14);

    double distanceM = calculateDsTwrDistanceM(
      initiatorPollTxTs,
      _savedPollRxTs,
      _savedRespTxTs,
      initiatorRespRxTs,
      initiatorFinalTxTs,
      finalRxTs
    );

    storeDistance(seq, (float)distanceM);
    sendResult(seq, distanceM, finalRxTs);

    return;
  }
}

void Dw1000Range::sendResult(uint8_t seq, double distanceM, uint64_t finalRxTs) {
  int32_t distanceMm = (int32_t)(distanceM * 1000.0 + (distanceM >= 0 ? 0.5 : -0.5));

  uint8_t resultFrame[12] = {0};

  makeHeader(resultFrame, MSG_RESULT, seq);
  writeI32(resultFrame + 4, distanceMm);

  resultFrame[8] = 'R';
  resultFrame[9] = 'E';
  resultFrame[10] = 'S';

  uint64_t resultTxTs = _dw.calculateDelayedTransmitTimestamp(
    finalRxTs,
    uusToDw1000Ticks(_cfg.resultDelayUus)
  );

  if (_dw.transmitDelayedAt(resultFrame, sizeof(resultFrame), resultTxTs)) {
     _resultTxCount++;
    _anchorState = ANCHOR_WAIT_RESULT_TX;
    _stateMs = millis();
    _status = "anchor_result_scheduled";
  } else {
    _anchorState = ANCHOR_WAIT_POLL;
    _dw.startReceive();
    _status = "anchor_result_failed";
  }
}

void Dw1000Range::storeDistance(uint8_t seq, float distanceM) {
  _last.valid = isfinite(distanceM) && distanceM > -100.0f && distanceM < 1000.0f;

  _last.rawM = distanceM;
  _last.rawCm = distanceM * 100.0f;
  _last.correctedCm = _last.rawCm * _scale + _offsetCm;
  _last.rxPowerDbm = _dw.getReceivePowerDbm();

  _last.sequence = seq;
  _last.timestampMs = millis();

  _last.quality = _lastRxQuality;

  _newReading = _last.valid;
}

// =====================================================
// Math / helpers
// =====================================================
uint32_t Dw1000Range::uusToDw1000Ticks(uint32_t uus) {
  return uus * UUS_TO_DW1000_TIME;
}

uint64_t Dw1000Range::diff40(uint64_t later, uint64_t earlier) {
  return (later - earlier) & DW1000_TIME40_MASK_LOCAL;
}

void Dw1000Range::makeHeader(uint8_t* frame, uint8_t type, uint8_t seq) {
  frame[0] = MAGIC0;
  frame[1] = MAGIC1;
  frame[2] = type;
  frame[3] = seq;
}

bool Dw1000Range::validHeader(const uint8_t* frame, uint16_t len) {
  return len >= 4 && frame[0] == MAGIC0 && frame[1] == MAGIC1;
}

void Dw1000Range::writeTs40(uint8_t* dst, uint64_t value) {
  value &= DW1000_TIME40_MASK_LOCAL;

  for (uint8_t i = 0; i < 5; i++) {
    dst[i] = (uint8_t)((value >> (8 * i)) & 0xFF);
  }
}

uint64_t Dw1000Range::readTs40(const uint8_t* src) {
  uint64_t value = 0;

  for (uint8_t i = 0; i < 5; i++) {
    value |= ((uint64_t)src[i]) << (8 * i);
  }

  return value & DW1000_TIME40_MASK_LOCAL;
}

void Dw1000Range::writeI32(uint8_t* dst, int32_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
  dst[2] = (uint8_t)((value >> 16) & 0xFF);
  dst[3] = (uint8_t)((value >> 24) & 0xFF);
}

int32_t Dw1000Range::readI32(const uint8_t* src) {
  return (int32_t)(
    (uint32_t)src[0] |
    ((uint32_t)src[1] << 8) |
    ((uint32_t)src[2] << 16) |
    ((uint32_t)src[3] << 24)
  );
}

double Dw1000Range::calculateDsTwrDistanceM(uint64_t pollTx,
                                            uint64_t pollRx,
                                            uint64_t respTx,
                                            uint64_t respRx,
                                            uint64_t finalTx,
                                            uint64_t finalRx) {
  double Ra = (double)diff40(respRx, pollTx);
  double Rb = (double)diff40(finalRx, respTx);
  double Da = (double)diff40(finalTx, respRx);
  double Db = (double)diff40(respTx, pollRx);

  double denominator = Ra + Rb + Da + Db;

  if (denominator <= 0.0) {
    return NAN;
  }

  double tofTicks = ((Ra * Rb) - (Da * Db)) / denominator;

  return tofTicks * DISTANCE_PER_UWB_TICK_M;
}

void Dw1000Range::getDebugInfo(Dw1000RangeDebug& out) {
  out.status = _status;

  out.irqLevel = _dw.getIrqLevel();
  out.lastSysStatus = _lastSysStatus;
  out.sysState = _dw.getSystemStateRaw();

  out.txDoneCount = _txDoneCount;
  out.rxDoneCount = _rxDoneCount;
  out.rxErrorCount = _rxErrorCount;
  out.txErrorCount = _txErrorCount;
  out.timeoutCount = _timeoutCount;

  out.pollTxCount = _pollTxCount;
  out.pollRxCount = _pollRxCount;
  out.respTxCount = _respTxCount;
  out.respRxCount = _respRxCount;
  out.finalTxCount = _finalTxCount;
  out.finalRxCount = _finalRxCount;
  out.resultTxCount = _resultTxCount;
  out.resultRxCount = _resultRxCount;
}

void Dw1000Range::getLastRxQuality(Dw1000RxQuality& out) {
  out = _lastRxQuality;
}