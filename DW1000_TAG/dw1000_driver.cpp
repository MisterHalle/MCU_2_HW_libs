#include "dw1000_driver.h"
#include <math.h>

// =====================================================
// DW1000 register addresses
// =====================================================
static constexpr uint8_t REG_DEV_ID     = 0x00;
static constexpr uint8_t REG_SYS_config    = 0x04;
static constexpr uint8_t REG_SYS_TIME   = 0x06;
static constexpr uint8_t REG_TX_FCTRL   = 0x08;
static constexpr uint8_t REG_TX_BUFFER  = 0x09;
static constexpr uint8_t REG_DX_TIME    = 0x0A;
static constexpr uint8_t REG_SYS_CTRL   = 0x0D;
static constexpr uint8_t REG_SYS_MASK   = 0x0E;
static constexpr uint8_t REG_SYS_STATUS = 0x0F;
static constexpr uint8_t REG_RX_FINFO   = 0x10;
static constexpr uint8_t REG_RX_BUFFER  = 0x11;
static constexpr uint8_t REG_RX_FQUAL   = 0x12;
static constexpr uint8_t REG_RX_TIME    = 0x15;
static constexpr uint8_t REG_TX_TIME    = 0x17;
static constexpr uint8_t REG_TX_ANTD    = 0x18;
static constexpr uint8_t REG_SYS_STATE  = 0x19;
static constexpr uint8_t REG_TX_POWER   = 0x1E;
static constexpr uint8_t REG_CHAN_CTRL  = 0x1F;
static constexpr uint8_t REG_AGC_TUNE   = 0x23;
static constexpr uint8_t REG_DRX_TUNE   = 0x27;
static constexpr uint8_t REG_RF_CONF    = 0x28;
static constexpr uint8_t REG_TX_CAL     = 0x2A;
static constexpr uint8_t REG_FS_CTRL    = 0x2B;
static constexpr uint8_t REG_OTP_IF     = 0x2D;
static constexpr uint8_t REG_LDE_IF     = 0x2E;
static constexpr uint8_t REG_PMSC       = 0x36;

static constexpr uint32_t DW1000_EXPECTED_DEV_ID = 0xDECA0130UL;

static constexpr uint16_t NO_SUB = 0xFFFF;

// =====================================================
// SYS_CTRL bits
// =====================================================
static constexpr uint32_t SYS_CTRL_TXSTRT = 1UL << 1;
static constexpr uint32_t SYS_CTRL_TXDLYS = 1UL << 2;
static constexpr uint32_t SYS_CTRL_TRXOFF = 1UL << 6;
static constexpr uint32_t SYS_CTRL_RXENAB = 1UL << 8;

// =====================================================
// SYS_STATUS bits
// =====================================================
static constexpr uint32_t SYS_STATUS_TXFRS   = 1UL << 7;
static constexpr uint32_t SYS_STATUS_RXFCG   = 1UL << 14;
static constexpr uint32_t SYS_STATUS_RXFCE   = 1UL << 15;
static constexpr uint32_t SYS_STATUS_RXRFSL  = 1UL << 16;
static constexpr uint32_t SYS_STATUS_RXRFTO  = 1UL << 17;
static constexpr uint32_t SYS_STATUS_LDEERR  = 1UL << 18;
static constexpr uint32_t SYS_STATUS_RXOVRR  = 1UL << 20;
static constexpr uint32_t SYS_STATUS_RXPTO   = 1UL << 21;
static constexpr uint32_t SYS_STATUS_RXSFDTO = 1UL << 26;
static constexpr uint32_t SYS_STATUS_HPDWARN = 1UL << 27;
static constexpr uint32_t SYS_STATUS_TXBERR  = 1UL << 28;

static constexpr uint32_t STATUS_RX_ERRORS =
  SYS_STATUS_RXFCE |
  SYS_STATUS_RXRFSL |
  SYS_STATUS_RXRFTO |
  SYS_STATUS_LDEERR |
  SYS_STATUS_RXOVRR |
  SYS_STATUS_RXPTO |
  SYS_STATUS_RXSFDTO;

static constexpr uint32_t STATUS_TX_ERRORS =
  SYS_STATUS_HPDWARN |
  SYS_STATUS_TXBERR;

static constexpr uint32_t STATUS_USED =
  SYS_STATUS_TXFRS |
  SYS_STATUS_RXFCG |
  STATUS_RX_ERRORS |
  STATUS_TX_ERRORS;

static constexpr uint64_t DW1000_TIME40_MASK = 0xFFFFFFFFFFULL;
static constexpr uint64_t DW1000_DELAYED_TX_ALIGN_MASK = ~0x1FFULL;

static uint8_t txPreambleCodeFromLength(uint16_t preambleLength) {
  switch (preambleLength) {
    case 64:   return 0x04;
    case 128:  return 0x14;
    case 256:  return 0x24;
    case 512:  return 0x34;
    case 1024: return 0x08;
    case 1536: return 0x18;
    case 2048: return 0x28;
    case 4096: return 0x0C;
    default:   return 0x14; // 128 por seguridad
  }
}

static uint16_t drxTune0bFromDataRate(uint8_t dataRate) {
  switch (dataRate) {
    case DW1000_DATA_RATE_110K:
      return 0x0016;

    case DW1000_DATA_RATE_850K:
      return 0x0006;

    case DW1000_DATA_RATE_6800K:
    default:
      return 0x0001;
  }
}

static uint16_t drxTune1bFromPac(uint8_t pacSize) {
  switch (pacSize) {
    case 8:  return 0x0010;
    case 16: return 0x0020;
    case 32: return 0x0064;
    case 64: return 0x0028;
    default: return 0x0010;
  }
}

static uint32_t drxTune2FromPacPrf64(uint8_t pacSize) {
  switch (pacSize) {
    case 8:  return 0x313B006BUL;
    case 16: return 0x333B00BEUL;
    case 32: return 0x353B015EUL;
    case 64: return 0x373B0296UL;
    default: return 0x313B006BUL;
  }
}

static uint16_t sfdTimeoutFromProfile(uint16_t preambleLength, uint8_t pacSize, uint8_t dataRate) {
  uint16_t sfdLength = 8;

  if (dataRate == DW1000_DATA_RATE_110K) {
    sfdLength = 64;
  }

  return preambleLength + 1 + sfdLength - pacSize;
}

// =====================================================
// SPI helpers
// =====================================================
void Dw1000Driver::select() {
  _spi->beginTransaction(_settings);
  digitalWrite(_pins.cs, LOW);
}

void Dw1000Driver::deselect() {
  digitalWrite(_pins.cs, HIGH);
  _spi->endTransaction();
}

void Dw1000Driver::readBytes(uint8_t reg, uint16_t sub, uint8_t* data, uint16_t len) {
  uint8_t header[3];
  uint8_t hlen = 1;

  if (sub == NO_SUB) {
    header[0] = reg & 0x3F;
  } else {
    header[0] = 0x40 | (reg & 0x3F);

    if (sub < 128) {
      header[1] = (uint8_t)sub;
      hlen = 2;
    } else {
      header[1] = 0x80 | (uint8_t)(sub & 0x7F);
      header[2] = (uint8_t)(sub >> 7);
      hlen = 3;
    }
  }

  select();

  for (uint8_t i = 0; i < hlen; i++) {
    _spi->transfer(header[i]);
  }

  for (uint16_t i = 0; i < len; i++) {
    data[i] = _spi->transfer(0x00);
  }

  deselect();
}

void Dw1000Driver::writeBytes(uint8_t reg, uint16_t sub, const uint8_t* data, uint16_t len) {
  uint8_t header[3];
  uint8_t hlen = 1;

  if (sub == NO_SUB) {
    header[0] = 0x80 | (reg & 0x3F);
  } else {
    header[0] = 0xC0 | (reg & 0x3F);

    if (sub < 128) {
      header[1] = (uint8_t)sub;
      hlen = 2;
    } else {
      header[1] = 0x80 | (uint8_t)(sub & 0x7F);
      header[2] = (uint8_t)(sub >> 7);
      hlen = 3;
    }
  }

  select();

  for (uint8_t i = 0; i < hlen; i++) {
    _spi->transfer(header[i]);
  }

  for (uint16_t i = 0; i < len; i++) {
    _spi->transfer(data[i]);
  }

  deselect();
}

uint32_t Dw1000Driver::readValue(uint8_t reg, uint16_t sub, uint8_t len) {
  uint8_t b[4] = {0, 0, 0, 0};

  if (len > 4) len = 4;

  readBytes(reg, sub, b, len);

  uint32_t v = 0;
  for (uint8_t i = 0; i < len; i++) {
    v |= ((uint32_t)b[i]) << (8 * i);
  }

  return v;
}

void Dw1000Driver::writeValue(uint8_t reg, uint16_t sub, uint32_t value, uint8_t len) {
  uint8_t b[4];

  if (len > 4) len = 4;

  for (uint8_t i = 0; i < len; i++) {
    b[i] = (uint8_t)((value >> (8 * i)) & 0xFF);
  }

  writeBytes(reg, sub, b, len);
}

// =====================================================
// Init
// =====================================================
bool Dw1000Driver::begin(const Dw1000Pins& pins, const Dw1000Config& config, SPIClass& spi) {
  _pins = pins;
  _config = config;
  _spi = &spi;
  _ok = false;
  _lastDeviceId = 0;

  _spiMode = config.spiMode;
  _spiHz = config.initSpiHz;
  _settings = SPISettings(_spiHz, MSBFIRST, _spiMode);

  pinMode(_pins.cs, OUTPUT);
  digitalWrite(_pins.cs, HIGH);

  if (_pins.irq >= 0) {
    pinMode(_pins.irq, INPUT);
  }

  _spi->begin(_pins.sck, _pins.miso, _pins.mosi, _pins.cs);
  delay(5);

  if (_pins.rst >= 0) {
    pinMode(_pins.rst, OUTPUT);
    digitalWrite(_pins.rst, LOW);
    delay(5);

    // RSTn no se debe forzar HIGH permanentemente.
    pinMode(_pins.rst, INPUT_PULLUP);
    delay(20);
  } else {
    delay(50);
  }

  if (!detectSpiSettings(config)) {
    _ok = false;
    return false;
  }

  _ok = true;
  return configure(config);
}

bool Dw1000Driver::detectSpiSettings(const Dw1000Config& config) {
  if (!config.autoDetectSpi) {
    _spiHz = config.initSpiHz;
    _spiMode = config.spiMode;
    _settings = SPISettings(_spiHz, MSBFIRST, _spiMode);

    return getDeviceId() == DW1000_EXPECTED_DEV_ID;
  }

  const uint32_t speeds[] = {
    config.initSpiHz,
    500000UL,
    1000000UL,
    2000000UL
  };

  const uint8_t modes[] = {
    config.spiMode,
    SPI_MODE0,
    SPI_MODE1,
    SPI_MODE2,
    SPI_MODE3
  };

  for (uint8_t si = 0; si < sizeof(speeds) / sizeof(speeds[0]); si++) {
    if (speeds[si] == 0) continue;

    for (uint8_t mi = 0; mi < sizeof(modes) / sizeof(modes[0]); mi++) {
      bool repeatedMode = false;

      for (uint8_t mj = 0; mj < mi; mj++) {
        if (modes[mj] == modes[mi]) {
          repeatedMode = true;
        }
      }

      if (repeatedMode) continue;

      _spiHz = speeds[si];
      _spiMode = modes[mi];
      _settings = SPISettings(_spiHz, MSBFIRST, _spiMode);

      delay(2);

      uint32_t id1 = getDeviceId();
      delay(1);
      uint32_t id2 = getDeviceId();

      if (id1 == DW1000_EXPECTED_DEV_ID || id2 == DW1000_EXPECTED_DEV_ID) {
        if (config.spiHz > 0 && config.spiHz != _spiHz) {
          uint32_t previousHz = _spiHz;
          uint8_t previousMode = _spiMode;

          _settings = SPISettings(config.spiHz, MSBFIRST, _spiMode);
          _spiHz = config.spiHz;

          delay(1);

          if (getDeviceId() != DW1000_EXPECTED_DEV_ID) {
            _spiHz = previousHz;
            _spiMode = previousMode;
            _settings = SPISettings(_spiHz, MSBFIRST, _spiMode);
          }
        }

        _lastDeviceId = DW1000_EXPECTED_DEV_ID;
        return true;
      }
    }
  }

  _settings = SPISettings(config.initSpiHz, MSBFIRST, config.spiMode);
  _spiHz = config.initSpiHz;
  _spiMode = config.spiMode;
  _lastDeviceId = getDeviceId();

  return false;
}

bool Dw1000Driver::readExpectedDeviceId() {
  return getDeviceId() == DW1000_EXPECTED_DEV_ID;
}

bool Dw1000Driver::configure(const Dw1000Config& config) {
  _config = config;
  _antennaDelay = config.antennaDelay;

  _dataRate = config.dataRate;
  _preambleLength = config.preambleLength;
  _pacSize = config.pacSize;

  forceTrxOff();
  clearAllStatus();

  /*
    SYS_config:
    - HIRQ_POL = 1: IRQ activo en HIGH.
    - DIS_DRXB = 1: desactiva double RX buffer para simplificar debug.
    - RXAUTR = 0: no auto-reenable por ahora.
    - FFEN = 0: frame filtering desactivado.
  */
  uint32_t sysconfig = 0;
  sysconfig |= (1UL << 9);   // HIRQ_POL
  sysconfig |= (1UL << 12);  // DIS_DRXB

  writeValue(REG_SYS_config, NO_SUB, sysconfig, 4);

  /*
    Interrupciones/eventos monitoreados.
    Aunque ahora leemos SYS_STATUS por polling, mantener SYS_MASK
    coherente nos ayuda a que IRQ funcione cuando queramos usarlo.
  */
  writeValue(REG_SYS_MASK, NO_SUB, STATUS_USED, 4);

  /*
    Tuning RF/DRX para:
    - Canal 5
    - PRF 64 MHz
    - Data rate 6.8 Mbps
    - Preambulo 128
    - PAC8
    - Preamble code 9
  */
  applyDefaultTuning();

  /*
    Carga LDE.
    Necesaria para timestamps RX correctos y recepción estable.
  */
  loadLdeMicrocode();

  /*
    CHAN_CTRL:
    Bits principales:
    - TX_CHAN: canal TX
    - RX_CHAN: canal RX
    - RXPRF: PRF del receptor
    - TX_PCODE: preamble code TX
    - RX_PCODE: preamble code RX

    Este era el punto crítico:
    antes configurábamos TXPRF en TX_FCTRL, pero NO RXPRF en CHAN_CTRL.
    Si RXPRF queda mal, el receptor puede no detectar nada.
  */

  uint8_t channel = config.channel;
  uint8_t preambleCode = config.preambleCode;

  // Seguridad: para esta etapa estamos validando canal 5 / code 9.
  // Si en el futuro cambiamos canal/code, conviene ajustar tuning asociado.
  if (channel == 0) {
    channel = 5;
  }

  if (preambleCode == 0) {
    preambleCode = 9;
  }

  uint32_t chanCtrl = 0;

  // TX_CHAN bits 0..3
  chanCtrl |= ((uint32_t)channel & 0x0F);

  // RX_CHAN bits 4..7
  chanCtrl |= (((uint32_t)channel & 0x0F) << 4);

  // RXPRF bits 18..19
  // 1 = 16 MHz, 2 = 64 MHz.
  // Nuestro TX_FCTRL usa TXPRF = 2, por lo tanto RXPRF tambien debe ser 2.
  chanCtrl |= (2UL << 18);

  // TX_PCODE bits 22..26
  chanCtrl |= (((uint32_t)preambleCode & 0x1F) << 22);

  // RX_PCODE bits 27..31
  chanCtrl |= (((uint32_t)preambleCode & 0x1F) << 27);

  writeValue(REG_CHAN_CTRL, NO_SUB, chanCtrl, 4);

  /*
    Antenna delay TX/RX.
  */
  setAntennaDelay(config.antennaDelay);

  /*
    Dejamos RX activo al final de la configuracion.
    En modo TX, transmit() hará forceTrxOff() antes de transmitir.
  */
  startReceive();

  return true;
}

void Dw1000Driver::applyDefaultTuning() {
  // Perfil actual:
  // CH5, PRF64, preamble code 9.
  // El data rate, preamble length y PAC se eligen desde:
  // _dataRate, _preambleLength y _pacSize.

  // AGC tuning
  writeValue(REG_AGC_TUNE, 0x04, 0x8870, 2);
  writeValue(REG_AGC_TUNE, 0x0C, 0x2502A907UL, 4);
  writeValue(REG_AGC_TUNE, 0x12, 0x0035, 2);

  // DRX tuning
  writeValue(REG_DRX_TUNE, 0x02, drxTune0bFromDataRate(_dataRate), 2);
  writeValue(REG_DRX_TUNE, 0x04, 0x008D, 2); // PRF64
  writeValue(REG_DRX_TUNE, 0x06, drxTune1bFromPac(_pacSize), 2);
  writeValue(REG_DRX_TUNE, 0x08, drxTune2FromPacPrf64(_pacSize), 4);

  uint16_t sfdTimeout = sfdTimeoutFromProfile(_preambleLength, _pacSize, _dataRate);
  writeValue(REG_DRX_TUNE, 0x20, sfdTimeout, 2);

  // Sin timeout de preambulo por ahora.
  writeValue(REG_DRX_TUNE, 0x24, 0x0000, 2);

  // PRF64
  writeValue(REG_DRX_TUNE, 0x26, 0x0028, 2);

  // RF tuning canal 5
  writeValue(REG_RF_CONF, 0x0B, 0xD8, 1);
  writeValue(REG_RF_CONF, 0x0C, 0x001E3FE3UL, 4);

  // TX power canal 5.
  // Mantener por ahora para no abrir otra variable.
  writeValue(REG_TX_POWER, NO_SUB, 0x0E082848UL, 4);

  // Pulse generator delay canal 5
  writeValue(REG_TX_CAL, 0x0B, 0xC0, 1);

  // PLL canal 5
  writeValue(REG_FS_CTRL, 0x07, 0x0800041DUL, 4);
  writeValue(REG_FS_CTRL, 0x0B, 0xBE, 1);
}

void Dw1000Driver::loadLdeMicrocode() {
  // Secuencia usada por ejemplos/librerias DW1000 para timestamps RX precisos.
  writeValue(REG_PMSC, 0x00, 0x0301, 2);
  writeValue(REG_OTP_IF, 0x06, 0x8000, 2);
  delay(1);
  writeValue(REG_OTP_IF, 0x06, 0x0000, 2);
  writeValue(REG_PMSC, 0x00, 0x0200, 2);
}

// =====================================================
// Device ID / status
// =====================================================
uint32_t Dw1000Driver::getDeviceId() {
  _lastDeviceId = readValue(REG_DEV_ID, NO_SUB, 4);
  return _lastDeviceId;
}

bool Dw1000Driver::isConnected() {
  return getDeviceId() == DW1000_EXPECTED_DEV_ID;
}

uint32_t Dw1000Driver::getLastDeviceId() const {
  return _lastDeviceId;
}

uint8_t Dw1000Driver::getSpiMode() const {
  return _spiMode;
}

uint32_t Dw1000Driver::getSpiHz() const {
  return _spiHz;
}

// =====================================================
// TX/RX
// =====================================================
void Dw1000Driver::writeTxFrameControl(uint16_t payloadLength) {
  uint16_t frameLen = payloadLength + 2;

  if (frameLen > 127) {
    frameLen = 127;
  }

  uint8_t b[5] = {0, 0, 0, 0, 0};

  // TXFLEN bits 0..6
  b[0] = frameLen & 0x7F;

  // TXBR bits 13..14:
  // 0 = 110 kbps
  // 1 = 850 kbps
  // 2 = 6.8 Mbps
  b[1] |= ((_dataRate & 0x03) << 5);

  // TXPRF bits 16..17:
  // 2 = PRF 64 MHz
  b[2] |= 2;

  // Preamble length.
  b[2] |= txPreambleCodeFromLength(_preambleLength);

  writeBytes(REG_TX_FCTRL, NO_SUB, b, 5);
}

bool Dw1000Driver::transmit(const uint8_t* data, uint16_t length) {
  if (!_ok || !data || length == 0 || length > 125) {
    return false;
  }

  forceTrxOff();
  clearAllStatus();

  writeBytes(REG_TX_BUFFER, 0, data, length);
  writeTxFrameControl(length);

  writeValue(REG_SYS_CTRL, NO_SUB, SYS_CTRL_TXSTRT, 4);

  return true;
}

void Dw1000Driver::writeTimestamp40(uint8_t reg, uint16_t sub, uint64_t value) {
  uint8_t b[5];

  value &= DW1000_TIME40_MASK;

  for (uint8_t i = 0; i < 5; i++) {
    b[i] = (uint8_t)((value >> (8 * i)) & 0xFF);
  }

  writeBytes(reg, sub, b, 5);
}

uint64_t Dw1000Driver::readTimestamp40(uint8_t reg, uint16_t sub) {
  uint8_t b[5];

  readBytes(reg, sub, b, 5);

  uint64_t v = 0;

  for (uint8_t i = 0; i < 5; i++) {
    v |= ((uint64_t)b[i]) << (8 * i);
  }

  return v & DW1000_TIME40_MASK;
}

uint64_t Dw1000Driver::calculateDelayedTransmitTimestamp(uint64_t referenceTimestamp, uint32_t delayUwbTicks) const {
  uint64_t delayedStart = (referenceTimestamp + (uint64_t)delayUwbTicks) & DW1000_TIME40_MASK;
  delayedStart &= DW1000_DELAYED_TX_ALIGN_MASK;

  return (delayedStart + (uint64_t)_antennaDelay) & DW1000_TIME40_MASK;
}

bool Dw1000Driver::transmitDelayedAt(const uint8_t* data, uint16_t length, uint64_t delayedTxTimestamp) {
  if (!_ok || !data || length == 0 || length > 125) {
    return false;
  }

  uint64_t delayedStart = (delayedTxTimestamp - (uint64_t)_antennaDelay) & DW1000_TIME40_MASK;
  delayedStart &= DW1000_DELAYED_TX_ALIGN_MASK;

  forceTrxOff();
  clearAllStatus();

  writeTimestamp40(REG_DX_TIME, 0, delayedStart);
  writeBytes(REG_TX_BUFFER, 0, data, length);
  writeTxFrameControl(length);

  writeValue(REG_SYS_CTRL, NO_SUB, SYS_CTRL_TXSTRT | SYS_CTRL_TXDLYS, 4);

  return true;
}

bool Dw1000Driver::startReceive() {
  if (!_ok) {
    return false;
  }

  forceTrxOff();
  clearAllStatus();

  writeValue(REG_SYS_CTRL, NO_SUB, SYS_CTRL_RXENAB, 4);

  return true;
}

void Dw1000Driver::forceTrxOff() {
  writeValue(REG_SYS_CTRL, NO_SUB, SYS_CTRL_TRXOFF, 4);
}

Dw1000Events Dw1000Driver::handleIrq() {
  Dw1000Events ev;

  if (!_ok) {
    return ev;
  }

  uint32_t st = readValue(REG_SYS_STATUS, NO_SUB, 4);

  _lastIrqStatus = st;

  ev.rawStatus = st;
  ev.txDone = st & SYS_STATUS_TXFRS;
  ev.rxDone = st & SYS_STATUS_RXFCG;
  ev.rxError = st & STATUS_RX_ERRORS;
  ev.txError = st & STATUS_TX_ERRORS;

  uint32_t clearMask = st & STATUS_USED;
  if (clearMask) {
    writeValue(REG_SYS_STATUS, NO_SUB, clearMask, 4);
  }

  if (ev.rxError || ev.txError) {
    forceTrxOff();
  }

  return ev;
}

bool Dw1000Driver::irqActive() const {
  if (_pins.irq < 0) {
    return false;
  }

  return digitalRead(_pins.irq) == HIGH;
}

bool Dw1000Driver::readReceivedData(uint8_t* buffer, uint16_t& length, uint16_t maxLength) {
  length = 0;

  if (!buffer || maxLength == 0) {
    return false;
  }

  uint32_t finfo = readValue(REG_RX_FINFO, NO_SUB, 4);
  uint16_t frameLen = finfo & 0x7F;

  if (frameLen < 2) {
    return false;
  }

  uint16_t payloadLen = frameLen - 2;

  if (payloadLen > maxLength) {
    payloadLen = maxLength;
  }

  readBytes(REG_RX_BUFFER, 0, buffer, payloadLen);

  length = payloadLen;
  return length > 0;
}

// =====================================================
// Timestamps
// =====================================================
uint64_t Dw1000Driver::getTransmitTimestamp() {
  return readTimestamp40(REG_TX_TIME, 0);
}

uint64_t Dw1000Driver::getReceiveTimestamp() {
  return readTimestamp40(REG_RX_TIME, 0);
}

uint64_t Dw1000Driver::getSystemTimestamp() {
  return readTimestamp40(REG_SYS_TIME, 0);
}

uint64_t Dw1000Driver::getSystemStateRaw() {
  uint8_t b[5] = {0, 0, 0, 0, 0};

  readBytes(REG_SYS_STATE, NO_SUB, b, 5);

  uint64_t v = 0;

  for (uint8_t i = 0; i < 5; i++) {
    v |= ((uint64_t)b[i]) << (8 * i);
  }

  return v;
}
// =====================================================
// Antenna delay / power / recovery
// =====================================================
void Dw1000Driver::setAntennaDelay(uint16_t delay) {
  _antennaDelay = delay;

  writeValue(REG_TX_ANTD, NO_SUB, delay, 2);
  writeValue(REG_LDE_IF, 0x1804, delay, 2); // LDE_RXANTD
}

uint16_t Dw1000Driver::getAntennaDelay() const {
  return _antennaDelay;
}

float Dw1000Driver::getReceivePowerDbm() {
  // Placeholder por ahora.
  // Luego calculamos potencia real usando RX_FQUAL/RXPACC.
  (void)readValue(REG_RX_FQUAL, NO_SUB, 4);
  return NAN;
}

void Dw1000Driver::softResetRadio() {
  forceTrxOff();
  clearAllStatus();
  configure(_config);
}

void Dw1000Driver::clearAllStatus() {
  writeValue(REG_SYS_STATUS, NO_SUB, 0xFFFFFFFFUL, 4);
}

uint32_t Dw1000Driver::getLastIrqStatus() const {
  return _lastIrqStatus;
}

bool Dw1000Driver::getIrqLevel() const {
  if (_pins.irq < 0) {
    return false;
  }

  return digitalRead(_pins.irq) == HIGH;
}

bool Dw1000Driver::readRxQuality(Dw1000RxQuality& out) {
  out = Dw1000RxQuality();

  /*
    Registros usados:

    RX_FINFO  0x10:
      - RXPACC: contador de simbolos de preambulo acumulados.

    RX_FQUAL  0x12:
      offset 0x00: STD_NOISE
      offset 0x02: FP_AMPL2
      offset 0x04: FP_AMPL3
      offset 0x06: CIR_PWR

    RX_TIME   0x15:
      offset 0x05: FP_INDEX
      offset 0x07: FP_AMPL1
  */

  uint8_t finfoBytes[4] = {0, 0, 0, 0};
  uint8_t fqualBytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t rxtimeDiag[4] = {0, 0, 0, 0};

  readBytes(REG_RX_FINFO, NO_SUB, finfoBytes, 4);
  readBytes(REG_RX_FQUAL, NO_SUB, fqualBytes, 8);
  readBytes(REG_RX_TIME, 0x05, rxtimeDiag, 4);

  uint32_t rxFinfo =
    ((uint32_t)finfoBytes[0]) |
    ((uint32_t)finfoBytes[1] << 8) |
    ((uint32_t)finfoBytes[2] << 16) |
    ((uint32_t)finfoBytes[3] << 24);

  out.rxPacc = (uint16_t)((rxFinfo >> 20) & 0x0FFF);

  out.stdNoise =
    ((uint16_t)fqualBytes[0]) |
    ((uint16_t)fqualBytes[1] << 8);

  out.fpAmpl2 =
    ((uint16_t)fqualBytes[2]) |
    ((uint16_t)fqualBytes[3] << 8);

  out.fpAmpl3 =
    ((uint16_t)fqualBytes[4]) |
    ((uint16_t)fqualBytes[5] << 8);

  out.cirPower =
    ((uint16_t)fqualBytes[6]) |
    ((uint16_t)fqualBytes[7] << 8);

  out.fpIndexRaw =
    ((uint16_t)rxtimeDiag[0]) |
    ((uint16_t)rxtimeDiag[1] << 8);

  out.fpAmpl1 =
    ((uint16_t)rxtimeDiag[2]) |
    ((uint16_t)rxtimeDiag[3] << 8);

  // FP_INDEX tiene parte fraccional. Dividir por 64 ayuda a verlo como indice decimal.
  out.fpIndex = ((float)out.fpIndexRaw) / 64.0f;

  if (out.rxPacc == 0) {
    out.valid = false;
    return false;
  }

  /*
    Formula DW1000 para potencia recibida:
      rxPower = 10 * log10(CIR_PWR * 2^17 / RXPACC^2) - A

    Formula para first path:
      fpPower = 10 * log10((F1^2 + F2^2 + F3^2) / RXPACC^2) - A

    Para PRF64 usamos A = 121.74.
  */
  const double A_PRF64 = 121.74;
  const double rxPacc = (double)out.rxPacc;
  const double rxPaccSq = rxPacc * rxPacc;

  if (out.cirPower > 0) {
    double numerator = ((double)out.cirPower) * 131072.0; // 2^17
    out.rxPowerDbm = (float)(10.0 * log10(numerator / rxPaccSq) - A_PRF64);
  }

  double f1 = (double)out.fpAmpl1;
  double f2 = (double)out.fpAmpl2;
  double f3 = (double)out.fpAmpl3;
  double fpSum = (f1 * f1) + (f2 * f2) + (f3 * f3);

  if (fpSum > 0.0) {
    out.fpPowerDbm = (float)(10.0 * log10(fpSum / rxPaccSq) - A_PRF64);
  }

  out.powerDiffDb = out.rxPowerDbm - out.fpPowerDbm;
  out.valid = true;

  return true;
}