#ifndef DW1000_DRIVER_H
#define DW1000_DRIVER_H

#include <Arduino.h>
#include <SPI.h>

// Minimal self-contained DW1000 driver for Arduino/ESP32.
// No dependency on UWB_DW1000_DW3000, DW1000Ng, DW3000Ng, or HardwareSerial.

struct Dw1000Pins {
  int cs = -1;
  int irq = -1;
  int rst = -1;
  int sck = -1;
  int miso = -1;
  int mosi = -1;
};

enum Dw1000DataRate : uint8_t {
  DW1000_DATA_RATE_110K  = 0,
  DW1000_DATA_RATE_850K  = 1,
  DW1000_DATA_RATE_6800K = 2
};

struct Dw1000Config {
  uint8_t channel = 5;
  uint8_t preambleCode = 9;

  uint8_t dataRate = DW1000_DATA_RATE_6800K;
  uint16_t preambleLength = 128;
  uint8_t pacSize = 8;

  uint16_t antennaDelay = 16436;

  uint32_t initSpiHz = 1000000UL;
  uint32_t spiHz = 2000000UL;
  uint8_t spiMode = SPI_MODE0;
  bool autoDetectSpi = true;
};

struct Dw1000Events {
  bool txDone = false;
  bool rxDone = false;
  bool rxError = false;
  bool txError = false;
  uint32_t rawStatus = 0;
};

struct Dw1000RxQuality {
  bool valid = false;

  uint16_t stdNoise = 0;
  uint16_t fpAmpl1 = 0;
  uint16_t fpAmpl2 = 0;
  uint16_t fpAmpl3 = 0;
  uint16_t cirPower = 0;
  uint16_t rxPacc = 0;
  uint16_t fpIndexRaw = 0;

  float fpIndex = 0.0f;

  float rxPowerDbm = 0.0f;       // RSL aproximado
  float fpPowerDbm = 0.0f;       // FSL aproximado
  float powerDiffDb = 0.0f;      // rxPowerDbm - fpPowerDbm
};

class Dw1000Driver {
public:
  bool begin(const Dw1000Pins& pins, const Dw1000Config& config, SPIClass& spi = SPI);
  bool configure(const Dw1000Config& config);

  uint32_t getDeviceId();
  uint32_t getLastDeviceId() const;
  bool isConnected();

  uint8_t getSpiMode() const;
  uint32_t getSpiHz() const;

  uint32_t getLastIrqStatus() const;
  bool getIrqLevel() const;

  bool transmit(const uint8_t* data, uint16_t length);
  bool transmitDelayedAt(const uint8_t* data, uint16_t length, uint64_t delayedTxTimestamp);
  bool startReceive();
  void forceTrxOff();

  Dw1000Events handleIrq();
  bool irqActive() const;

  bool readReceivedData(uint8_t* buffer, uint16_t& length, uint16_t maxLength);

  uint64_t getTransmitTimestamp();
  uint64_t getReceiveTimestamp();
  uint64_t getSystemTimestamp();
  uint64_t getSystemStateRaw();

  uint64_t calculateDelayedTransmitTimestamp(uint64_t referenceTimestamp, uint32_t delayDw1000Ticks) const;

  void setAntennaDelay(uint16_t delay);
  uint16_t getAntennaDelay() const;
  bool readRxQuality(Dw1000RxQuality& out);

  float getReceivePowerDbm();

  void softResetRadio();
  void clearAllStatus();

private:
  Dw1000Pins _pins;
  Dw1000Config _config;
  SPIClass* _spi = nullptr;

  SPISettings _settings = SPISettings(1000000UL, MSBFIRST, SPI_MODE0);

  uint16_t _antennaDelay = 16436;
  bool _ok = false;
  uint32_t _lastDeviceId = 0;

  uint32_t _lastIrqStatus = 0;

  uint8_t _spiMode = SPI_MODE0;
  uint32_t _spiHz = 1000000UL;

  uint8_t _dataRate = DW1000_DATA_RATE_6800K;
  uint16_t _preambleLength = 128;
  uint8_t _pacSize = 8;

  void select();
  void deselect();

  void readBytes(uint8_t reg, uint16_t sub, uint8_t* data, uint16_t len);
  void writeBytes(uint8_t reg, uint16_t sub, const uint8_t* data, uint16_t len);

  uint32_t readValue(uint8_t reg, uint16_t sub, uint8_t len);
  void writeValue(uint8_t reg, uint16_t sub, uint32_t value, uint8_t len);

  void writeTxFrameControl(uint16_t payloadLength);

  void writeTimestamp40(uint8_t reg, uint16_t sub, uint64_t value);
  uint64_t readTimestamp40(uint8_t reg, uint16_t sub);

  bool detectSpiSettings(const Dw1000Config& config);
  bool readExpectedDeviceId();

  void applyDefaultTuning();
  void loadLdeMicrocode();
};

#endif