/*
  TCP_SERVICE_v1.1.0
  Archivo: TcpService.h

  Cambios:
  - bodyPart pasa a ser nodeName.
  - fireInterval es opcional.
  - Se mantiene bodyPart() como alias temporal.
  - TcpService queda como modulo TCP plug&play.
*/

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "NodeDiscovery.h"

#ifndef TCP_SERVICE_RX_LINE_MAX
#define TCP_SERVICE_RX_LINE_MAX 256
#endif

#ifndef TCP_SERVICE_RX_JSON_CAPACITY
#define TCP_SERVICE_RX_JSON_CAPACITY 256
#endif

enum TcpServiceMode : uint8_t {
  TCP_MODE_DISCOVERY = 0,
  TCP_MODE_MANUAL    = 1
};

enum TcpServiceState : uint8_t {
  TCP_SERVICE_IDLE = 0,
  TCP_SERVICE_WIFI_CONNECTING,
  TCP_SERVICE_WIFI_CONNECTED,
  TCP_SERVICE_DISCOVERING,
  TCP_SERVICE_MASTER_READY,
  TCP_SERVICE_CONNECTED,
  TCP_SERVICE_ERROR
};

typedef void (*TcpJsonCallback)(JsonDocument& doc);
typedef void (*TcpSimpleCallback)();

struct TcpServiceConfig {
  const char* ssid = "";
  const char* pass = "";

  int32_t wifiChannel = 0;
  uint32_t wifiTimeoutMs = 15000;
  uint32_t wifiRetryIntervalMs = 5000;

  bool wifiPersistent = false;
  bool wifiAutoReconnect = true;

  TcpServiceMode mode = TCP_MODE_DISCOVERY;

  NodeDiscoveryConfig discovery;

  IPAddress manualIP = IPAddress(0, 0, 0, 0);
  uint16_t manualTcpPort = 0;

  uint32_t tcpRetryIntervalMs = 3000;
  bool tcpNoDelay = true;

  bool validatePortOffset = false;
  uint16_t tcpBasePort = 0;
  int expectedPortOffset = 0;

  bool debug = true;
};

class TcpService {
public:
  TcpService();

  bool begin(const TcpServiceConfig& config);
  void update();

  bool wifiConnected() const;

  // No const por compatibilidad con ESP32 core 3.x.
  bool tcpConnected();

  bool masterReady() const;
  bool tcpConnectedOnce() const;

  TcpServiceState state() const;

  IPAddress remoteIP() const;
  uint16_t remotePort() const;

  String nodeName() const;

  bool hasFireInterval() const;
  uint32_t fireInterval(uint32_t fallback = 0) const;

  // Alias temporal para no romper codigo anterior.
  String bodyPart() const {
    return nodeName();
  }

  WiFiClient& client();

  void stopTcp();
  void resetDiscovery();

  void onJsonReceived(TcpJsonCallback cb);
  void onTcpConnected(TcpSimpleCallback cb);
  void onTcpDisconnected(TcpSimpleCallback cb);

  bool connectTcpNow();

  template<typename TDoc>
  bool sendJson(TDoc& doc, bool newline = true) {
    if (!_client.connected()) {
      if (_config.debug) Serial.println("[TCP_SERVICE] No conectado. JSON no enviado.");
      return false;
    }

    size_t n = serializeJson(doc, _client);

    if (newline) {
      _client.println();
    }

    if (n == 0) {
      if (_config.debug) Serial.println("[TCP_SERVICE] Error serializando JSON hacia TCP.");
      return false;
    }

    if (_config.debug) {
      Serial.print("[TCP_SERVICE] JSON enviado: ");
      serializeJson(doc, Serial);
      Serial.println();
    }

    return true;
  }

private:
  TcpServiceConfig _config;

  WiFiClient _client;
  NodeDiscovery _discovery;

  TcpServiceState _state = TCP_SERVICE_IDLE;

  bool _begun = false;
  bool _wifiWasConnected = false;
  bool _discoveryStarted = false;
  bool _masterReady = false;
  bool _tcpConnectedOnce = false;
  bool _lastTcpConnectedState = false;

  IPAddress _remoteIP = IPAddress(0, 0, 0, 0);
  uint16_t _remotePort = 0;

  String _nodeName = "";

  bool _hasFireInterval = false;
  uint32_t _fireInterval = 0;

  uint32_t _lastWifiTryMs = 0;
  uint32_t _lastTcpTryMs = 0;

  char _rxLine[TCP_SERVICE_RX_LINE_MAX];
  size_t _rxLen = 0;

  TcpJsonCallback _jsonCallback = nullptr;
  TcpSimpleCallback _tcpConnectedCallback = nullptr;
  TcpSimpleCallback _tcpDisconnectedCallback = nullptr;

  bool connectWifiBlocking();

  void updateWifi();
  void startDiscoveryIfNeeded();
  void updateDiscovery();
  void updateManualEndpoint();
  void updateTcp();

  void readIncomingTcp();
  void processIncomingLine(const char* line);

  bool remoteEndpointValid() const;
  bool validateRemotePort() const;
};