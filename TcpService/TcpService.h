/* Copyright 2026 Hall-e SpA

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     www.apache.org

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License. */

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "NodeDiscovery.h"
#include <WiFiMulti.h>

#ifndef TCP_SERVICE_RX_LINE_MAX
#define TCP_SERVICE_RX_LINE_MAX 256
#endif

#ifndef TCP_SERVICE_RX_JSON_CAPACITY
#define TCP_SERVICE_RX_JSON_CAPACITY 256
#endif

#ifndef TCP_SERVICE_WIFI_POOL_JSON_CAPACITY
#define TCP_SERVICE_WIFI_POOL_JSON_CAPACITY 768
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

  // Pool opcional de redes WiFi en formato JSON.
  // Si useWifiPool = true, TcpService intentara conectarse usando WiFiMulti.
  // Si falla o esta desactivado, puede seguir usando ssid/pass normal.
  bool useWifiPool = false;
  const char* wifiPoolJson = nullptr;

  // Tiempo interno de cada intento WiFiMulti.run().
  uint32_t wifiMultiRunTimeoutMs = 1000;

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
  WiFiMulti _wifiMulti;

  bool _wifiPoolLoaded = false;
  uint8_t _wifiPoolCount = 0;

  bool loadWiFiPool();
  bool connectWifiFromPoolBlocking();

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

  void handleDiscoveryCleanRequest();

  void readIncomingTcp();
  void processIncomingLine(const char* line);

  bool remoteEndpointValid() const;
  bool validateRemotePort() const;
};