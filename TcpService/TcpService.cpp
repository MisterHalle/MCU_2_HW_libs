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

#include "TcpService.h"

TcpService::TcpService() {}

bool TcpService::begin(const TcpServiceConfig& config) {
  _config = config;

  _begun = true;
  _wifiWasConnected = false;
  _discoveryStarted = false;
  _masterReady = false;
  _tcpConnectedOnce = false;
  _lastTcpConnectedState = false;

  _remoteIP = IPAddress(0, 0, 0, 0);
  _remotePort = 0;
  _nodeName  = "";

  _hasFireInterval = false;
  _fireInterval = 0;

  _rxLen = 0;

  WiFi.mode(WIFI_STA);
  WiFi.persistent(_config.wifiPersistent);
  WiFi.setAutoReconnect(_config.wifiAutoReconnect);

  bool ok = connectWifiBlocking();

  if (!ok) {
    _state = TCP_SERVICE_ERROR;
    return false;
  }

  if (_config.mode == TCP_MODE_MANUAL) {
    updateManualEndpoint();
  } else {
    startDiscoveryIfNeeded();
  }

  return true;
}

void TcpService::update() {
  if (!_begun) return;

  updateWifi();

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (_config.mode == TCP_MODE_DISCOVERY) {
    startDiscoveryIfNeeded();
    updateDiscovery();
  } else {
    updateManualEndpoint();
  }

  updateTcp();
  readIncomingTcp();

  bool nowConnected = _client.connected();

  if (_lastTcpConnectedState && !nowConnected) {
    if (_config.debug) Serial.println("[TCP_SERVICE] TCP desconectado.");

    if (_tcpDisconnectedCallback) {
      _tcpDisconnectedCallback();
    }
  }

  _lastTcpConnectedState = nowConnected;
}

bool TcpService::wifiConnected() const {
  return WiFi.status() == WL_CONNECTED;
}

bool TcpService::tcpConnected() {
  return _client.connected();
}

bool TcpService::masterReady() const {
  return _masterReady;
}

bool TcpService::tcpConnectedOnce() const {
  return _tcpConnectedOnce;
}

TcpServiceState TcpService::state() const {
  return _state;
}

IPAddress TcpService::remoteIP() const {
  return _remoteIP;
}

uint16_t TcpService::remotePort() const {
  return _remotePort;
}

String TcpService::nodeName() const {
  return _nodeName;
}

bool TcpService::hasFireInterval() const {
  return _hasFireInterval;
}

uint32_t TcpService::fireInterval(uint32_t fallback) const {
  if (_hasFireInterval) {
    return _fireInterval;
  }

  return fallback;
}

WiFiClient& TcpService::client() {
  return _client;
}

void TcpService::resetDiscovery() {
  stopTcp();

  _masterReady = false;
  _discoveryStarted = false;

  _remoteIP = IPAddress(0, 0, 0, 0);
  _remotePort = 0;

  _nodeName = "";

  _hasFireInterval = false;
  _fireInterval = 0;

  _lastTcpTryMs = 0;

  _discovery.clearMaster();

  if (_config.debug) {
    Serial.println("[TCP_SERVICE] Discovery reiniciado.");
  }
}

void TcpService::stopTcp() {
  if (_client.connected()) {
    _client.stop();
  }

  _lastTcpConnectedState = false;
}

void TcpService::onJsonReceived(TcpJsonCallback cb) {
  _jsonCallback = cb;
}

void TcpService::onTcpConnected(TcpSimpleCallback cb) {
  _tcpConnectedCallback = cb;
}

void TcpService::onTcpDisconnected(TcpSimpleCallback cb) {
  _tcpDisconnectedCallback = cb;
}

bool TcpService::connectTcpNow() {
  if (!remoteEndpointValid()) {
    if (_config.debug) Serial.println("[TCP_SERVICE] Endpoint TCP invalido.");
    return false;
  }

  if (_client.connected()) {
    return true;
  }

  _client.stop();

  if (_config.debug) {
    Serial.print("[TCP_SERVICE] Intentando TCP ");
    Serial.print(_remoteIP);
    Serial.print(":");
    Serial.println(_remotePort);
  }

  bool ok = _client.connect(_remoteIP, _remotePort);

  if (ok) {
    _tcpConnectedOnce = true;
    _state = TCP_SERVICE_CONNECTED;

    if (_config.tcpNoDelay) {
      _client.setNoDelay(true);
    }

    if (_config.debug) {
      Serial.println("[TCP_SERVICE] TCP conectado correctamente.");
    }

    if (_tcpConnectedCallback) {
      _tcpConnectedCallback();
    }
  } else {
    _state = TCP_SERVICE_ERROR;

    if (_config.debug) {
      Serial.println("[TCP_SERVICE] Fallo conexion TCP.");
    }
  }

  return ok;
}

bool TcpService::loadWiFiPool() {
  _wifiPoolLoaded = false;
  _wifiPoolCount = 0;

  if (!_config.useWifiPool) {
    return false;
  }

  if (_config.wifiPoolJson == nullptr || strlen(_config.wifiPoolJson) == 0) {
    if (_config.debug) {
      Serial.println("[TCP_SERVICE] WiFi Pool vacio o no definido.");
    }

    return false;
  }

  StaticJsonDocument<TCP_SERVICE_WIFI_POOL_JSON_CAPACITY> wifiPool;

  DeserializationError error = deserializeJson(wifiPool, _config.wifiPoolJson);

  if (error) {
    if (_config.debug) {
      Serial.print("[TCP_SERVICE] Error cargando WiFi Pool: ");
      Serial.println(error.c_str());
    }

    return false;
  }

  JsonObject redes = wifiPool.as<JsonObject>();

  for (JsonPair red : redes) {
    JsonArray datos = red.value().as<JsonArray>();

    const char* ssid = datos[0] | "";
    const char* pass = datos[1] | "";

    if (strlen(ssid) > 0) {
      _wifiMulti.addAP(ssid, pass);
      _wifiPoolCount++;

      if (_config.debug) {
        Serial.print("[TCP_SERVICE][WiFi Pool] Agregada: ");
        Serial.println(ssid);
      }
    }
  }

  _wifiPoolLoaded = _wifiPoolCount > 0;

  if (_config.debug) {
    Serial.print("[TCP_SERVICE][WiFi Pool] Total redes cargadas: ");
    Serial.println(_wifiPoolCount);
  }

  return _wifiPoolLoaded;
}

bool TcpService::connectWifiFromPoolBlocking() {
  if (!_wifiPoolLoaded) {
    if (!loadWiFiPool()) {
      return false;
    }
  }

  _state = TCP_SERVICE_WIFI_CONNECTING;

  if (_config.debug) {
    Serial.println("[TCP_SERVICE] Conectando usando WiFi Pool...");
  }

  uint32_t t0 = millis();

  while (millis() - t0 < _config.wifiTimeoutMs) {
    uint8_t  status = _wifiMulti.run(_config.wifiMultiRunTimeoutMs);

    if (status == WL_CONNECTED) {
      _wifiWasConnected = true;
      _state = TCP_SERVICE_WIFI_CONNECTED;

      if (_config.debug) {
        Serial.println();
        Serial.println("[TCP_SERVICE] WiFi Pool conectado correctamente.");

        Serial.print("[TCP_SERVICE] SSID conectado: ");
        Serial.println(WiFi.SSID());

        Serial.print("[TCP_SERVICE] IP local: ");
        Serial.println(WiFi.localIP());
      }

      return true;
    }

    delay(100);
    yield();

    if (_config.debug) {
      Serial.print(".");
    }
  }

  if (_config.debug) {
    Serial.println();
    Serial.println("[TCP_SERVICE] Timeout conectando desde WiFi Pool.");
  }

  _wifiWasConnected = false;
  return false;
}

bool TcpService::connectWifiBlocking() {
  WiFi.mode(WIFI_STA);

  if (_config.useWifiPool) {
    bool poolOk = connectWifiFromPoolBlocking();

    if (poolOk) {
      return true;
    }

    if (_config.debug) {
      Serial.println("[TCP_SERVICE] WiFi Pool fallo. Intentando ssid/pass fallback...");
    }
  }

  if (_config.ssid == nullptr || strlen(_config.ssid) == 0) {
    Serial.println("[TCP_SERVICE] SSID vacio. No se puede conectar WiFi.");
    return false;
  }

  _state = TCP_SERVICE_WIFI_CONNECTING;

  if (_config.debug) {
    Serial.print("[TCP_SERVICE] Conectando a WiFi directo: ");
    Serial.println(_config.ssid);
  }

  WiFi.begin(_config.ssid, _config.pass, _config.wifiChannel);

  uint32_t t0 = millis();

  while (millis() - t0 < _config.wifiTimeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      _wifiWasConnected = true;
      _state = TCP_SERVICE_WIFI_CONNECTED;

      if (_config.debug) {
        Serial.println();
        Serial.print("[TCP_SERVICE] WiFi OK. IP local: ");
        Serial.println(WiFi.localIP());
      }

      return true;
    }

    delay(100);
    yield();

    if (_config.debug) {
      Serial.print(".");
    }
  }

  if (_config.debug) {
    Serial.println();
    Serial.println("[TCP_SERVICE] Timeout conectando WiFi directo.");
  }

  _wifiWasConnected = false;
  return false;
}

void TcpService::updateWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!_wifiWasConnected) {
      _wifiWasConnected = true;

      if (_config.debug) {
        Serial.print("[TCP_SERVICE] WiFi reconectado. IP: ");
        Serial.println(WiFi.localIP());
      }

      _discoveryStarted = false;
      _masterReady = false;

      _remoteIP = IPAddress(0, 0, 0, 0);
      _remotePort = 0;

      _nodeName = "";

      _hasFireInterval = false;
      _fireInterval = 0;

      _lastTcpTryMs = 0;
    }

    return;
  }

  if (_client.connected()) {
    _client.stop();
  }

  _wifiWasConnected = false;
  _discoveryStarted = false;
  _masterReady = false;
  _state = TCP_SERVICE_WIFI_CONNECTING;

  if (!_config.wifiAutoReconnect) {
    return;
  }

  uint32_t now = millis();

  if (now - _lastWifiTryMs < _config.wifiRetryIntervalMs) {
    return;
  }

  _lastWifiTryMs = now;

  if (_config.debug) {
    Serial.println("[TCP_SERVICE] Reintentando WiFi...");
  }

  if (_config.useWifiPool && _wifiPoolLoaded) {
    if (_config.debug) {
      Serial.println("[TCP_SERVICE] Reintentando WiFi desde Pool...");
    }

    _wifiMulti.run(_config.wifiMultiRunTimeoutMs);
  } 
  else {
    if (_config.debug) {
      Serial.println("[TCP_SERVICE] Reintentando WiFi directo...");
    }

    WiFi.disconnect(false);
    WiFi.begin(_config.ssid, _config.pass, _config.wifiChannel);
  }
}

void TcpService::startDiscoveryIfNeeded() {
  if (_discoveryStarted) return;
  if (WiFi.status() != WL_CONNECTED) return;

  _state = TCP_SERVICE_DISCOVERING;

  if (_config.debug) {
    Serial.println("[TCP_SERVICE] Iniciando NodeDiscovery...");
  }

  bool ok = _discovery.begin(_config.discovery);

  if (ok) {
    _discoveryStarted = true;
  } else {
    _state = TCP_SERVICE_ERROR;
  }
}

void TcpService::updateDiscovery() {
  if (!_discoveryStarted) return;

  _discovery.update();

  if (_discovery.consumeCleanRequest()) {
    handleDiscoveryCleanRequest();
    return;
  }

  if (!_discovery.hasMaster()) {
    return;
  }

  _remoteIP = _discovery.getMasterIP();
  _remotePort = _discovery.getMasterTcpPort();

  _nodeName = _discovery.getNodeName();

  _hasFireInterval = _discovery.hasFireInterval();
  _fireInterval = _discovery.getFireInterval(0);

  _masterReady = true;
  _state = TCP_SERVICE_MASTER_READY;
}

void TcpService::updateManualEndpoint() {
  if (_config.manualIP == IPAddress(0, 0, 0, 0)) {
    _masterReady = false;
    return;
  }

  if (_config.manualTcpPort == 0) {
    _masterReady = false;
    return;
  }

  _remoteIP = _config.manualIP;
  _remotePort = _config.manualTcpPort;

  _masterReady = true;
  _state = TCP_SERVICE_MASTER_READY;
}

void TcpService::updateTcp() {
  if (!_masterReady) return;

  if (_client.connected()) {
    return;
  }

  uint32_t now = millis();

  if (now - _lastTcpTryMs < _config.tcpRetryIntervalMs) {
    return;
  }

  _lastTcpTryMs = now;

  connectTcpNow();
}

void TcpService::readIncomingTcp() {
  if (!_client.connected()) return;

  while (_client.available()) {
    char c = (char)_client.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      _rxLine[_rxLen] = '\0';

      if (_rxLen > 0) {
        processIncomingLine(_rxLine);
      }

      _rxLen = 0;
      continue;
    }

    if (_rxLen < sizeof(_rxLine) - 1) {
      _rxLine[_rxLen++] = c;
    } else {
      _rxLen = 0;

      if (_config.debug) {
        Serial.println("[TCP_SERVICE] RX line overflow. Buffer limpiado.");
      }
    }
  }
}

void TcpService::processIncomingLine(const char* line) {
  if (_config.debug) {
    Serial.print("[TCP_SERVICE] RX: ");
    Serial.println(line);
  }

  StaticJsonDocument<TCP_SERVICE_RX_JSON_CAPACITY> doc;

  DeserializationError err = deserializeJson(doc, line);

  if (err) {
    if (_config.debug) {
      Serial.print("[TCP_SERVICE] deserializeJson() failed: ");
      Serial.println(err.c_str());
    }

    return;
  }

  if (_jsonCallback) {
    _jsonCallback(doc);
  }
}

bool TcpService::remoteEndpointValid() const {
  if (!_masterReady) return false;
  if (_remoteIP == IPAddress(0, 0, 0, 0)) return false;
  if (_remotePort == 0) return false;

  return true;
}

void TcpService::handleDiscoveryCleanRequest() {
  if (_config.debug) {
    Serial.println("[TCP_SERVICE] Clean UDP recibido. Reiniciando sesion TCP/discovery.");
  }

  stopTcp();

  _masterReady = false;

  _remoteIP = IPAddress(0, 0, 0, 0);
  _remotePort = 0;

  _nodeName = "";

  _hasFireInterval = false;
  _fireInterval = 0;

  _lastTcpTryMs = 0;

  _state = TCP_SERVICE_DISCOVERING;

  _discovery.clearMaster();
}