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

#include "NodeDiscovery.h"

NodeDiscovery::NodeDiscovery() {}

bool NodeDiscovery::begin(const NodeDiscoveryConfig& config) {
  _config = config;
  normalizeConfig();

  _hasMaster = false;
  _cleanRequested = false;
  _masterIP = IPAddress(0, 0, 0, 0);
  _masterTcpPort = _config.defaultTcpPort;

  _nodeName = _config.defaultNodeName;

  _hasFireInterval = false;
  _fireInterval = _config.defaultFireInterval;

  _beginMs = millis();
  _lastHelloMs = 0;
  

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[DISCOVERY] WiFi no conectado. No se puede iniciar UDP.");
    return false;
  }

  _broadcastIP = computeBroadcastIP();

  _udpReady = _udp.begin(_config.udpListenPort);

  if (!_udpReady) {
    Serial.println("[DISCOVERY] Error iniciando UDP.");
    return false;
  }

  Serial.println("[DISCOVERY] UDP iniciado correctamente.");

  Serial.print("[DISCOVERY] IP local: ");
  Serial.println(WiFi.localIP());

  Serial.print("[DISCOVERY] Broadcast IP: ");
  Serial.println(_broadcastIP);

  Serial.print("[DISCOVERY] UDP send port: ");
  Serial.println(_config.udpSendPort);

  Serial.print("[DISCOVERY] UDP listen port: ");
  Serial.println(_config.udpListenPort);

  Serial.print("[DISCOVERY] Hello interval ms: ");
  Serial.println(_config.helloIntervalMs);

  return true;
}

void NodeDiscovery::update() {
  if (!_udpReady) return;
  if (WiFi.status() != WL_CONNECTED) return;

  readUdpPacket();

  if (_hasMaster) return;

  uint32_t now = millis();

  if (now - _lastHelloMs >= _config.helloIntervalMs) {
    _lastHelloMs = now;
    sendHello();
  }

  if ((now - _beginMs) > _config.discoveryTimeoutMs) {
    _beginMs = now;
    Serial.println("[DISCOVERY] Maestro aun no detectado. Continuando broadcast JSON...");
  }
}

bool NodeDiscovery::hasMaster() const {
  return _hasMaster;
}

IPAddress NodeDiscovery::getMasterIP() const {
  return _masterIP;
}

uint16_t NodeDiscovery::getMasterTcpPort() const {
  return _masterTcpPort;
}

uint16_t NodeDiscovery::getUdpSendPort() const {
  return _config.udpSendPort;
}

uint16_t NodeDiscovery::getUdpListenPort() const {
  return _config.udpListenPort;
}

String NodeDiscovery::getNodeName() const {
  return _nodeName;
}

bool NodeDiscovery::hasFireInterval() const {
  return _hasFireInterval;
}

uint32_t NodeDiscovery::getFireInterval(uint32_t fallback) const {
  if (_hasFireInterval) {
    return _fireInterval;
  }

  if (_config.defaultFireInterval > 0) {
    return _config.defaultFireInterval;
  }

  return fallback;
}

void NodeDiscovery::clearMaster() {
  _hasMaster = false;
  _cleanRequested = false;

  _masterIP = IPAddress(0, 0, 0, 0);
  _masterTcpPort = _config.defaultTcpPort;

  _nodeName = _config.defaultNodeName;

  _hasFireInterval = false;
  _fireInterval = _config.defaultFireInterval;

  _beginMs = millis();
  _lastHelloMs = 0;

  Serial.println("[DISCOVERY] Maestro limpiado. Reiniciando busqueda UDP JSON.");
}

bool NodeDiscovery::cleanRequested() const {
  return _cleanRequested;
}

bool NodeDiscovery::consumeCleanRequest() {
  if (!_cleanRequested) {
    return false;
  }

  _cleanRequested = false;
  return true;
}


void NodeDiscovery::normalizeConfig() {
  if (_config.udpSendPort == 0) {
    _config.udpSendPort = 7010;
  }

  if (_config.udpListenPort == 0) {
    if (_config.udpSendPort < 65535) {
      _config.udpListenPort = _config.udpSendPort + 1;
    } else {
      _config.udpListenPort = _config.udpSendPort;
    }
  }

  if (_config.helloIntervalMs == 0) {
    _config.helloIntervalMs = 5000;
  }

  if (_config.discoveryTimeoutMs == 0) {
    _config.discoveryTimeoutMs = 30000;
  }

  if (_config.defaultTcpPort == 0) {
    _config.defaultTcpPort = 7000;
  }
}

IPAddress NodeDiscovery::computeBroadcastIP() {
  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();

  IPAddress broadcast;

  for (uint8_t i = 0; i < 4; i++) {
    broadcast[i] = ip[i] | (uint8_t)(~mask[i]);
  }

  return broadcast;
}

void NodeDiscovery::sendHello() {
  StaticJsonDocument<256> doc;
  //Rutina importante, sincronizar con nombres en teléfono
  doc["type"] = _config.helloType;
  doc["protocol"] = _config.protocol;
  doc["name"] = _config.deviceName;
  doc["user"] = _config.nodeUser;

  doc["ip"] = WiFi.localIP().toString();

  doc["udp_send"] = _config.udpSendPort;
  doc["udp_listen"] = _config.udpListenPort;

  size_t needed = measureJson(doc) + 1;

  if (needed > sizeof(_txBuffer)) {
    Serial.println("[DISCOVERY] Error: JSON HELLO demasiado grande para txBuffer.");
    return;
  }

  size_t len = serializeJson(doc, _txBuffer, sizeof(_txBuffer));

  if (len == 0) {
    Serial.println("[DISCOVERY] Error serializando JSON HELLO.");
    return;
  }

  _udp.beginPacket(_broadcastIP, _config.udpSendPort);
  _udp.write((const uint8_t*)_txBuffer, len);
  _udp.endPacket();

  Serial.print("[DISCOVERY] Broadcast JSON enviado a ");
  Serial.print(_broadcastIP);
  Serial.print(":");
  Serial.print(_config.udpSendPort);
  Serial.print(" -> ");
  Serial.println(_txBuffer);
}

void NodeDiscovery::readUdpPacket() {
  int packetSize = _udp.parsePacket();

  if (packetSize <= 0) return;

  int len = _udp.read(_rxBuffer, sizeof(_rxBuffer) - 1);

  if (len <= 0) return;

  _rxBuffer[len] = '\0';

  Serial.print("[DISCOVERY] UDP JSON recibido en puerto ");
  Serial.print(_config.udpListenPort);
  Serial.print(" desde ");
  Serial.print(_udp.remoteIP());
  Serial.print(":");
  Serial.print(_udp.remotePort());
  Serial.print(" -> ");
  Serial.println(_rxBuffer);

  if (parseCleanJson(_rxBuffer)) {
    Serial.println("[DISCOVERY] Comando UDP clean recibido.");
    _cleanRequested = true;
    return;
  }

  IPAddress parsedIP;
  uint16_t parsedPort = _config.defaultTcpPort;

  String parsedNodeName = _config.defaultNodeName;

  bool parsedHasFireInterval = false;
  uint32_t parsedFireInterval = _config.defaultFireInterval;

  if (
    parseMasterJson(
      _rxBuffer,
      parsedIP,
      parsedPort,
      parsedNodeName,
      parsedHasFireInterval,
      parsedFireInterval
    )
  ) {
    _masterIP = parsedIP;
    _masterTcpPort = parsedPort;

    _nodeName = parsedNodeName;

    _hasFireInterval = parsedHasFireInterval;
    _fireInterval = parsedFireInterval;

    _hasMaster = true;

    Serial.println("[DISCOVERY] Maestro registrado correctamente desde JSON.");

    Serial.print("[DISCOVERY] Master IP: ");
    Serial.println(_masterIP);

    Serial.print("[DISCOVERY] Master TCP Port: ");
    Serial.println(_masterTcpPort);

    Serial.print("[DISCOVERY] nodeName: ");
    Serial.println(_nodeName);

    if (_hasFireInterval) {
      Serial.print("[DISCOVERY] fireInterval: ");
      Serial.println(_fireInterval);
    } else {
      Serial.println("[DISCOVERY] fireInterval no recibido. Parametro opcional.");
    }
  }
}

bool NodeDiscovery::parseCleanJson(const char* jsonText) {
  StaticJsonDocument<128> doc;

  DeserializationError error = deserializeJson(doc, jsonText);

  if (error) {
    return false;
  }

  if (!doc.containsKey("clean")) {
    return false;
  }

  int clean = doc["clean"] | 0;

  return clean == 1;
}

bool NodeDiscovery::parseMasterJson(
  const char* jsonText,
  IPAddress& outIP,
  uint16_t& outPort,
  String& outNodeName,
  bool& outHasFireInterval,
  uint32_t& outFireInterval
) {
  StaticJsonDocument<256> doc;

  DeserializationError error = deserializeJson(doc, jsonText);

  if (error) {
    Serial.print("[DISCOVERY] JSON invalido: ");
    Serial.println(error.c_str());
    return false;
  }

  const char* type = doc["type"] | "";

  bool validAck =
    strcmp(type, "MASTER_ACK") == 0 ||
    strcmp(type, "HALLE_MASTER_ACK") == 0;

  if (!validAck) {
    Serial.print("[DISCOVERY] JSON ignorado. type no valido: ");
    Serial.println(type);
    return false;
  }

  const char* ipText = doc["ip"] | "";

  if (strlen(ipText) == 0) {
    Serial.println("[DISCOVERY] JSON sin campo ip.");
    return false;
  }

  if (!outIP.fromString(ipText)) {
    Serial.print("[DISCOVERY] IP invalida recibida en JSON: ");
    Serial.println(ipText);
    return false;
  }

  uint16_t tcpPort = doc["tcp"] | _config.defaultTcpPort;

  if (tcpPort == 0) {
    Serial.println("[DISCOVERY] Puerto TCP invalido. Usando default.");
    tcpPort = _config.defaultTcpPort;
  }

  outPort = tcpPort;

  const char* nodeNameText = doc["nodeName"] | "";

  // Compatibilidad temporal con el nombre anterior.
  if (strlen(nodeNameText) == 0) {
    nodeNameText = doc["bodyPart"] | "";
  }

  if (strlen(nodeNameText) == 0) {
    outNodeName = _config.defaultNodeName;
    Serial.println("[DISCOVERY] JSON sin nodeName. Usando default.");
  } else {
    outNodeName = String(nodeNameText);
  }

  if (doc.containsKey("fireInterval")) {
    uint32_t interval = doc["fireInterval"] | 0;

    if (interval > 0) {
      outHasFireInterval = true;
      outFireInterval = interval;
    } else {
      outHasFireInterval = false;
      outFireInterval = _config.defaultFireInterval;
    }
  } else {
    outHasFireInterval = false;
    outFireInterval = _config.defaultFireInterval;
  }

  return true;
}