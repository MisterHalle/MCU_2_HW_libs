/*
  NODE_DISCOVERY_v1.4.0
  Archivo: NodeDiscovery.h

  Cambios:
  - bodyPart pasa a ser nodeName.
  - fireInterval es opcional.
  - Se mantiene compatibilidad temporal con bodyPart.
  - NodeDiscovery queda como modulo generico de discovery UDP.
*/

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

struct NodeDiscoveryConfig {
  // Nombre local o descripcion del dispositivo que se anuncia.
  // Ejemplo: "MARCADOR", "NFC_READER", "IMU_NODE", "SENSOR_NODE".
  const char* deviceName = "";

  // Protocolo o familia del sistema.
  // Ejemplo: "MARCADOR", "ZATA_NODE_DISCOVERY_V1", etc.
  const char* protocol = "NODE_DISCOVERY_V1";

  // ID logico local. En tu caso actual puede ser AP_NUMBER.
  int nodeUser = 0;

  // Tipo de mensaje HELLO enviado por el nodo.
  // Se deja compatible con tu flujo actual.
  const char* helloType = "NODE_HELLO";

  // Nombre por defecto si el maestro no envia nodeName.
  const char* defaultNodeName = "";

  // fireInterval por defecto.
  // Si queda en 0, significa "no configurado".
  uint32_t defaultFireInterval = 0;

  uint16_t udpSendPort = 7010;
  uint16_t udpListenPort = 0;

  uint16_t defaultTcpPort = 7000;

  uint32_t helloIntervalMs = 5000;
  uint32_t discoveryTimeoutMs = 30000;
};

class NodeDiscovery {
public:
  NodeDiscovery();

  bool begin(const NodeDiscoveryConfig& config);
  void update();

  bool hasMaster() const;

  IPAddress getMasterIP() const;
  uint16_t getMasterTcpPort() const;

  uint16_t getUdpSendPort() const;
  uint16_t getUdpListenPort() const;

  String getNodeName() const;

  bool hasFireInterval() const;
  uint32_t getFireInterval(uint32_t fallback = 0) const;

  void clearMaster();

  // Alias temporal para no romper codigo viejo.
  // Idealmente despues se elimina.
  String getbodyPart() const {
    return getNodeName();
  }

private:
  NodeDiscoveryConfig _config;
  WiFiUDP _udp;

  bool _udpReady = false;
  bool _hasMaster = false;

  IPAddress _broadcastIP;
  IPAddress _masterIP;

  uint16_t _masterTcpPort = 0;

  String _nodeName = "";

  bool _hasFireInterval = false;
  uint32_t _fireInterval = 0;

  uint32_t _lastHelloMs = 0;
  uint32_t _beginMs = 0;

  char _rxBuffer[256];
  char _txBuffer[256];

  void normalizeConfig();

  IPAddress computeBroadcastIP();

  void sendHello();
  void readUdpPacket();

  bool parseMasterJson(
    const char* jsonText,
    IPAddress& outIP,
    uint16_t& outPort,
    String& outNodeName,
    bool& outHasFireInterval,
    uint32_t& outFireInterval
  );
};