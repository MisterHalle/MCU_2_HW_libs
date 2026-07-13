/*
  TCP_SERVICE_BASIC_EXAMPLE_v1.0.0

  Ejemplo básico de uso de TcpService:

  - Modo Discovery:
      El nodo se conecta a WiFi.
      Envía UDP broadcast.
      Recibe MASTER_ACK.
      Obtiene IP/puerto TCP.
      Conecta al servicio TCP.

  - Modo Manual:
      El nodo se conecta a WiFi.
      Usa IP/puerto definidos globalmente.
      Conecta directo al servicio TCP.

  Requiere:
  - NodeDiscovery.h / NodeDiscovery.cpp
  - TcpService.h / TcpService.cpp
  - ArduinoJson
*/

#include <WiFi.h>
#include <ArduinoJson.h>

#include "NodeDiscovery.h"
#include "TcpService.h"

// =====================================================
// CONFIGURACION GENERAL
// =====================================================

#define AP_SSID "Zata Tech"
#define AP_PASS "Zata.2026"
#define AP_CH   0

#define FIRST_CONNECTION_TIME 15000

// Cambiar a false para probar modo manual
const bool USE_DISCOVERY_MODE = false;

// =====================================================
// CONFIGURACION DISCOVERY UDP
// =====================================================

#define UDP_DISCOVERY_PORT  7010
#define UDP_LISTEN_PORT     7011

// Puerto base por defecto.
// En discovery, el maestro puede responder otro puerto TCP.
#define TCP_DEFAULT_PORT    7001

// Identificador local del nodo.
// En tus marcadores puede ser AP_NUMBER.
int NODE_USER = 1;

// =====================================================
// CONFIGURACION MANUAL TCP
// =====================================================

IPAddress MANUAL_MASTER_IP(192, 168, 100, 18);
uint16_t MANUAL_TCP_PORT = 7001;

// =====================================================
// CONFIGURACION DE PRUEBA
// =====================================================

const uint32_t TCP_RETRY_INTERVAL_MS = 3000;
const uint32_t TEST_SEND_INTERVAL_MS = 5000;

String NODE_NAME = "UNKNOWN";

uint32_t lastTestSendMs = 0;

// =====================================================
// OBJETO PRINCIPAL TCP
// =====================================================

TcpService tcpService;

// =====================================================
// PROTOTIPOS
// =====================================================

void configTcpService();

void onTcpConnected();
void onTcpDisconnected();
void onTcpJsonReceived(JsonDocument& doc);

void sendTestJson();

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" TCP_SERVICE_BASIC_EXAMPLE_v1.0.0");
  Serial.println("======================================");

  configTcpService();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  tcpService.update();

  if (tcpService.masterReady()) {
    NODE_NAME = tcpService.nodeName();
  }

  if (tcpService.tcpConnected()) {
    uint32_t now = millis();

    if (now - lastTestSendMs >= TEST_SEND_INTERVAL_MS) {
      lastTestSendMs = now;
      sendTestJson();
    }
  }

  delay(1);
}

// =====================================================
// CONFIGURACION TCP SERVICE
// =====================================================

void configTcpService() {
  TcpServiceConfig tcpCfg;

  tcpCfg.ssid = AP_SSID;
  tcpCfg.pass = AP_PASS;

  tcpCfg.wifiChannel = AP_CH;
  tcpCfg.wifiTimeoutMs = FIRST_CONNECTION_TIME;
  tcpCfg.wifiRetryIntervalMs = 5000;

  tcpCfg.wifiPersistent = false;
  tcpCfg.wifiAutoReconnect = true;

  tcpCfg.tcpRetryIntervalMs = TCP_RETRY_INTERVAL_MS;
  tcpCfg.tcpNoDelay = true;

  tcpCfg.debug = true;

  if (USE_DISCOVERY_MODE) {
    Serial.println("[MAIN] Modo TCP: DISCOVERY");

    tcpCfg.mode = TCP_MODE_DISCOVERY;

    tcpCfg.discovery.deviceName = "TEST_NODE";
    tcpCfg.discovery.protocol = "NODE_DISCOVERY_V1";
    tcpCfg.discovery.nodeUser = NODE_USER;

    tcpCfg.discovery.udpSendPort = UDP_DISCOVERY_PORT;
    tcpCfg.discovery.udpListenPort = UDP_LISTEN_PORT;

    tcpCfg.discovery.defaultTcpPort = TCP_DEFAULT_PORT;
    tcpCfg.discovery.defaultNodeName = "UNKNOWN";
    tcpCfg.discovery.defaultFireInterval = 0;

    tcpCfg.discovery.helloIntervalMs = 5000;
    tcpCfg.discovery.discoveryTimeoutMs = 30000;
  } 
  else {
    Serial.println("[MAIN] Modo TCP: MANUAL");

    tcpCfg.mode = TCP_MODE_MANUAL;

    tcpCfg.manualIP = MANUAL_MASTER_IP;
    tcpCfg.manualTcpPort = MANUAL_TCP_PORT;
  }

  tcpService.onTcpConnected(onTcpConnected);
  tcpService.onTcpDisconnected(onTcpDisconnected);
  tcpService.onJsonReceived(onTcpJsonReceived);

  bool ok = tcpService.begin(tcpCfg);

  if (ok) {
    Serial.println("[MAIN] TcpService iniciado correctamente.");
  } else {
    Serial.println("[MAIN] Error iniciando TcpService.");
  }
}

// =====================================================
// CALLBACK: TCP CONECTADO
// =====================================================

void onTcpConnected() {
  Serial.println("[MAIN] Callback TCP conectado.");

  NODE_NAME = tcpService.nodeName();

  Serial.print("[MAIN] IP remota: ");
  Serial.println(tcpService.remoteIP());

  Serial.print("[MAIN] Puerto remoto: ");
  Serial.println(tcpService.remotePort());

  Serial.print("[MAIN] NODE_NAME: ");
  Serial.println(NODE_NAME);

  if (tcpService.hasFireInterval()) {
    Serial.print("[MAIN] fireInterval recibido: ");
    Serial.println(tcpService.fireInterval());
  } else {
    Serial.println("[MAIN] fireInterval no recibido. Parametro opcional.");
  }

  StaticJsonDocument<128> doc;

  doc["type"] = "NODE_READY";
  doc["nodeName"] = NODE_NAME;
  doc["nodeUser"] = NODE_USER;

  tcpService.sendJson(doc);
}

// =====================================================
// CALLBACK: TCP DESCONECTADO
// =====================================================

void onTcpDisconnected() {
  Serial.println("[MAIN] Callback TCP desconectado.");
}

// =====================================================
// CALLBACK: JSON RECIBIDO DESDE MAESTRO
// =====================================================

void onTcpJsonReceived(JsonDocument& doc) {
  Serial.print("[MAIN] JSON recibido desde maestro: ");
  serializeJson(doc, Serial);
  Serial.println();

  const char* type = doc["type"] | "";

  if (strcmp(type, "PING") == 0) {
    StaticJsonDocument<96> pong;

    pong["type"] = "PONG";
    pong["nodeName"] = NODE_NAME;
    pong["ms"] = millis();

    tcpService.sendJson(pong);
  }
}

// =====================================================
// ENVIO JSON DE PRUEBA
// =====================================================

void sendTestJson() {
  StaticJsonDocument<128> doc;

  doc["type"] = "TEST";
  doc["nodeName"] = NODE_NAME;
  doc["nodeUser"] = NODE_USER;
  doc["uptime"] = millis();
  doc["OK"] = true;

  tcpService.sendJson(doc);
}