#include <Wire.h>
#include <ArduinoJson.h>

#define SLAVE_ADDR 0x09

char incomingBuffer[32];
volatile bool newPacket = false;

void setup() {
  Serial.begin(115200);
  Config();
}

void loop() {
  if (newPacket) {
    receiving();
    newPacket = false;
  }
}

void Config() {
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  Serial.println("Arduino Esclavo listo.");
}

void receiveEvent(int howMany) {
  int i = 0;
  while (Wire.available() && i < 31) {
    incomingBuffer[i++] = Wire.read();
  }
  incomingBuffer[i] = '\0';
  newPacket = true;
}

void receiving() {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, incomingBuffer);

  if (!error) {
    const char* sensor = doc["sensor"];
    int data = doc["data"];
    Serial.print("Recibido de ");
    Serial.print(sensor);
    Serial.print(": ");
    Serial.println(data);
  }
}
