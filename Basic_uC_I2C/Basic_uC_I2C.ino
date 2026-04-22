#include <Wire.h>
#include <ArduinoJson.h>

void setup() {
  Serial3.begin(115200);
  Config();
}

void loop() {
  sending();
  delay(2000);
}

void Config() {
  Wire.begin(); // Inicia como maestro en PB6 y PB7
  Serial3.println("STM32 Maestro I2C iniciado...");
}

void sending() {
  JsonDocument doc;
  doc["sensor"] = "STM32";
  doc["data"] = random(10, 500);

  char buffer[32]; 
  serializeJson(doc, buffer);

  Wire.beginTransmission(0x09); // Dirección del Arduino
  Wire.write(buffer);
  
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial3.print("Enviado a Arduino: ");
    serializeJson(doc, Serial3);
    Serial3.println();
  } else {
    Serial3.print("Error de transmisión: ");
    Serial3.println(error);
  }
}
