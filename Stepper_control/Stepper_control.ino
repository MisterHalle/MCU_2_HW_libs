#include "Stepper.h"
#include <ArduinoJson.h>


#define BOARD_LED 2

MotorPaso ejeZ(4, 7, 11);
MotorPaso ejeX(2, 5, 10);

//Configuracion colores
enum Roles {
    STM32,
    ESP32,   
    Torreta
};

void setup() {
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

  ejeX.begin();
  ejeZ.begin();
  setMotoresGlobalEnable(true);
  Serial.begin(921600);

  Serial2.begin(921600);


  // Calibrar eje Z usando pines 9 (Min) y 10 (Max)
  Serial.println("Calibrando...");
  //ejeX.autoCalibrar(1000);
  //ejeZ.autoCalibrar(1000);
  //Serial.print("Rango detectado X: ");
  //Serial.println(ejeX.getLimiteMax());
  //Serial.print("Rango detectado Z: ");
  //Serial.println(ejeZ.getLimiteMax());

  //ejeZ.setMaxLimit(3000); 
}

static int posX = 0, posZ = 0;
static bool dirX = false, dirZ = false;
static long t_x =  0, t_z = 0;

void loop() {

  if (Serial2.available()) {
    String tramaCompleta = Serial2.readStringUntil('\n');

    Serial.println(tramaCompleta);

    tramaCompleta.trim();
    if (tramaCompleta.length() > 0) {

      StaticJsonDocument<512> doc;
      DeserializationError err = deserializeJson(doc, tramaCompleta);
      if (!err) {
        int rol = (int)doc["node"]; //roles STM32, ESP32, Torreta
        Serial.print("Rol: ");
        Serial.println(rol);

        //Estructura de mensaje: Json con ROL, CMD, VALORES
        //Rol: 0 STM32, 1 ESP32, 2 torreta
        //cmd: mover, detener, switchCAM, disparar, laser, bocina, granada, baliza, luces,

        switch(rol){
          case STM32:
            if(doc["cmd"] == "laser"){
              //digitalWrite(BOARD_LED, (int)doc["valor"][0]);
              Serial.print("Recibido comando de laser: ");
              Serial.println((int)doc["valor"][0]);
            }
            break;
          case ESP32:
            
            break;
          case Torreta:
            if(doc["cmd"] == "shoot"){
              digitalWrite(BOARD_LED, (int)doc["valor"][0]);
              Serial.print("Recibido comando de disparo:");
              Serial.println((int)doc["valor"][0]);
              delay(1000);
            }
            break;
          default:
            break;
        }
      }
      else {
        Serial.println("Error en trama recibida");
      }
    }
    else {
    Serial.println("Error: Trama vacía.");
    }
  }



  if(millis() - t_z > 100){
    switch(dirZ){
      case false:
        posZ++;
        break;
      case true:
        posZ--;
        break;
    }
    t_z = millis();
  }

  if(millis() - t_x > 100){
    switch(dirX){
      case false:
        posX++;
        break;
      case true:
        posX--;
        break;
    }
    t_x = millis();
  }

  if(posZ >= (ejeZ.getLimiteMax()-10) || posZ <= 10){
    dirZ = !dirZ;
    //Serial.println("Cambio de sentido");
  }

  if(posX >= (ejeX.getLimiteMax()-10) || posX <= 10){
    dirX = !dirX;
    //Serial.println("Cambio de sentido");
  }

  //Serial.print("posX: ");
  //Serial.print(posX);
  //Serial.print("  posZ: ");
  //Serial.println(posZ);


  // Mapear el pot al rango real descubierto
  //long target = map((int)lecturaFiltrada, 0, 1023, 0, ejeZ.getLimiteMax());


  ejeX.irA(posX, 200, 0.01);
  ejeZ.irA(posZ, 200, 0.01);

  /* 
  static long ultimoTarget = 0;
  if (abs(target - ultimoTarget) >= 6) {
    ultimoTarget = target;
    ejeZ.irA(target, 1200, 0.01);
  }
  */

  ejeX.actualizar();
  ejeZ.actualizar();
}
