#include "Stepper.h"

MotorPaso ejeZ(4, 7, 11);
MotorPaso ejeX(2, 5, 9);

void setup() {
  ejeX.begin();
  ejeZ.begin();
  setMotoresGlobalEnable(true);
  Serial.begin(115200);

  // Calibrar eje Z usando pines 9 (Min) y 10 (Max)
  Serial.println("Calibrando...");
  ejeX.autoCalibrar(1000);
  ejeZ.autoCalibrar(1000);
  Serial.print("Rango detectado: ");
  Serial.println(ejeZ.getLimiteMax());

  ejeZ.setMaxLimit(3000); 
}

static int posX = 0, posZ = 0;
static bool dirX = false, dirZ = false;
static long t_x =  0, t_z = 0;

void loop() {

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

  if(posZ >= ejeZ.getLimiteMax()-10 || posZ <= 0 + 10){
    dirZ = !dirZ;
  }
  if(posX >= ejeX.getLimiteMax()-10 || posX <= 0 + 10){
    dirX = !dirX;
  }

  Serial.print("posX: ");
  Serial.print(posX);
  Serial.print("  posZ: ");
  Serial.println(posZ);


  // Mapear el pot al rango real descubierto
  //long target = map((int)lecturaFiltrada, 0, 1023, 0, ejeZ.getLimiteMax());


  ejeX.irA(posX, 1200, 0.01);
  ejeZ.irA(posZ, 1200, 0.01);

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
