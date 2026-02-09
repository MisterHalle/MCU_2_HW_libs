#include "Stepper.h"

#define STEPPER_VEL 2000

// Definición de motores según pines CNC Shield Z e Y
MotorPaso ejeZ(4, 7);
MotorPaso ejeX(2, 5);

void setup() {
  ejeZ.begin();
  ejeX.begin();
  
  setMotoresGlobalEnable(true); // Activa todos los drivers
  
  // Movimiento de prueba
  ejeZ.mover(STEPPER_VEL, 1000, 0.005, true);
  ejeX.mover(STEPPER_VEL, 1000, 0.005, false);
}

void loop() {
  // Ambas funciones corren en paralelo sin bloquearse
  ejeZ.actualizar();
  ejeX.actualizar();
  
  // Podrías leer un sensor aquí y el motor no se detendría
}
