#include "Stepper.h"

MotorPaso ejeZ(4, 7);
const int pinPot = A5;
const float EscalarA = 1.0; // Ajusta según necesites más o menos recorrido

void setup() {
  ejeZ.begin();
  setMotoresGlobalEnable(true);
  Serial.begin(115200);
}

void loop() {
  // 1. Lectura con promedio simple para eliminar picos de ruido
  static int lecturaFiltrada = 0;
  int lecturaRaw = analogRead(pinPot);
  
  // Filtro básico: la lectura filtrada sigue a la raw lentamente
  lecturaFiltrada = (lecturaFiltrada * 0.95) + (lecturaRaw * 0.05);

  // 2. Aplicar un umbral de movimiento
  static long ultimoTarget = 0;
  long nuevoTarget = (long)(lecturaFiltrada * EscalarA);

  // Si el cambio es mayor a 2 pasos, actualizamos el objetivo del motor
  if (abs(nuevoTarget - ultimoTarget) > 6) {
      ultimoTarget = nuevoTarget;
      ejeZ.irA(nuevoTarget, 1000, 0.01);
  }

  // 3. El motor siempre debe actualizarse
  ejeZ.actualizar();
}
