#ifndef MOTORPASO_H
#define MOTORPASO_H

#include <Arduino.h>

class MotorPaso {
  private:
    int _stepPin;
    int _dirPin;
    long _pasosRestantes;
    float _currentDelay;
    float _targetDelay;
    float _alpha;
    unsigned long _ultimoMicro;
    bool _estadoStep;
    bool _moviendo;

  public:
    // Constructor: define pines de control
    MotorPaso(int stepPin, int dirPin);

    // Inicializa los pines (llamar en setup)
    void begin();

    // Configura y arranca el movimiento (pasos, pasos/seg, alpha, dir)
    void mover(long pasos, int pasosSeg, float alpha, bool dir);

    // Procesa el movimiento (llamar en loop)
    void actualizar();

    // Estado del motor
    bool estaMoviendo();

    // Parada de emergencia
    void stop();
};

// Función global para el pin Enable (Pin 8 en CNC Shield)
void setMotoresGlobalEnable(bool enable);

#endif
