#ifndef MOTORPASO_H
#define MOTORPASO_H

#include <Arduino.h>

class MotorPaso {
  private:
    int _stepPin, _dirPin;
    long _posicionActual;  // Posición interna en pasos
    long _targetPosicion;  // A dónde queremos ir
    float _currentDelay;
    float _targetDelay;
    float _alpha;
    unsigned long _ultimoMicro;
    bool _estadoStep;
    bool _moviendo;

  public:
    MotorPaso(int stepPin, int dirPin);
    void begin();
    
    // Nueva función: Define el objetivo sin detener el motor
    void irA(long posicionObjetivo, int pasosSeg, float alpha);
    
    void actualizar();
    long getPosicion() { return _posicionActual; }
    void resetPosicion(long pos) { _posicionActual = pos; _targetPosicion = pos; }
    bool estaMoviendo() { return _moviendo; }
    void stop();
};

void setMotoresGlobalEnable(bool enable);

#endif
