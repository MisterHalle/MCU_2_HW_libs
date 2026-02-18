#ifndef MOTORPASO_H
#define MOTORPASO_H

#include <Arduino.h>

class MotorPaso {
  private:
    int _stepPin, _dirPin, _limitPin;
    long _posicionActual;
    long _targetPosicion;
    long _limiteMax;       // Almacena el rango descubierto
    float _currentDelay;
    float _targetDelay;
    float _alpha;
    unsigned long _ultimoMicro;
    bool _estadoStep;
    bool _moviendo;

  public:
    MotorPaso(int stepPin, int dirPin, int limitPin);
    void begin();

    void setMaxLimit(long limite);
    
    // Función para descubrir el rango entre dos switches
    void autoCalibrar(int pasosSeg);
    
    void irA(long posicionObjetivo, int pasosSeg, float alpha);
    void actualizar();
    
    long getPosicion() { return _posicionActual; }
    long getLimiteMax() { return _limiteMax; } // Útil para el map() del pot
    void resetPosicion(long pos) { _posicionActual = pos; _targetPosicion = pos; }
    bool estaMoviendo() { return _moviendo; }
    void stop();
};

void setMotoresGlobalEnable(bool enable);

#endif
