#include "Stepper.h"

#define PIN_EN_GLOBAL 8

MotorPaso::MotorPaso(int stepPin, int dirPin) {
  _stepPin = stepPin; _dirPin = dirPin;
  _posicionActual = 0; _targetPosicion = 0;
  _moviendo = false; _ultimoMicro = 0;
}

void MotorPaso::begin() {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
}

void MotorPaso::irA(long posicionObjetivo, int pasosSeg, float alpha) {
  _targetPosicion = posicionObjetivo;
  _alpha = alpha;
  _targetDelay = 500000.0 / pasosSeg;
  _moviendo = true;
}

void MotorPaso::actualizar() {
  // Si la diferencia es menor a 1 paso, consideramos que llegó.
  if (abs(_targetPosicion - _posicionActual) < 5) {
    if (_moviendo) {
      digitalWrite(_stepPin, LOW); // Apagar pulso
      _moviendo = false;
      _posicionActual = _targetPosicion; // Forzar igualdad exacta
    }
    return;
  }

  bool dir = (_targetPosicion > _posicionActual);
  digitalWrite(_dirPin, dir);

  unsigned long microActual = micros();
  if (microActual - _ultimoMicro >= (unsigned long)_currentDelay) {
    _ultimoMicro = microActual;
    _estadoStep = !_estadoStep;
    digitalWrite(_stepPin, _estadoStep);

    if (!_estadoStep) {
      if (dir) _posicionActual++; else _posicionActual--;
      
      // Frenado suave al acercarse
      long error = abs(_targetPosicion - _posicionActual);
      float objetivoVel = (error < 50) ? (_targetDelay * 3.0) : _targetDelay;
      _currentDelay = _currentDelay + _alpha * (objetivoVel - _currentDelay);
    }
  }
}


void MotorPaso::stop() { _targetPosicion = _posicionActual; _moviendo = false; }

void setMotoresGlobalEnable(bool enable) {
  pinMode(PIN_EN_GLOBAL, OUTPUT);
  digitalWrite(PIN_EN_GLOBAL, enable ? LOW : HIGH);
}
