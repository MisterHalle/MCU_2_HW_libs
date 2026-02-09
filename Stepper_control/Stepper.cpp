#include "Stepper.h"

// Pin fijo para la CNC Shield V3
#define PIN_EN_GLOBAL 8

MotorPaso::MotorPaso(int stepPin, int dirPin) {
  _stepPin = stepPin;
  _dirPin = dirPin;
  _moviendo = false;
  _pasosRestantes = 0;
  _ultimoMicro = 0;
  _estadoStep = false;
}

void MotorPaso::begin() {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
}

void MotorPaso::mover(long pasos, int pasosSeg, float alpha, bool dir) {
  if (pasos <= 0) return;
  
  digitalWrite(_dirPin, dir);
  _pasosRestantes = pasos;
  _alpha = alpha;
  
  // Cálculo: 1,000,000 micros / (pasosSeg * 2 estados por paso)
  _targetDelay = 500000.0 / pasosSeg;
  _currentDelay = _targetDelay * 2.5; // Factor de suavizado inicial
  _moviendo = true;
}

void MotorPaso::actualizar() {
  if (!_moviendo || _pasosRestantes <= 0) {
    _moviendo = false;
    return;
  }

  unsigned long microActual = micros();
  
  if (microActual - _ultimoMicro >= (unsigned long)_currentDelay) {
    _ultimoMicro = microActual;
    _estadoStep = !_estadoStep;
    digitalWrite(_stepPin, _estadoStep);

    if (!_estadoStep) {
      _pasosRestantes--;
      
      // Dinámica de destino: si quedan pocos pasos, frenar volviendo al delay lento
      float objetivo = (_pasosRestantes < 100) ? (_targetDelay * 2.5) : _targetDelay;
      
      // Aplicación del filtro EMA
      _currentDelay = _currentDelay + _alpha * (objetivo - _currentDelay);
    }
  }
}

bool MotorPaso::estaMoviendo() {
  return _moviendo;
}

void MotorPaso::stop() {
  _moviendo = false;
  _pasosRestantes = 0;
}

// Implementación de la función global
void setMotoresGlobalEnable(bool enable) {
  pinMode(PIN_EN_GLOBAL, OUTPUT);
  digitalWrite(PIN_EN_GLOBAL, enable ? LOW : HIGH);
}
