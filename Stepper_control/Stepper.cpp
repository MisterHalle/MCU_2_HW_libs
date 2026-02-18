#include "Stepper.h"

#define PIN_EN_GLOBAL 8

MotorPaso::MotorPaso(int stepPin, int dirPin, int limitPin) {
  _stepPin = stepPin; _dirPin = dirPin; _limitPin = limitPin;
  _posicionActual = 0; _targetPosicion = 0; _limiteMax = 2000;
  _moviendo = false; _ultimoMicro = 0;
  _currentDelay = 1000.0; // Valor inicial seguro
}

void MotorPaso::begin() {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_limitPin, INPUT_PULLUP);
}

void MotorPaso::setMaxLimit(long limite) {
  _limiteMax = limite;
}

void MotorPaso::autoCalibrar(int pasosSeg) {
  //pinMode(_limitPin, INPUT_PULLUP);
  //pinMode(_limitPin, INPUT_PULLUP);
  
  float delayCal = 500000.0 / pasosSeg;
  _currentDelay = delayCal; // Sincronizar delay para evitar vibración inicial

  // 1. BUSCAR EL MÍNIMO (Homing)
  digitalWrite(_dirPin, LOW); // Hacia el switch MIN
  while (digitalRead(_limitPin) == HIGH) {
    digitalWrite(_stepPin, HIGH); delayMicroseconds((int)delayCal);
    digitalWrite(_stepPin, LOW);  delayMicroseconds((int)delayCal);
  }
  
  // Rebote para salir del switch y entrar lento (Precisión)
  digitalWrite(_dirPin, HIGH);
  for(int i=0; i<100; i++) {
    digitalWrite(_stepPin, HIGH); delayMicroseconds((int)delayCal);
    digitalWrite(_stepPin, LOW);  delayMicroseconds((int)delayCal);
  }
  digitalWrite(_dirPin, LOW);
  while (digitalRead(_limitPin) == HIGH) {
    digitalWrite(_stepPin, HIGH); delayMicroseconds((int)delayCal * 2);
    digitalWrite(_stepPin, LOW);  delayMicroseconds((int)delayCal * 2);
  }
  
  _posicionActual = 0;
  _targetPosicion = 0;
  delay(200);

  // 2. BUSCAR EL MÁXIMO Y CONTAR PASOS
  digitalWrite(_dirPin, HIGH);
  long contador = 0;
  
  // Salir del switch para no falsear el conteo
  for(int i=0; i<100; i++) {
    digitalWrite(_stepPin, HIGH); delayMicroseconds((int)delayCal);
    digitalWrite(_stepPin, LOW);  delayMicroseconds((int)delayCal);
    contador++;
  }

  while (digitalRead(_limitPin) == HIGH) {
    digitalWrite(_stepPin, HIGH); delayMicroseconds((int)delayCal);
    digitalWrite(_stepPin, LOW);  delayMicroseconds((int)delayCal);
    contador++;
  }
  
  _limiteMax = contador;
  _posicionActual = contador;
  _targetPosicion = contador;
  _moviendo = false;

  // Separarse un poco del switch final
  this->irA(_limiteMax - 50, pasosSeg/2, 0.05);
  while(this->estaMoviendo()) { this->actualizar(); }
}

void MotorPaso::irA(long posicionObjetivo, int pasosSeg, float alpha) {
  _targetPosicion = posicionObjetivo;
  _alpha = alpha;
  _targetDelay = 500000.0 / pasosSeg;
  _moviendo = true;
}

void MotorPaso::actualizar() {
  if (abs(_targetPosicion - _posicionActual) < 11 || digitalRead(_limitPin)) {
    if (_moviendo) {
      digitalWrite(_stepPin, LOW);
      _moviendo = false;
      _posicionActual = _targetPosicion;
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
