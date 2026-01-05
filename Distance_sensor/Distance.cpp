/* Copyright 2026 Hall-e SpA

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     www.apache.org

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License. */

#include "Distance.h"

// Inicialización de variables estáticas
SensorData DistanceManager::sensors[5];
uint8_t DistanceManager::_trigPin = 0;

void DistanceManager::begin(uint8_t trigPin) {
    _trigPin = trigPin;
    pinMode(_trigPin, OUTPUT);
    digitalWrite(_trigPin, LOW);
}

void DistanceManager::triggerBothSensors() {
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
}

void DistanceManager::addSensor(uint8_t index, uint8_t echoPin, float o, void (*isr)()) {
    if (index < 5) {
        sensors[index].echoPin = echoPin;
        sensors[index].newReading = false;
        sensors[index].o = o;           //mientras mas grande o,mas prioriza valores actuales
        sensors[index].dist_f = 0;
        pinMode(echoPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(echoPin), isr, CHANGE);
    }
}

float DistanceManager::getDistance(uint8_t index) {
    if (index < 5 && sensors[index].newReading) {
        // Cálculo de distancia en cm (velocidad sonido 0.0343)
        float f_distance_1 = sensors[index].o*((sensors[index].pulseDuration * 0.0343) / 2.0);
        float f_distance_2 = (1 - sensors[index].o)*sensors[index].dist_f;
        sensors[index].distance = f_distance_1 + f_distance_2;
        sensors[index].newReading = false;
    }
    return sensors[index].distance;
}
