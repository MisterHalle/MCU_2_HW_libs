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

#ifndef DISTANCE_H
#define DISTANCE_H

#include <Arduino.h>

struct SensorData {
    uint8_t echoPin;
    volatile unsigned long startTime;
    volatile unsigned long pulseDuration;
    volatile bool newReading;
    float o;
    float dist_f;
    float distance;
};

class DistanceManager {
public:
    // Configura el pin de Trigger común para todos los sensores
    static void begin(uint8_t trigPin);
    
    // Dispara el pulso en el pin común
    static void triggerBothSensors();

    // Vincula un pin de ECHO a una estructura de datos y una ISR
    static void addSensor(uint8_t index, uint8_t echoPin, float o, void (*isr)());

    // Calcula y retorna la distancia del sensor indicado
    static float getDistance(uint8_t index);

    // Arreglo global de datos accesible por las ISR
    static SensorData sensors[5];

private:
    static uint8_t _trigPin;
};

#endif
