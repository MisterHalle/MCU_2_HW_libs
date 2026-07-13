
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


#ifndef BTN_HANDLER_H
#define BTN_HANDLER_H

#include <Arduino.h>

// Tipo de función para los callbacks (funciones que ejecutará la librería)
typedef void (*ButtonCallback)();

class ButtonHandler {
private:
    uint8_t _pin;
    unsigned long _long_press_time;
    unsigned long _click_window;
    
    volatile bool _isr_triggered;
    bool _is_pressing;
    bool _long_press_executed;
    int _click_count;
    unsigned long _btn_press_start;
    unsigned long _last_release_time;

    // Punteros a funciones de usuario
    ButtonCallback _onSingleClick = nullptr;
    ButtonCallback _onDoubleClick = nullptr;
    ButtonCallback _onTripleClick = nullptr;
    ButtonCallback _onLongPress = nullptr;

public:
    // Constructor: Pin, tiempo para long press (ms), ventana entre clics (ms)
    ButtonHandler(uint8_t pin, unsigned long long_p_ms = 3000, unsigned long window_ms = 400);

    void begin();
    void handle(); // Se debe llamar en el loop()
    
    // Configuración de acciones desde el sketch principal
    void setOnSingleClick(ButtonCallback cb) { _onSingleClick = cb; }
    void setOnDoubleClick(ButtonCallback cb) { _onDoubleClick = cb; }
    void setOnTripleClick(ButtonCallback cb) { _onTripleClick = cb; }
    void setOnLongPress(ButtonCallback cb) { _onLongPress = cb; }

    // Método para ser llamado por la ISR
    void IRAM_ATTR isr();

    // Getters útiles
    bool isPressed() { return digitalRead(_pin) == LOW; }
};

#endif