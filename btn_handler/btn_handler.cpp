# Copyright 2026 Hall-e SpA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     www.apache.org
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


#include "btn_handler.h"

ButtonHandler::ButtonHandler(uint8_t pin, unsigned long long_p_ms, unsigned long window_ms) {
    _pin = pin;
    _long_press_time = long_p_ms;
    _click_window = window_ms;
    
    _isr_triggered = false;
    _is_pressing = false;
    _long_press_executed = false;
    _click_count = 0;
}

void ButtonHandler::begin() {
    pinMode(_pin, INPUT_PULLUP);
}

void IRAM_ATTR ButtonHandler::isr() {
    _isr_triggered = true;
}

void ButtonHandler::handle() {
    unsigned long now = millis();
    bool btn_state = digitalRead(_pin);

    // 1. Detección de cambio de estado vía ISR
    if (_isr_triggered) {
        if (btn_state == LOW && !_is_pressing) {
            _is_pressing = true;
            _btn_press_start = now;
            _long_press_executed = false;
        }
        _isr_triggered = false;
    }

    // 2. Lógica de pulsación larga (mientras se mantiene presionado)
    if (_is_pressing && btn_state == LOW) {
        if (!_long_press_executed && (now - _btn_press_start > _long_press_time)) {
            _long_press_executed = true; // Marcar como ejecutado para este ciclo
            if (_onLongPress) _onLongPress();
        }
    }

    // 3. Detección al soltar el botón
    if (_is_pressing && btn_state == HIGH) {
        _is_pressing = false;
        _last_release_time = now;

        // Solo sumamos clic si NO fue una pulsación larga
        if (!_long_press_executed) {
            _click_count++;
        }
    }

    // 4. Procesar el conteo de clics (cuando pasa el tiempo de espera)
    if (_click_count > 0 && (now - _last_release_time > _click_window) && !_is_pressing) {
        if (_click_count == 1 && _onSingleClick) _onSingleClick();
        else if (_click_count == 2 && _onDoubleClick) _onDoubleClick();
        else if (_click_count == 3 && _onTripleClick) _onTripleClick();
        
        _click_count = 0; // Reset para el siguiente gesto
    }
}