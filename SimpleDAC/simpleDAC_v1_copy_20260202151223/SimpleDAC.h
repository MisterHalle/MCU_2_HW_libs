#ifndef SIMPLEDAC_H
#define SIMPLEDAC_H

#include <Arduino.h>
#include "driver/dac_continuous.h"

enum AudioSourceType {
    SOURCE_INTERNAL, // Efectos (RTOS)
    SOURCE_VOICE     // Voz/WiFi (Loop)
};

class SimpleDAC {
  private:
    dac_continuous_handle_t _handle = NULL;
    bool _enabled = false;
    float _masterVolume = 1.0;
    
    // Control de prioridad
    volatile AudioSourceType _activePriority = SOURCE_INTERNAL;
    volatile uint32_t _lastVoiceTime = 0;
    const uint32_t VOICE_TIMEOUT_MS = 300; // Tiempo de reserva para el stream de voz

  public:
    bool begin(uint32_t sample_rate) {
        dac_continuous_config_t cfg = {
            .chan_mask = DAC_CHANNEL_MASK_CH0,
            .desc_num = 16,
            .buf_size = 2048,
            .freq_hz = sample_rate,
            .offset = 0,
            .clk_src = DAC_DIGI_CLK_SRC_APLL, 
            .chan_mode = DAC_CHANNEL_MODE_SIMUL
        };
        return (dac_continuous_new_channels(&cfg, &_handle) == ESP_OK);
    }

    void setMasterVolume(float vol) { _masterVolume = constrain(vol, 0.0, 1.0); }

    void feed(const unsigned char* data, size_t len, AudioSourceType type, float sourceVolume = 1.0) {
        if (!_handle) return;

        uint32_t now = millis();

        // Lógica de Prioridad: La VOZ (llamada desde el loop) manda
        if (type == SOURCE_VOICE) {
            _activePriority = SOURCE_VOICE;
            _lastVoiceTime = now;
        } 
        else if (type == SOURCE_INTERNAL) {
            // Si hubo voz recientemente, descartamos el efecto interno para no ensuciar el stream
            if (_activePriority == SOURCE_VOICE && (now - _lastVoiceTime < VOICE_TIMEOUT_MS)) {
                return; 
            }
            _activePriority = SOURCE_INTERNAL;
        }

        if (!_enabled) {
            dac_continuous_enable(_handle);
            _enabled = true;
        }

        float finalGain = _masterVolume * sourceVolume;
        size_t bytes_written = 0;

        // Si el volumen es máximo, inyectamos directo al DMA para ahorrar ciclos en el loop
        if (finalGain > 0.98) {
            dac_continuous_write(_handle, (uint8_t*)data, len, &bytes_written, portMAX_DELAY);
        } else {
            const size_t chunkSize = 512; // Chunks más pequeños para mayor reactividad en el loop
            uint8_t temp[chunkSize];
            for (size_t i = 0; i < len; i += chunkSize) {
                size_t current = (len - i < chunkSize) ? len - i : chunkSize;
                for (size_t j = 0; j < current; j++) {
                    int32_t s = (int32_t)data[i + j] - 128;
                    temp[j] = (uint8_t)(((s * (int32_t)(finalGain * 256)) >> 8) + 128);
                }
                dac_continuous_write(_handle, temp, current, &bytes_written, portMAX_DELAY);
            }
        }
    }

    void stop() {
        if (_handle && _enabled) {
            vTaskDelay(pdMS_TO_TICKS(50));
            dac_continuous_disable(_handle);
            _enabled = false;
        }
    }
};

#endif
