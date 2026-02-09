#ifndef SIMPLEDAC_H
#define SIMPLEDAC_H

#include <Arduino.h>
#include "driver/dac_continuous.h"

enum AudioSourceType { SOURCE_INTERNAL, SOURCE_VOICE };

class SimpleDAC {
  private:
    dac_continuous_handle_t _handle = NULL;
    bool _enabled = false;
    float _masterVolume = 1.0;
    
    volatile AudioSourceType _activePriority = SOURCE_INTERNAL;
    volatile uint32_t _lastVoiceTime = 0;
    const uint32_t VOICE_TIMEOUT_MS = 300;

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

    // Función para encontrar el inicio del payload 'data' en un archivo WAV
    uint32_t getWavOffset(const unsigned char* data, size_t len) {
        // Buscamos la firma "data" (0x64 0x61 0x74 0x61)
        for (uint32_t i = 0; i < len - 4; i++) {
            if (data[i] == 'd' && data[i+1] == 'a' && data[i+2] == 't' && data[i+3] == 'a') {
                return i + 8; // Retornamos la posición después de 'data' y el tamaño del chunk
            }
        }
        return 44; // Si no lo encuentra, por defecto el header WAV estándar es de 44 bytes
    }

    void feed(const unsigned char* data, size_t len, AudioSourceType type, float sourceVolume = 1.0, bool looping = false) {
        if (!_handle) return;

        uint32_t now = millis();
        if (type == SOURCE_VOICE) {
            _activePriority = SOURCE_VOICE;
            _lastVoiceTime = now;
        } else if (type == SOURCE_INTERNAL) {
            if (_activePriority == SOURCE_VOICE && (now - _lastVoiceTime < VOICE_TIMEOUT_MS)) return;
            _activePriority = SOURCE_INTERNAL;
        }

        if (!_enabled) {
            dac_continuous_enable(_handle);
            _enabled = true;
        }

        // --- GAPLESS LOOP LOGIC ---
        uint32_t offset = getWavOffset(data, len);
        const unsigned char* payload = data + offset;
        size_t payloadLen = len - offset;
        
        float finalGain = _masterVolume * sourceVolume;
        size_t bytes_written = 0;

        // Ejecutar mientras looping sea true o una sola vez si es false
        do {
            const size_t chunkSize = 512;
            uint8_t temp[chunkSize];

            for (size_t i = 0; i < payloadLen; i += chunkSize) {
                // Verificación de interrupción por Voz (Prioridad)
                if (type == SOURCE_INTERNAL && _activePriority == SOURCE_VOICE) return;


                size_t current = (payloadLen - i < chunkSize) ? payloadLen - i : chunkSize;
                for (size_t j = 0; j < current; j++) {
                    int32_t s = (int32_t)payload[i + j] - 128;
                    temp[j] = (uint8_t)(((s * (int32_t)(finalGain * 256)) >> 8) + 128);
                }
                dac_continuous_write(_handle, temp, current, &bytes_written, portMAX_DELAY);
            }
            
            // Si no estamos en modo loop, salimos del 'do-while'
            if (!looping) break;
            
            // Verificación de seguridad para no quedar atrapado si looping cambia externamente
            yield(); 

        } while (looping);
    }
};

#endif
