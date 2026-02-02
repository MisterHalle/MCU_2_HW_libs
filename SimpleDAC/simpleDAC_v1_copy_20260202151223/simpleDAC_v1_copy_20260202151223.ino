#include "audio.h"
#include "SimpleDAC.h"

SimpleDAC dac;

// Tarea para efectos (Core 0) - Se encarga de sonidos repetitivos
void taskEfectosInternos(void *p) {
    while(1) {
        // Reproduce el efecto solo si la Voz no está activa
        dac.feed(audio_data, sizeof(audio_data), SOURCE_INTERNAL, 0.9);
        vTaskDelay(pdMS_TO_TICKS(1)); // Espera entre efectos
    }
}

void setup() {
    Serial.begin(115200);
    dac.begin(16000);
    dac.setMasterVolume(0.8);

    // Iniciamos la tarea de efectos en background
    xTaskCreatePinnedToCore(taskEfectosInternos, "Efectos", 4096, NULL, 1, NULL, 0);
}

void loop() {
    // --- LÓGICA DE VOZ / WIFI (HILO PRINCIPAL) ---
    
    // Simulamos que recibes un paquete de voz por WiFi:
    if (Serial.available() > 0) { // O tu función 'if (WiFiClient.available())'
        // Al llamar con SOURCE_VOICE, esta función tomará el control del DAC
        // bloqueando los efectos que vengan de la taskEfectosInternos
        
        // uint8_t bufferVoz[512];
        // dac.feed(bufferVoz, 512, SOURCE_VOICE, 1.0);
        
        Serial.println("Voz detectada: Prioridad otorgada.");
    }

    // El loop puede hacer otras cosas sin bloquear el audio
    // ...
}
 