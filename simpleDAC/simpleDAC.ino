#include "audio.h"
#include "SimpleDAC.h"

SimpleDAC dac;
bool isLooping = true; // Control de bucle infinito

void taskEfectos(void *p) {
    while(1) {
        // Ejemplo: Reproducir un tono sinusoidal infinito
        dac.feed(audio_data, sizeof(audio_data), SOURCE_INTERNAL, 0.5, isLooping);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    dac.begin(16000);
    dac.setMasterVolume(0.8);

    xTaskCreatePinnedToCore(taskEfectos, "AudioTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
    // Si envías 's' por Serial, detienes el bucle
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 's') {
            isLooping = false;
            Serial.println("Loop detenido.");
        }
    }
}
