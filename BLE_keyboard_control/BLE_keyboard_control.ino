#include <BleKeyboard.h>

BleKeyboard bleKeyboard("Control C3", "Espressif", 100);

const int buttonPin = 1; 
bool ultimoEstadoFisico = HIGH;
bool estadoLogicoPresionado = false;
unsigned long ultimoTiempoCambio = 0;
const unsigned long tiempoDebounce = 50; // 50ms para ignorar ruidos

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  bleKeyboard.begin();
}

void loop() {
  if(bleKeyboard.isConnected()) {
    int lecturaActual = digitalRead(buttonPin);

    // Si el botón cambió de estado (por ruido o presión real)
    if (lecturaActual != ultimoEstadoFisico) {
      ultimoTiempoCambio = millis(); // Reiniciamos el cronómetro
    }

    // Solo si el estado se mantiene estable por más de 50ms
    if ((millis() - ultimoTiempoCambio) > tiempoDebounce) {
      
      // Si el estado estable es diferente al que teníamos guardado
      if (lecturaActual == LOW && !estadoLogicoPresionado) {
        Serial.println("PAUSA (Estable)");
        // Usamos press y release para que el iPhone no lo ignore
        bleKeyboard.write(' ');
        delay(100); 
        bleKeyboard.releaseAll();
        
        estadoLogicoPresionado = true;
      } 
      else if (lecturaActual == HIGH && estadoLogicoPresionado) {
        Serial.println("PLAY (Estable)");
        bleKeyboard.write(' ');
        delay(100);
        bleKeyboard.releaseAll();
        
        estadoLogicoPresionado = false;
      }
    }
    ultimoEstadoFisico = lecturaActual;
  }
}



