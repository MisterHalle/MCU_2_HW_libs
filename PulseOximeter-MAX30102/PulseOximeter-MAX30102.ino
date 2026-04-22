#include <Wire.h>
#include "PulseOximeter-MAX30102.h"

PulseOximeter pox;
PulseOximeter::Sample samples[8];

#define SDA_PIN 5 
#define SCL_PIN 4 

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!pox.begin()) {
    Serial.println("MAX30102 no encontrado o fallo de inicializacion");
    while (1);
  }

  pox.setExposureTargets(70000, 180000);
  pox.setLedBackoffMargin(30000);
  pox.setMinLedPulseAmplitude(0x04);

  Serial.print("PART_ID: 0x");
  Serial.println(pox.readPartID(), HEX);

  Serial.print("REV_ID: 0x");
  Serial.println(pox.readRevisionID(), HEX);

  Serial.print("SMP_AVE actual: ");
  Serial.println(pox.getSampleAveraging());

  Serial.println("ts_ms,red,ir,avail,ovf");
}

void loop() {
  uint8_t avail = pox.availableSamples();

  if (avail == 0) {
    delay(10);
    return;
  }

  size_t n = pox.readAvailableSamples(samples, 8);

  // autoajuste sobre el lote recién leído
  bool changed = pox.autoAdjustExposure(samples, n);

  for (size_t i = 0; i < n; i++) {
    Serial.print(samples[i].timestampMs);
    Serial.print(",");
    Serial.print(samples[i].red);
    Serial.print(",");
    Serial.print(samples[i].ir);
    Serial.print(",");
    Serial.print(avail);
    Serial.print(",");
    Serial.print(pox.readOverflowCounter());
    Serial.print(",adc=");
    Serial.print(pox.getAdcRange());
    Serial.print(",redPA=");
    Serial.print(pox.getConfig().redLedPA);
    Serial.print(",irPA=");
    Serial.print(pox.getConfig().irLedPA);
    Serial.print(",adj=");
    Serial.println(changed ? 1 : 0);
  }

  delay(5);
}