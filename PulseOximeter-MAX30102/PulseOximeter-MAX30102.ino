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
  pox.setLedBackoffMargin(10000);
  pox.setMinLedPulseAmplitude(0x04);
  pox.setFingerThresholds(10000, 5000);
  pox.setSaturationThreshold(250000);
  pox.setUsableStableSamples(24);
  pox.setDCSettleTolerance(3000, 3000);
  pox.enableAutoExposure(true);

  pox.setHeartRateSmoothing(0.40f, 0.18f, 8.0f);
  pox.setHeartRateTimeout(2500);

  pox.setHeartRatePeakDetection(0.30f, 35.0f);
  pox.setHeartRateHoldTimeout(5000);
  pox.setHeartRateJumpLimit(15.0f);
  pox.setHeartRateSmoothing(0.40f, 0.18f, 8.0f);

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

  if (n > 0) {
    // 1) autoajuste sigue vivo
    bool adj = pox.processSamples(samples, n);

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
      Serial.print(adj ? 1 : 0);

      Serial.print(",usable=");
      Serial.print(pox.isSignalUsable() ? 1 : 0);

      Serial.print(",HR=");
      Serial.print(pox.getHeartRateBpm(), 1);

      Serial.print(",R=");
      Serial.print(pox.getRValue(), 4);

      Serial.print(",SpO2=");
      Serial.print(pox.getSpO2(), 1);

      Serial.print(",stableN=");
      Serial.println(pox.getStableSampleCount());
    }
  }

  delay(10);
}