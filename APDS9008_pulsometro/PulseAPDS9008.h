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

#pragma once

#include <Arduino.h>

enum PulseMeasurementState : uint8_t {
  PULSE_STATE_NO_CONTACT = 0,
  PULSE_STATE_ACQUIRING,
  PULSE_STATE_TRACKING,
  PULSE_STATE_MOTION
};

struct PulseAPDS9008Config {
  uint8_t signalPin = 2;
  uint8_t ledPin = 0;

  // 10 ms = 100 Hz.
  uint16_t sampleIntervalMs = 10;

  uint8_t ledPower = 255;
  bool autoLedControl = false;

  // Guardrail fisiologico amplio.
  // No se usa como objetivo ni como prior del BPM.
  uint16_t minBpm = 35;
  uint16_t maxBpm = 220;

  uint16_t warmupMs = 1200;

  // Piso del detector AC.
  float minPulseAmplitude = 12.0f;

  // Filtros.
  float dcAlpha = 0.005f;
  float signalAlpha = 0.25f;
  float envelopeAlpha = 0.035f;
  float noiseAlpha = 0.06f;

  // Threshold dinamico.
  float envelopeThresholdFactor = 0.35f;
  float noiseThresholdFactor = 3.5f;
  float releaseFactor = 0.25f;

  // Validacion morfologica del pico.
  float prominenceFactor = 1.15f;
  float noiseProminenceFactor = 4.0f;
  float peakFallNoiseFactor = 1.25f;
  float minPeakFall = 3.0f;
  uint8_t fallingSamplesToConfirm = 2;
  uint16_t candidateTimeoutMs = 280;

  // --------------------------------------------------------
  // CONTACTO OPTICO
  // --------------------------------------------------------

  // Evita los extremos del ADC.
  uint16_t contactAdcMin = 80;
  uint16_t contactAdcMax = 4015;

  // La presencia de una variacion optica minima debe sostenerse
  // durante un tiempo para declarar contacto probable.
  float contactEnvelopeFactor = 0.35f;

  // Piso absoluto adicional para evitar que ruido ambiente
  // muy pequeño sea interpretado como contacto.
  float contactMinEnvelope = 8.0f;

  // Criterios de limpieza de señal para contacto pasivo.
  //
  // SNR aproximado:
  //   envelope / (noise + 0.5)
  float contactMinSnr = 3.5f;

  // noise / (envelope + 0.5)
  float contactMaxNoiseRatio = 0.30f;

  uint16_t contactConfirmMs = 850;
  uint16_t contactLostMs = 1300;

  // --------------------------------------------------------
  // MOVIMIENTO / ARTEFACTO
  // --------------------------------------------------------

  // Salto RAW minimo considerado sospechoso.
  float motionRawJumpFloor = 90.0f;

  // Componente AC extremadamente grande.
  float motionAcFloor = 260.0f;

  // Ruido relativo a la envolvente.
  float motionNoiseRatio = 0.55f;

  // Dos muestras consecutivas de evidencia antes de bloquear.
  uint8_t motionConfirmSamples = 2;

  // Mantiene el estado MOTION para dejar estabilizar el sensor.
  uint16_t motionHoldMs = 700;

  // --------------------------------------------------------
  // ADQUISICION / TRACKING
  // --------------------------------------------------------

  // Espera luego de obtener contacto o salir de movimiento.
  uint16_t acquisitionSettleMs = 450;

  // Cantidad minima de IBI coherentes para bloquear el ritmo.
  uint8_t acquisitionMinIbi = 4;

  // Coherencia interna durante adquisicion.
  float acquisitionTolerance = 0.22f;

  // Los clusters de frecuencia alta requieren mas evidencia,
  // ya que el doble conteo de la onda PPG puede generar un
  // segundo armonico cercano al doble de BPM.
  uint16_t highRateBpmThreshold = 120;
  uint8_t highRateAcquisitionMinIbi = 5;

  // En tracking se permite una tolerancia algo mayor.
  float trackingIbiTolerance = 0.32f;

  // Refractario dinamico:
  // max(refractario fisiologico, robustIBI * factor)
  float dynamicRefractoryFactor = 0.52f;

  // Rearme morfologico entre pulsos.
  //
  // Tras detectar un pico, no se habilita otro candidato hasta
  // observar un valle AC suficientemente negativo.
  bool beatRearmEnabled = true;
  float beatRearmMinAc = 4.0f;
  float beatRearmNoiseFactor = 1.0f;

  // Si aparecen IBI coherentes pero diferentes a la referencia,
  // el sistema puede adoptar automaticamente el nuevo cluster.
  uint8_t reacquireMinIbi = 3;
  float reacquireTolerance = 0.22f;

  // Muchos outliers incoherentes hacen volver a ACQUIRING.
  uint8_t maxConsecutiveOutliers = 5;

  // Signal/contact hold.
  uint16_t signalHoldMs = 1800;
};

struct PulseAPDS9008Data {
  uint16_t raw = 0;

  float dc = 0.0f;
  float ac = 0.0f;
  float envelope = 0.0f;
  float noise = 0.0f;
  float threshold = 0.0f;

  float peak = 0.0f;
  float prominence = 0.0f;

  // 0..100
  float motionScore = 0.0f;

  // Diagnóstico de contacto pasivo.
  float contactSnr = 0.0f;

  // True cuando la morfología ya atravesó un valle suficiente
  // después del último pico y puede buscar el siguiente.
  bool beatRearmed = true;

  uint16_t bpm = 0;
  uint16_t ibiMs = 0;
  uint16_t robustIbiMs = 0;

  uint32_t beatCount = 0;
  uint32_t validBeatCount = 0;
  uint32_t rejectedPeakCount = 0;
  uint32_t rejectedIbiCount = 0;
  uint32_t artifactCount = 0;
  uint32_t reacquisitionCount = 0;

  uint8_t validIbiCount = 0;

  // Calidad optica/morfologica.
  uint8_t quality = 0;

  // Confianza de la medicion final.
  uint8_t confidence = 0;

  uint8_t ledPower = 255;

  bool beat = false;
  bool ibiAccepted = false;
  bool bpmStable = false;

  bool contactLikely = false;
  bool motionDetected = false;
  bool measurementValid = false;
  bool candidateActive = false;

  PulseMeasurementState state = PULSE_STATE_NO_CONTACT;
};

class PulseAPDS9008 {
public:
  PulseAPDS9008();

  bool begin(const PulseAPDS9008Config& config);
  bool update();

  void reset();

  const PulseAPDS9008Data& data() const;

  bool beatDetected() const;
  uint16_t bpm() const;
  uint16_t ibiMs() const;
  uint32_t beatCount() const;
  uint8_t quality() const;
  uint8_t confidence() const;

  bool contactLikely() const;
  bool motionDetected() const;
  bool measurementValid() const;

  PulseMeasurementState state() const;
  const char* stateName() const;

  void setLedPower(uint8_t power);
  uint8_t ledPower() const;

  void setAutoLedControl(bool enabled);
  bool autoLedControl() const;

  void setMinPulseAmplitude(float amplitude);
  float minPulseAmplitude() const;

private:
  static const uint8_t IBI_BUFFER_SIZE = 7;
  static const uint8_t ACQ_BUFFER_SIZE = 6;

  PulseAPDS9008Config _cfg;
  PulseAPDS9008Data _data;

  bool _started = false;
  bool _signalInitialized = false;

  uint32_t _startMs = 0;
  uint32_t _lastSampleMs = 0;

  uint32_t _stateEnteredMs = 0;

  uint32_t _lastDetectedBeatMs = 0;
  uint32_t _lastAcceptedBeatMs = 0;
  uint32_t _lastSignalEvidenceMs = 0;

  uint32_t _contactEvidenceStartMs = 0;
  uint32_t _motionUntilMs = 0;

  float _filteredAc = 0.0f;
  float _previousFilteredAc = 0.0f;
  float _previousRaw = 0.0f;

  float _clipLevel = 0.0f;
  float _qualityEma = 0.0f;
  float _motionEma = 0.0f;

  uint8_t _motionEvidenceCount = 0;
  uint8_t _consecutiveOutliers = 0;

  // Protección contra doble detección dentro de una misma
  // onda PPG.
  bool _beatRearmed = true;

  // Pico/candidato.
  float _cycleValley = 0.0f;

  bool _candidateActive = false;
  uint32_t _candidateStartMs = 0;

  float _candidateThreshold = 0.0f;
  float _candidateValley = 0.0f;

  float _candidatePeak = 0.0f;
  uint32_t _candidatePeakMs = 0;

  uint8_t _fallingSamples = 0;

  // Historial estable.
  uint16_t _ibiBuffer[IBI_BUFFER_SIZE] = {0};
  uint8_t _ibiIndex = 0;
  uint8_t _ibiCount = 0;

  // Adquisicion/reacquisicion.
  uint16_t _acqBuffer[ACQ_BUFFER_SIZE] = {0};
  uint8_t _acqCount = 0;

  uint16_t _reacqBuffer[ACQ_BUFFER_SIZE] = {0};
  uint8_t _reacqCount = 0;

  void processSample(uint16_t raw, uint32_t nowMs);

  void updateMotion(
    uint16_t raw,
    uint32_t nowMs
  );

  void updateContact(
    uint16_t raw,
    uint32_t nowMs
  );

  void updateState(uint32_t nowMs);

  void enterState(
    PulseMeasurementState newState,
    uint32_t nowMs
  );

  uint32_t currentRefractoryMs() const;

  void startCandidate(
    uint32_t nowMs,
    float threshold
  );

  void updateCandidate(
    uint32_t nowMs,
    float releaseThreshold
  );

  void finalizeCandidate(uint32_t nowMs);
  void cancelCandidate();

  void processBeat(
    uint32_t peakMs,
    float peakAmplitude,
    float prominence
  );

  bool physiologicIbi(
    uint32_t ibi
  ) const;

  bool ibiCloseTo(
    uint32_t ibi,
    uint16_t reference,
    float tolerance
  ) const;

  void processAcquisitionIbi(
    uint16_t ibi,
    uint32_t peakMs
  );

  void processTrackingIbi(
    uint16_t detectedIbi,
    uint32_t peakMs
  );

  bool acquisitionClusterReady(
    const uint16_t* values,
    uint8_t count,
    uint16_t& clusterMedian
  ) const;

  void lockFromAcquisition(
    uint16_t clusterMedian,
    uint32_t peakMs
  );

  void pushStableIbi(uint16_t ibi);
  void pushAcqIbi(uint16_t ibi);
  void pushReacqIbi(uint16_t ibi);

  uint16_t medianIbi() const;
  uint16_t robustAverageIbi() const;

  uint16_t medianOf(
    const uint16_t* values,
    uint8_t count
  ) const;

  void clearStableHistory();
  void clearAcquisition();
  void clearReacquisition();

  void updateQuality(
    uint16_t raw
  );

  void updateConfidence();

  void applyLedPower();

  // Valor absoluto local para evitar depender directamente
  // de <math.h> en proyectos Arduino con posibles colisiones
  // de nombres de headers.
  static float absFloat(float value);

  static float clampFloat(
    float value,
    float minimum,
    float maximum
  );

  static void sortUint16(
    uint16_t* values,
    uint8_t count
  );
};