/*
  HALLE_VS1053_ESP32_SerialPlayer_v1.0.1
  ------------------------------------------------------------
  Corrección principal:
    - VS1053 y microSD usan el MISMO bus SPI hardware.
    - SPI se inicializa primero con los pines personalizados.
    - Se eliminó la mezcla entre SPI por software y SPI hardware.

  PINOUT CONFIRMADO:
    SCK      -> GPIO14
    MISO     -> GPIO19
    MOSI     -> GPIO13
    SD_CS    -> GPIO21
    X_RESET  -> 3V3
    X_DCS    -> GPIO17
    X_CS     -> GPIO16
    DREQ     -> GPIO35

  ARCHIVOS:
    /track001.mp3
    /track002.mp3
    ...
    /track010.mp3

  COMANDOS:
    1 ... 10  Reproducir pista
    p         Pausar / continuar
    s         Detener
    t         Tono de prueba
    l         Listar microSD
    d         Diagnóstico
    h         Ayuda

  Librerías:
    - Adafruit VS1053 Library
    - Adafruit BusIO
    - SD
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>

// ============================================================
// PINES
// ============================================================

static constexpr int8_t PIN_SPI_SCK   = 14;
static constexpr int8_t PIN_SPI_MISO  = 19;
static constexpr int8_t PIN_SPI_MOSI  = 13;

static constexpr int8_t PIN_SD_CS      = 21;
static constexpr int8_t PIN_VS_RST     = -1;  // X_RESET conectado a 3V3
static constexpr int8_t PIN_VS_DCS     = 17;  // X_DCS
static constexpr int8_t PIN_VS_CS      = 16;  // X_CS
static constexpr int8_t PIN_VS_DREQ    = 35;

// ============================================================
// VS1053 USANDO SPI HARDWARE
// ============================================================
//
// Este constructor NO recibe MOSI/MISO/SCK.
// Usa el objeto global SPI, que inicializamos manualmente
// con SPI.begin(14, 19, 13, -1).
//
Adafruit_VS1053_FilePlayer musicPlayer(
  PIN_VS_RST,
  PIN_VS_CS,
  PIN_VS_DCS,
  PIN_VS_DREQ,
  PIN_SD_CS
);

// ============================================================
// CONFIGURACIÓN
// ============================================================

static constexpr uint8_t VOLUME_LEFT  = 20;
static constexpr uint8_t VOLUME_RIGHT = 20;

// Frecuencia inicial conservadora para validar la microSD.
static constexpr uint32_t SD_SPI_FREQUENCY = 1000000;

String serialBuffer;

bool codecReady = false;
bool sdReady = false;
bool playbackWasActive = false;

// ============================================================
// PROTOTIPOS
// ============================================================

void printHelp();
void printDiagnostic();
void listRootFiles();
void playTrack(uint8_t trackNumber);
void processCommand(String command);
bool isNumericCommand(const String &text);

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println(F("===================================================="));
  Serial.println(F(" HALLE VS1053 + ESP32 Serial Player v1.0.1"));
  Serial.println(F(" SPI hardware compartido VS1053 + microSD"));
  Serial.println(F("===================================================="));

  // ----------------------------------------------------------
  // Desactivar todos los dispositivos SPI antes de iniciar.
  // ----------------------------------------------------------
  pinMode(PIN_SD_CS, OUTPUT);
  pinMode(PIN_VS_CS, OUTPUT);
  pinMode(PIN_VS_DCS, OUTPUT);
  pinMode(PIN_VS_DREQ, INPUT);

  digitalWrite(PIN_SD_CS, HIGH);
  digitalWrite(PIN_VS_CS, HIGH);
  digitalWrite(PIN_VS_DCS, HIGH);

  // ----------------------------------------------------------
  // Inicializar PRIMERO el bus SPI hardware personalizado.
  // ----------------------------------------------------------
  Serial.println(F("[SPI] Inicializando bus hardware..."));
  Serial.printf(
    "[SPI] SCK=%d | MISO=%d | MOSI=%d\n",
    PIN_SPI_SCK,
    PIN_SPI_MISO,
    PIN_SPI_MOSI
  );

  SPI.begin(
    PIN_SPI_SCK,
    PIN_SPI_MISO,
    PIN_SPI_MOSI,
    -1
  );

  delay(100);

  Serial.print(F("[DREQ] Nivel antes de iniciar VS1053: "));
  Serial.println(digitalRead(PIN_VS_DREQ) ? F("HIGH") : F("LOW"));

  // ----------------------------------------------------------
  // Inicializar VS1053 sobre el SPI hardware ya configurado.
  // ----------------------------------------------------------
  Serial.print(F("[VS1053] Inicializando... "));

  codecReady = musicPlayer.begin();

  if (!codecReady) {
    Serial.println(F("ERROR"));
    Serial.println(F("[VS1053] No se recibió versión válida del codec."));
    Serial.print(F("[DREQ] Nivel después del intento: "));
    Serial.println(digitalRead(PIN_VS_DREQ) ? F("HIGH") : F("LOW"));
  } else {
    Serial.println(F("OK"));

    musicPlayer.setVolume(VOLUME_LEFT, VOLUME_RIGHT);

    Serial.println(F("[VS1053] Registros detectados:"));
    musicPlayer.dumpRegs();

    Serial.println(F("[VS1053] Ejecutando tono de prueba..."));
    musicPlayer.sineTest(0x44, 300);
    Serial.println(F("[VS1053] Tono finalizado."));
  }

  // Volver a dejar desactivado el VS1053 antes de montar SD.
  digitalWrite(PIN_VS_CS, HIGH);
  digitalWrite(PIN_VS_DCS, HIGH);
  digitalWrite(PIN_SD_CS, HIGH);

  delay(50);

  // ----------------------------------------------------------
  // Inicializar microSD sobre el mismo bus SPI hardware.
  // ----------------------------------------------------------
  Serial.print(F("[SD] Inicializando microSD a 1 MHz... "));

  sdReady = SD.begin(
    PIN_SD_CS,
    SPI,
    SD_SPI_FREQUENCY
  );

  if (!sdReady) {
    Serial.println(F("ERROR"));
    Serial.println(F("[SD] La tarjeta no respondió."));
  } else {
    Serial.println(F("OK"));

    const uint8_t cardType = SD.cardType();

    Serial.print(F("[SD] Tipo: "));

    switch (cardType) {
      case CARD_MMC:
        Serial.println(F("MMC"));
        break;

      case CARD_SD:
        Serial.println(F("SDSC"));
        break;

      case CARD_SDHC:
        Serial.println(F("SDHC/SDXC"));
        break;

      case CARD_NONE:
        Serial.println(F("SIN TARJETA"));
        sdReady = false;
        break;

      default:
        Serial.println(F("DESCONOCIDA"));
        break;
    }

    if (sdReady) {
      const uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);

      Serial.print(F("[SD] Capacidad aproximada: "));
      Serial.print(cardSizeMB);
      Serial.println(F(" MB"));

      listRootFiles();
    }
  }

  printDiagnostic();
  printHelp();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  if (codecReady && musicPlayer.playingMusic) {
    musicPlayer.feedBuffer();
  }

  if (
    playbackWasActive &&
    !musicPlayer.playingMusic &&
    !musicPlayer.paused()
  ) {
    Serial.println(F("[AUDIO] Reproducción finalizada."));
    playbackWasActive = false;
  }

  if (codecReady) {
    playbackWasActive =
      musicPlayer.playingMusic ||
      musicPlayer.paused();
  }

  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\n' || incoming == '\r') {
      if (!serialBuffer.isEmpty()) {
        processCommand(serialBuffer);
        serialBuffer.clear();
      }
    } else if (serialBuffer.length() < 40) {
      serialBuffer += incoming;
    }
  }

  delay(1);
}

// ============================================================
// COMANDOS
// ============================================================

void processCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command.isEmpty()) {
    return;
  }

  if (isNumericCommand(command)) {
    const int trackNumber = command.toInt();

    if (trackNumber >= 1 && trackNumber <= 10) {
      playTrack(static_cast<uint8_t>(trackNumber));
    } else {
      Serial.println(F("[CMD] Usa un número entre 1 y 10."));
    }

    return;
  }

  if (command == "p") {
    if (!codecReady) {
      Serial.println(F("[AUDIO] VS1053 no disponible."));
      return;
    }

    if (musicPlayer.paused()) {
      musicPlayer.pausePlaying(false);
      Serial.println(F("[AUDIO] Reproducción continuada."));
    } else if (musicPlayer.playingMusic) {
      musicPlayer.pausePlaying(true);
      Serial.println(F("[AUDIO] Reproducción pausada."));
    } else {
      Serial.println(F("[AUDIO] No hay pista activa."));
    }

    return;
  }

  if (command == "s") {
    if (!codecReady) {
      Serial.println(F("[AUDIO] VS1053 no disponible."));
      return;
    }

    if (musicPlayer.playingMusic || musicPlayer.paused()) {
      musicPlayer.stopPlaying();
      playbackWasActive = false;
      Serial.println(F("[AUDIO] Reproducción detenida."));
    } else {
      Serial.println(F("[AUDIO] No hay pista activa."));
    }

    return;
  }

  if (command == "t") {
    if (!codecReady) {
      Serial.println(F("[VS1053] Codec no disponible."));
      return;
    }

    if (musicPlayer.playingMusic || musicPlayer.paused()) {
      musicPlayer.stopPlaying();
    }

    Serial.println(F("[VS1053] Ejecutando tono de prueba..."));
    musicPlayer.sineTest(0x44, 500);
    Serial.println(F("[VS1053] Tono finalizado."));
    return;
  }

  if (command == "l") {
    listRootFiles();
    return;
  }

  if (command == "d") {
    printDiagnostic();
    return;
  }

  if (command == "h" || command == "help" || command == "?") {
    printHelp();
    return;
  }

  Serial.print(F("[CMD] Comando desconocido: "));
  Serial.println(command);
}

// ============================================================
// REPRODUCIR PISTA
// ============================================================

void playTrack(uint8_t trackNumber) {
  if (!codecReady) {
    Serial.println(F("[AUDIO] VS1053 no inicializado."));
    return;
  }

  if (!sdReady) {
    Serial.println(F("[AUDIO] microSD no inicializada."));
    return;
  }

  char filename[24];

  snprintf(
    filename,
    sizeof(filename),
    "/track%03u.mp3",
    trackNumber
  );

  if (!SD.exists(filename)) {
    Serial.print(F("[AUDIO] Archivo no encontrado: "));
    Serial.println(filename);
    return;
  }

  if (musicPlayer.playingMusic || musicPlayer.paused()) {
    musicPlayer.stopPlaying();
    delay(20);
  }

  Serial.print(F("[AUDIO] Reproduciendo: "));
  Serial.println(filename);

  if (!musicPlayer.startPlayingFile(filename)) {
    Serial.println(F("[AUDIO] No fue posible iniciar el archivo."));
    playbackWasActive = false;
    return;
  }

  playbackWasActive = true;
}

// ============================================================
// LISTAR MICROSD
// ============================================================

void listRootFiles() {
  if (!sdReady) {
    Serial.println(F("[SD] microSD no disponible."));
    return;
  }

  File root = SD.open("/");

  if (!root) {
    Serial.println(F("[SD] No se pudo abrir el directorio raíz."));
    return;
  }

  if (!root.isDirectory()) {
    Serial.println(F("[SD] La raíz no es un directorio."));
    root.close();
    return;
  }

  Serial.println();
  Serial.println(F("[SD] Archivos encontrados:"));

  bool foundAnyFile = false;

  while (true) {
    File entry = root.openNextFile();

    if (!entry) {
      break;
    }

    foundAnyFile = true;

    Serial.print(F("  "));
    Serial.print(entry.isDirectory() ? F("[DIR]  ") : F("[FILE] "));
    Serial.print(entry.name());

    if (!entry.isDirectory()) {
      Serial.print(F(" | "));
      Serial.print(entry.size());
      Serial.print(F(" bytes"));
    }

    Serial.println();

    entry.close();
  }

  if (!foundAnyFile) {
    Serial.println(F("  (microSD vacía)"));
  }

  root.close();

  Serial.println();
}

// ============================================================
// DIAGNÓSTICO
// ============================================================

void printDiagnostic() {
  Serial.println();
  Serial.println(F("--------------- DIAGNÓSTICO ---------------"));

  Serial.print(F("SPI SCK GPIO"));
  Serial.println(PIN_SPI_SCK);

  Serial.print(F("SPI MISO GPIO"));
  Serial.println(PIN_SPI_MISO);

  Serial.print(F("SPI MOSI GPIO"));
  Serial.println(PIN_SPI_MOSI);

  Serial.print(F("VS1053 X_CS GPIO"));
  Serial.println(PIN_VS_CS);

  Serial.print(F("VS1053 X_DCS GPIO"));
  Serial.println(PIN_VS_DCS);

  Serial.print(F("VS1053 DREQ GPIO"));
  Serial.println(PIN_VS_DREQ);

  Serial.print(F("DREQ actual: "));
  Serial.println(digitalRead(PIN_VS_DREQ) ? F("HIGH") : F("LOW"));

  Serial.print(F("VS1053: "));
  Serial.println(codecReady ? F("OK") : F("ERROR"));

  Serial.print(F("microSD: "));
  Serial.println(sdReady ? F("OK") : F("ERROR"));

  Serial.println(F("-------------------------------------------"));
  Serial.println();
}

// ============================================================
// AYUDA
// ============================================================

void printHelp() {
  Serial.println(F("--------------- COMANDOS ----------------"));
  Serial.println(F("1 ... 10 : reproducir track001.mp3 ... track010.mp3"));
  Serial.println(F("p        : pausar / continuar"));
  Serial.println(F("s        : detener"));
  Serial.println(F("t        : tono interno VS1053"));
  Serial.println(F("l        : listar microSD"));
  Serial.println(F("d        : diagnóstico"));
  Serial.println(F("h        : ayuda"));
  Serial.println(F("-----------------------------------------"));
  Serial.println(F("Monitor Serial: 115200 + Nueva línea"));
  Serial.println();
}

// ============================================================
// VALIDAR NÚMERO
// ============================================================

bool isNumericCommand(const String &text) {
  if (text.isEmpty()) {
    return false;
  }

  for (size_t i = 0; i < text.length(); ++i) {
    if (!isDigit(text.charAt(i))) {
      return false;
    }
  }

  return true;
}