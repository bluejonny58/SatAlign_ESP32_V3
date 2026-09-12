/*
  SatAlign V3 - Hauptsketch
  ------------------------------------------------------------
  Startet Hardware, WLAN, OTA, Webserver, RF-Detector und Runtime.
  Der Sketch verbindet die Module, enthaelt aber moeglichst wenig eigene
  Fachlogik. Neue Projektkommentare verwenden ab jetzt die Versionsnummer V3.
*/

/*
  SatAlign ESP32 - Hauptsketch
  Version: V3
  ---------------------------------------------------------------------------
  Einstiegspunkt des ESP32-Projekts.

  setup(): initialisiert Settings, Serial, Sensoren, TFT, WLAN, Web-UI und OTA.
  loop(): aktualisiert RF, Runtime, Display, Webserver, OTA und Heartbeat.

  Die eigentliche Bedien- und AUTO-Logik liegt bewusst in live_runtime.cpp.
*/
#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "ota_manager.h"
#include "settings.h"
#include "rf_detector.h"
#include "display_ui.h"
#include "pins.h"
#include "sensor_mpu6050.h"
#include "control_state.h"
#include "azimuth_control.h"
#include "elevation_control.h"
#include "live_runtime.h"

unsigned long lastHeartbeat = 0;

// V3.1.4 Diagnosebuild: kompakte RF-Messzeile fuer die reale Kalibrierung.
// Ausgabeintervall bewusst 500 ms, damit Messreihen im seriellen Monitor
// gut nachvollziehbar sind, ohne die Steuerlogik zu veraendern.
static unsigned long lastRfCalibrationDiag = 0;
static const unsigned long RF_CAL_DIAG_INTERVAL_MS = 500;

static float bestSignalNorm = 0.0f;
static char signalTextBuffer[32] = { 0 };

// V3.1.4: Eine einzige RF-Prozentbewertung im gesamten Projekt.
// Anzeige und AUTO-Logik verwenden beide rfGetSignalNorm()/rfGetSignalPercent().
// Dadurch kann die Anzeige nicht mehr einen anderen Prozentwert zeigen als
// die Kandidatenerkennung intern bewertet.
static const char* signalTextFromNorm(float norm) {
  const float percent = norm * 100.0f;

  if (percent >= 75.0f) return "sehr gut";
  if (percent >= 50.0f) return "gut";
  if (percent >= 25.0f) return "schwach";
  return "schlecht";
}

static UiMode currentUiMode() {
  if (!liveMpuReady()) {
    return UI_MODE_WARN;
  }

  if (liveAutoFailed()) {
    return UI_MODE_WARN;
  }

  if (controlMode == CONTROL_MAIN_MENU) {
    return UI_MODE_MENU;
  }

  if (controlMode == CONTROL_CENTER) {
    return UI_MODE_CENTER_ALIGN;
  }

  if (controlMode == CONTROL_AUTO) {
    if (liveAutoHasPeak()) {
      return UI_MODE_AUTO;
    }

    return UI_MODE_SEARCH;
  }

  if (azimuthIsPulseActive() || elevationIsPulseActive()) {
    return UI_MODE_SEARCH;
  }

  return UI_MODE_MANUAL;
}

static const char* currentElevationText() {
  return liveGetElevationStateText();
}

static const char* currentAzimuthText() {
  switch (azimuthGetDirection()) {
    case AZ_DIR_EAST:
      return "EAST >>";

    case AZ_DIR_WEST:
      return "WEST <<";

    default:
      return "STOP";
  }
}

static const char* currentAzimuthState() {
  return liveGetAzimuthStateText();
}

static const char* currentInfoText() {
  if (controlMode == CONTROL_MAIN_MENU || controlMode == CONTROL_CENTER) {
    return liveGetInfoText();
  }

  if (controlMode == CONTROL_AUTO) {
    return liveGetInfoText();
  }

  const char* liveInfo = liveGetInfoText();

  if (
    strncmp(liveInfo, "AZWARN|", 7) == 0 ||
    strncmp(liveInfo, "AUTO_SETUP|", 11) == 0
  ) {
    return liveInfo;
  }

  if (!rfIsReady()) {
    return "INFO: RF nicht bereit";
  }

  if (!rfHasValidSignalDrop()) {
    return "RF: Receiver/Signalweg pruefen";
  }

  return liveInfo;
}

static void updateDisplayFromLiveData() {
  DisplayData ui;

  const float signalVolts = rfGetPinVoltage();
  // V3.1.4: exakt dieselbe normierte RF-Bewertung wie in der AUTO-Logik.
  const float signalNorm = rfGetSignalNorm();

  if (signalNorm > bestSignalNorm) {
    bestSignalNorm = signalNorm;
  }

  ui.mode = currentUiMode();

  ui.signalVolts = signalVolts;
  ui.signalNorm = signalNorm;
  ui.signalBestNorm = bestSignalNorm;
  ui.signalText = signalTextFromNorm(signalNorm);

  ui.elevationDeg =
    liveMpuReady()
      ? liveGetRelativeAngleDeg()
      : 0.0f;

  ui.elevationText = currentElevationText();

  ui.azimuthText = currentAzimuthText();
  ui.azimuthState = currentAzimuthState();

  ui.infoText = currentInfoText();

  displayRender(ui);
}

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);

#if DEBUG_BOOT
  Serial.println();
  Serial.println("==================================");
  Serial.print("Selfsat Autotracker - V");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("==================================");
#endif

  bool loadedFromFlash = initSettings();

#if DEBUG_BOOT
  if (loadedFromFlash) {
    Serial.println(
      "Settings aus Flash geladen. "
      "WARNUNG: In No-NVS-Version unerwartet."
    );
  } else {
    Serial.println(
      "Settings aus Code-Defaults gesetzt. "
      "Preferences/NVS wird nicht benutzt."
    );
  }
#endif

#if DEBUG_VERBOSE
  printSettingsToSerial();
#endif

  // ---------------------------------------------------
  // TFT
  // ---------------------------------------------------
  displayInit();

  displayShowStartupStatus(
    "Display bereit",
    "System initialisiert"
  );

  // ---------------------------------------------------
  // Runtime, Taster, Motoren und MPU6050
  // ---------------------------------------------------
  initLiveRuntime();

  const bool mpuBootOk = liveMpuReady();

  displayShowMpuBootTestResult(
    mpuBootOk,
    liveGetRelativeAngleDeg()
  );

  if (!mpuBootOk) {
    azimuthStop();
    elevationStop();

#if DEBUG_BOOT
    Serial.println(
      "BOOT ABGEBROCHEN: MPU6050/GY-521 fehlt "
      "oder konnte nicht initialisiert werden."
    );

    Serial.println(
      "Anlage stromlos machen, Sensor/Kabel "
      "pruefen und neu starten."
    );
#endif

    displayShowMpuFatalBootError();

    while (true) {
      delay(1000);
    }
  }

  // ---------------------------------------------------
  // Bekannter Hinweis zur groben Südausrichtung
  // ---------------------------------------------------
  // Während dieses Startbildschirms werden noch keine
  // Tastenfunktionen angezeigt.
  displayShowSouthAlignPrompt();

  // ---------------------------------------------------
  // WLAN
  // ---------------------------------------------------
  displayShowStartupStatus(
    "WLAN",
    "Verbindung wird aufgebaut"
  );

  wifiInit();

  // ---------------------------------------------------
  // OTA
  // ---------------------------------------------------
  displayShowStartupStatus(
    "OTA",
    "Dienst wird gestartet"
  );

  otaInit();

  // ---------------------------------------------------
  // Webserver
  // ---------------------------------------------------
  displayShowStartupStatus(
    "Webserver",
    "Dienst wird gestartet"
  );

  webServerInit();

  // ---------------------------------------------------
  // RF-Sensor
  // ---------------------------------------------------
  displayShowStartupStatus(
    "RF-Sensor",
    "Signalweg wird gestartet"
  );

  initRFDetector();

  // ---------------------------------------------------
  // Alle Initialisierungen abgeschlossen
  // ---------------------------------------------------
  // Erst jetzt wird dem Benutzer signalisiert, dass das
  // System Eingaben verarbeitet.
  displayShowSystemReady();

  // Danach wird das normale Hauptmenü mit seinen
  // Tastenhinweisen eingeblendet.
  liveCommandOpenMainMenu();
  updateDisplayFromLiveData();

#if DEBUG_BOOT
  Serial.println("RF-Test: Bitte Sat-Receiver einschalten.");

  Serial.println(
    "RF-Anzeige nutzt RF-Spannung; "
    "AUTO bewertet Signal dynamisch."
  );

  Serial.println("Live-Setup abgeschlossen.");
#endif
}

void loop() {
  wifiLoop();
  otaLoop();
  webServerLoop();

  rfUpdate();

  runLiveRuntime();

  updateDisplayFromLiveData();

  // ---------------------------------------------------
  // RF-Kalibrierdiagnose V3.1.4
  // ---------------------------------------------------
  // Diese Ausgabe verwendet exakt dieselben Mess- und Prozentwerte wie
  // Display und AUTO-Logik. Dadurch koennen reale ADC-Grenzwerte spaeter
  // ohne Schaetzung aus den Aussentests abgeleitet werden.
  // Im AUTO-Betrieb liefert serviceAutoSearchLog() bereits den kompakten,
  // strukturierten Messdatenstrom. Die normale RF_DIAG-Ausgabe wird dort
  // bewusst unterdrueckt, damit der kopierbare AUTO-Block nicht mit
  // redundanten Diagnosezeilen vermischt wird.
  if (!isAutoMode() && millis() - lastRfCalibrationDiag >= RF_CAL_DIAG_INTERVAL_MS) {
    lastRfCalibrationDiag = millis();

    Serial.print("RF_DIAG | RAW=");
    Serial.print(rfGetRawAdc());

    Serial.print(" | FILTER=");
    Serial.print(rfGetFilteredAdc(), 1);

    Serial.print(" | V=");
    Serial.print(rfGetPinVoltage(), 3);

    Serial.print(" | SIGNAL=");
    Serial.print(rfGetSignalPercent(), 1);
    Serial.print("%");

    Serial.print(" | DROP=");
    Serial.print(rfGetDropAdc(), 1);

    Serial.print(" | AUTO=");
    Serial.print(liveGetAutoStateText());

    Serial.print(" | MODE=");
    Serial.println(liveGetModeText());
  }

#if DEBUG_STATUS
  if (millis() - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = millis();

    Serial.print("STATUS | ");

    Serial.print(liveGetModeText());

    Serial.print(" | AUTO=");
    Serial.print(liveGetAutoStateText());

    Serial.print(" | RF=");
    Serial.print(rfGetSignalPercent(), 0);

    Serial.print("% | RAW=");
    Serial.print(rfGetRawAdc());

    Serial.print(" | DROP_ADC=");
    Serial.print(rfGetDropAdc(), 0);

    Serial.print(" | EL=");
    Serial.print(liveGetRelativeAngleDeg(), 1);

    Serial.println(" deg");
  }
#endif
}