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

// Zentrale Konfigurationswerte.
// Hier kommen z. B. Baudrate, Heartbeat-Zeit und weitere globale
// Systemparameter her, die über settings.cpp mit Werten versorgt werden.
#include "config.h"

// -----------------------------------------------------
// Netzwerk
// -----------------------------------------------------
// Verantwortlich für WLAN-Verbindung und laufende Reconnect-Versuche.
#include "wifi_manager.h"

// Webserver mit der Browser-Oberfläche für MANUELL/AUTO sowie
// Statusanzeige im Netzwerk.
#include "web_server.h"

// OTA-Updates über WLAN
#include "ota_manager.h"

// -----------------------------------------------------
// Runtime-Einstellungen
// -----------------------------------------------------
// Setzt die zentralen Projektwerte aus settings.cpp.
// No-NVS-Version: Es werden keine Werte aus Preferences/NVS geladen
// und keine Werte in den ESP32-Flash geschrieben.
#include "settings.h"

// -----------------------------------------------------
// Sensoren / Anzeige
// -----------------------------------------------------
// RF-Detektor für die Auswertung des Satellitensignals.
#include "rf_detector.h"

// TFT-Anzeige für Live-Status, Signal, Azimut, Elevation und Infotexte.
#include "display_ui.h"

// Pinbelegung fuer die Boot-EZ-Handbedienung vor dem Hauptmenue.
// Kommentarstand: V3
#include "pins.h"

// Direkter Zugriff auf den MPU-Filter fuer die Boot-Anfahrt auf Standard-EZ.
// Kommentarstand: V3
#include "sensor_mpu6050.h"

// -----------------------------------------------------
// Zustände / Steuerung
// -----------------------------------------------------
// Enthält die globale Modusumschaltung MANUELL / AUTO.
#include "control_state.h"

// Azimut-Achse: EAST/WEST, Hall-Sensoren, Referenzfahrt, Pulsbetrieb.
#include "azimuth_control.h"

// Elevations-Achse: L298N-Ansteuerung, Pulse, Softlimits.
#include "elevation_control.h"

// -----------------------------------------------------
// Live-Laufzeit
// -----------------------------------------------------
// Enthält die eigentliche Bedienlogik:
// - 3-Taster-Bedienung
// - MANUELL / AUTO
// - AUTO-Zustandsmaschine
// - Statusfunktionen für Web und Display
#include "live_runtime.h"

// Letzter Zeitpunkt für die serielle Heartbeat-Ausgabe.
// Dient nur zur zyklischen Diagnose im seriellen Monitor.
unsigned long lastHeartbeat = 0;

// Bester bisher gesehener Signalwert fuer die Anzeige.
// Der Balken basiert bewusst auf der aktuell gemessenen RF-Spannung,
// weil dein Praxistest gezeigt hat:
//   gutes Signal        ca. 0.62 V
//   vom Satelliten weg  ca. 0.80 V
// Beim AD8317/AD8318 bedeutet hier: niedrigere Spannung = staerkeres Signal.
static float bestSignalNorm = 0.0f;

// Kleiner Textpuffer fuer die RF-Anzeige.
// Wichtig: displayRender() bekommt nur einen const char*, deshalb bleibt
// der Puffer global/statisch gueltig.
static char signalTextBuffer[32] = { 0 };

// Hilfsfunktion zum Begrenzen eines normierten Werts auf 0.0 ... 1.0.
// Das verhindert, dass Anzeige- oder Rechenwerte außerhalb des
// erwarteten Bereichs laufen.
static float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

// Berechnet den normierten Signal-Anzeigewert zwischen 0 und 1.
//
// RF-Spannungsanzeige:
// Die vorherige DROP_ADC-Anzeige blieb bei deinem Test dauerhaft voll gruen,
// obwohl die RF-Spannung beim Wegfahren sichtbar von ca. 0.62 V auf ca. 0.80 V
// gestiegen ist. Deshalb bewertet der TFT-Balken jetzt direkt die gemessene
// RF-Spannung am ADC-Pfad.
//
// Schwellen fuer den ersten Praxistest:
//   <= 0.66 V  -> sehr gut / gruen / Balken voll
//   0.66-0.73  -> gut
//   0.73-0.82  -> schwach / gelb-orange
//   >  0.82 V  -> schlecht / rot
static float currentSignalNormFromVoltage(float volts) {
  // Niedrige Spannung = gutes Signal.
  if (volts <= 0.66f) return 1.0f;

  // 0.66 ... 0.73 V: noch gut, aber Balken nimmt sichtbar ab.
  if (volts <= 0.73f) {
    return clamp01(0.75f + ((0.73f - volts) / 0.07f) * 0.25f);
  }

  // 0.73 ... 0.82 V: schwacher Bereich, gelb bis rot.
  if (volts <= 0.82f) {
    return clamp01(0.25f + ((0.82f - volts) / 0.09f) * 0.50f);
  }

  // Ab 0.82 V wird es als sehr schwach/kaum Signal bewertet.
  // Bis ca. 0.95 V faellt der Balken weiter gegen 0.
  if (volts <= 0.95f) {
    return clamp01(((0.95f - volts) / 0.13f) * 0.25f);
  }

  return 0.0f;
}

// Wandelt die RF-Spannung in einen kurzen Text fuer das TFT um.
// Die Spannung selbst steht bereits oben im Signalblock; dieser Klartext ist
// nur die qualitative Bewertung fuer den Balken.
static const char* signalTextFromVoltage(float volts) {
  const char* label = "schlecht";
  if (volts <= 0.66f) label = "sehr gut";
  else if (volts <= 0.73f) label = "gut";
  else if (volts <= 0.82f) label = "schwach";

  snprintf(signalTextBuffer, sizeof(signalTextBuffer), "%s", label);
  return signalTextBuffer;
}

// Legt fest, welcher globale Anzeige-Modus auf dem TFT verwendet wird.
//
// Die UI-Modi sind rein darstellungsbezogen:
// - MENUE   -> Hauptmenue nach erfolgreichem Boot, keine Bewegung
// - MANUAL  -> ruhiger Betriebszustand im manuellen Modus
// - SEARCH  -> aktive Bewegung / laufende Suche
// - AUTO    -> AUTO aktiv und Peak erfolgreich gefunden
// - WARN    -> Fehlerzustand, z. B. MPU fehlt oder AUTO fehlgeschlagen
static UiMode currentUiMode() {
  // Ohne gültige MPU-Daten ist die Elevationsregelung nicht verlässlich.
  if (!liveMpuReady()) {
    return UI_MODE_WARN;
  }

  // Wenn AUTO in einen Fehler gelaufen ist, soll das klar sichtbar werden.
  if (liveAutoFailed()) {
    return UI_MODE_WARN;
  }

  // Hauptmenue nach erfolgreichem Boot.
  if (controlMode == CONTROL_MAIN_MENU) {
    return UI_MODE_MENU;
  }

  // Menuepunkt 1: Mitte einstellen zeigt einen eigenen Bedienbildschirm.
  if (controlMode == CONTROL_CENTER) {
    return UI_MODE_CENTER_ALIGN;
  }

  // AUTO aktiv:
  // - Peak schon gefunden -> AUTO
  // - ansonsten Suchzustand
  if (controlMode == CONTROL_AUTO) {
    if (liveAutoHasPeak()) {
      return UI_MODE_AUTO;
    }
    return UI_MODE_SEARCH;
  }

  // Auch im manuellen Modus soll während echter Bewegung SEARCH angezeigt werden.
  if (azimuthIsPulseActive() || elevationIsPulseActive()) {
    return UI_MODE_SEARCH;
  }

  // Standardfall: manuelle Bereitschaft
  return UI_MODE_MANUAL;
}

// Liefert den aktuell passenden Text für die Elevationszeile auf dem Display.
// Die eigentliche Entscheidung, welcher Text sinnvoll ist, liegt in live_runtime.cpp.
static const char* currentElevationText() {
  return liveGetElevationStateText();
}

// Liefert die grobe Azimut-Richtung für das Display.
// Dieser Text ist bewusst kurz, damit er in die kleine TFT-Zeile passt.
static const char* currentAzimuthText() {
  switch (azimuthGetDirection()) {
    case AZ_DIR_EAST: return "EAST >>";
    case AZ_DIR_WEST: return "WEST <<";
    default:          return "STOP";
  }
}

// Liefert den erweiterten Azimut-Zustand, z. B. "scan grob", "Mitte", "peak".
// Die eigentliche Zustandslogik liegt in live_runtime.cpp.
static const char* currentAzimuthState() {
  return liveGetAzimuthStateText();
}

// Zusätzlicher Infotext für die untere Displayzeile.
// Falls der RF-Detektor noch nicht initialisiert ist, wird das direkt gemeldet.
static const char* currentInfoText() {
  if (controlMode == CONTROL_MAIN_MENU || controlMode == CONTROL_CENTER) {
    return liveGetInfoText();
  }

  // AUTO-Anzeigen haben Vorrang vor der allgemeinen RF-Warnung.
  // Grund: Waehrend der AUTO-Mittenfahrt gibt es oft noch keinen gueltigen
  // RF-Drop. Trotzdem muss auf dem TFT sichtbar bleiben, dass gerade
  // "AUTO: MITTENFAHRT", "GROBSUCHE" usw. laeuft.
  if (controlMode == CONTROL_AUTO) {
    return liveGetInfoText();
  }

  const char* liveInfo = liveGetInfoText();

  // Warn- und Setup-Anzeigen aus der Runtime haben Vorrang vor
  // allgemeinen RF-Hinweisen. Sonst wuerden AZ-Limit-Warnungen im manuellen
  // Override oder im Startblock durch "Receiver/Signalweg pruefen" verdeckt.
  if (strncmp(liveInfo, "AZWARN|", 7) == 0 || strncmp(liveInfo, "AUTO_SETUP|", 11) == 0) {
    return liveInfo;
  }

  // Manuelle EZ-Steuerung ist echter Override; alte Softlimit-Stopps greifen im manuellen EZ nicht mehr.
  // Hauptmenue- und Center-Anzeigen duerfen nicht durch die allgemeine
  // RF-Warnung ersetzt werden. Sonst aendert PLUS/MINUS zwar intern die Auswahl
  // (Serial: "HAUPTMENUE Auswahl"), das TFT bekommt aber nur
  // "RF: Receiver/Signalweg pruefen" und bleibt optisch auf Menuepunkt 1.
  // RF-Hinweise sind fuer Live-/Suchbetrieb sinnvoll, aber nicht fuer die reine
  // Menue-Navigation.
  if (controlMode == CONTROL_MAIN_MENU || controlMode == CONTROL_CENTER) {
    return liveInfo;
  }

  if (!rfIsReady()) return "INFO: RF nicht bereit";

  // Praxis-Hinweis fuer den RF-Test ausserhalb der Automatik:
  // Der Sat-Receiver muss eingeschaltet sein. Bleibt DROP_ADC unter 30,
  // kommt am RF-Detector sehr wahrscheinlich kein verwertbares Signal an.
  if (!rfHasValidSignalDrop()) {
    return "RF: Receiver/Signalweg pruefen";
  }

  return liveInfo;
}

// Baut aus den aktuellen Live-Daten die komplette Display-Struktur zusammen
// und übergibt sie an displayRender().
//
// Hier werden keine Motoren gesteuert und keine Zustände geändert.
// Diese Funktion sammelt nur Daten für die Anzeige.
static void updateDisplayFromLiveData() {
  DisplayData ui;

  // Aktuellen Signalwert fuer die Anzeige bestimmen.
  // Der normierte Balken wird aus der RF-Spannung gebildet.
  // Niedrigere Spannung bedeutet bei deinem Aufbau staerkeres Signal.
  const float signalVolts = rfGetPinVoltage();
  const float signalNorm = currentSignalNormFromVoltage(signalVolts);

  // Für die Anzeige den besten bisher erreichten Signalwert merken
  if (signalNorm > bestSignalNorm) {
    bestSignalNorm = signalNorm;
  }

  // Globale UI-Färbung / Modus
  ui.mode = currentUiMode();

  // RF-Block
  ui.signalVolts = signalVolts;
  ui.signalNorm = signalNorm;
  ui.signalBestNorm = bestSignalNorm;
  ui.signalText = signalTextFromVoltage(signalVolts);

  // Elevation-Block
  ui.elevationDeg = liveMpuReady() ? liveGetRelativeAngleDeg() : 0.0f;
  ui.elevationText = currentElevationText();

  // Azimut-Block
  ui.azimuthText = currentAzimuthText();
  ui.azimuthState = currentAzimuthState();

  // Untere Infozeile
  ui.infoText = currentInfoText();

  // An das Display-Modul übergeben
  displayRender(ui);
}


// -----------------------------------------------------
// Der fruehere manuelle EZ-Startbildschirm wurde in V3 entfernt.
// Der Start erfolgt nach dem MPU-Test direkt mit dem Sued-Hinweis und danach
// mit dem Hauptmenue. Die Elevation kann weiterhin in den passenden Menues
// manuell korrigiert werden.

// -----------------------------------------------------
// setup()
// -----------------------------------------------------
// Wird genau einmal beim Start des ESP32 ausgeführt.
// Hier wird die komplette Systeminitialisierung in einer
// sinnvollen Reihenfolge durchgeführt.
void setup() {
  // Serielle Schnittstelle für Diagnosemeldungen
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);

#if DEBUG_BOOT
  Serial.println();
  Serial.println("==================================");
  Serial.println("Selfsat Autotracker - V3");
  Serial.println("==================================");
#endif

  // Runtime-Einstellungen initialisieren.
  // No-NVS-Version: Es werden immer die Code-Defaults aus settings.cpp
  // verwendet. Alte Werte im ESP32-Flash werden bewusst ignoriert.
  bool loadedFromFlash = initSettings();

#if DEBUG_BOOT
  if (loadedFromFlash) {
    Serial.println("Settings aus Flash geladen. WARNUNG: In No-NVS-Version unerwartet.");
  } else {
    Serial.println("Settings aus Code-Defaults gesetzt. Preferences/NVS wird nicht benutzt.");
  }
#endif

  // Aktive Code-Defaultwerte nur bei ausfuehrlicher Diagnose ausgeben.
  // Normalerweise bleibt DEBUG_VERBOSE = 0, damit der serielle Monitor
  // nicht mit dem kompletten Settings-Dump ueberladen wird.
#if DEBUG_VERBOSE
  printSettingsToSerial();
#endif

  // TFT zuerst initialisieren.
  // Grund: Der GY-521-/MPU6050-Starttest soll direkt am Gerät angezeigt
  // werden, bevor irgendein automatischer Suchlauf freigegeben werden kann.
  displayInit();
  displayShowSplash();

  // Live-Runtime initialisieren.
  // Hier werden u. a. Taster, Azimut, Elevation und der GY-521/MPU6050
  // gestartet. Die Runtime setzt liveMpuReady() nur dann auf true, wenn
  // der Sensor gefunden, kalibriert und plausibel eingelesen wurde.
  initLiveRuntime();

  // Ergebnis des GY-521-Starttests auswerten.
  // Kommentarstand: V3
  //
  // Sicherheitsentscheidung:
  // Wenn der MPU6050/GY-521 fehlt oder nicht initialisiert werden konnte, wird
  // der Bootvorgang hier bewusst angehalten. Ohne sicheren Elevationswinkel
  // darf weder Suche noch manuelle Bewegung als normaler Betriebszustand
  // freigegeben werden. Beide Motorachsen werden gestoppt, das TFT zeigt einen
  // dauerhaften Fehlerhinweis. Danach soll die Anlage stromlos gemacht, die
  // Sensor-/I2C-Verkabelung geprueft und der ESP32 neu gestartet werden.
  if (!liveMpuReady()) {
    azimuthStop();
    elevationStop();

#if DEBUG_BOOT
    Serial.println("START GESPERRT: MPU6050/GY-521 fehlt oder konnte nicht initialisiert werden.");
    Serial.println("Anlage stromlos machen, Sensor/I2C pruefen und danach neu starten.");
#endif

    displayShowMpuFatalError();

    while (true) {
      // Keine WLAN-/OTA-/Webserver-Initialisierung und keine Runtime.
      // Der Fehler ist absichtlich nur durch Strom AUS/EIN bzw. Reset nach
      // behobener Ursache zu verlassen.
      delay(1000);
    }
  }

  // Erfolgreicher MPU-Start: kurze OK-Anzeige, dann direkt ins Hauptmenue.
  displayShowMpuBootTestResult(true, liveGetRelativeAngleDeg());

  // V3: Der fruehere 10-s-EZ-Startbildschirm wurde ersatzlos entfernt.
  // Es gibt keine eigene Boot-EZ-Startphase mehr. Die Elevation kann weiterhin
  // bewusst in den Menues Ausrichten, Suchen oder Manuell EZ korrigiert werden.

  // Nach dem MPU-Test wird das echte Hauptmenue angezeigt.
  // Wichtig: Dabei wird NICHT automatisch in MANUELL gewechselt und
  // keine Automatik startet von allein.
  liveCommandOpenMainMenu();
  updateDisplayFromLiveData();

  // Erst nach dem lokalen Sensor-/Sicherheitstest werden Netzwerkfunktionen
  // gestartet. Das macht die Boot-Reihenfolge klarer und verhindert, dass
  // der Eindruck entsteht, das System sei suchbereit, obwohl der GY-521 fehlt.

  // WLAN initialisieren
  wifiInit();

  // OTA initialisieren.
  // V3: OTA ist weiterhin aktiv und bewusst mit dem bekannten
  // Passwort aus secrets.h gesichert. Der vorherige Kommentar "ohne Passwort"
  // war veraltet und hat nicht zum realen Code in ota_manager.cpp gepasst.
  // Wichtig fuer den Test: OTA wird erst nach der lokalen Startphase und
  // nach erfolgreicher WLAN-Verbindung sichtbar.
  otaInit();

  // Webserver starten
  webServerInit();

  // RF-Detektor initialisieren
  initRFDetector();

  // RF wird beim Boot nur initialisiert.
  // Die aktuelle V3-AUTO-Logik bewertet das Signal waehrend der Suche
  // dynamisch: hoehere ADC-Werte dienen als schwache Referenz, niedrigere
  // ADC-/Spannungswerte als moeglicher Signalpeak.
  // Ein fester RF-Zero wird beim Boot bewusst nicht gesetzt.

#if DEBUG_BOOT
  Serial.println("RF-Test: Bitte Sat-Receiver einschalten.");
  Serial.println("RF-Anzeige nutzt RF-Spannung; AUTO bewertet Signal dynamisch.");
  Serial.println("Live-Setup abgeschlossen.");
#endif
}

// -----------------------------------------------------
// loop()
// -----------------------------------------------------
// Wird vom ESP32 fortlaufend immer wieder aufgerufen.
// Hier laufen alle zyklischen Systemfunktionen zusammen.
void loop() {
  // WLAN-Reconnect / Überwachung
  wifiLoop();

  // OTA-Events verarbeiten.
  // Muss zyklisch sehr häufig laufen, damit Uploads stabil angenommen werden.
  otaLoop();

  // Browser-Anfragen bearbeiten
  webServerLoop();

  // RF-Werte fortlaufend aktualisieren
  rfUpdate();

  // Hauptlogik:
  // - Taster
  // - MANUELL / AUTO
  // - AUTO-Zustandsmaschine
  // - Sensor-/Motorlaufzeit
  runLiveRuntime();

  // Anzeige aktualisieren
  updateDisplayFromLiveData();

  // Kompakte serielle Statusausgabe.
  // Wird ueber DEBUG_STATUS in config.h ein- oder ausgeschaltet.
  // Ziel: wichtige Live-Werte sehen, ohne den Monitor mit Detaildaten zu fluten.
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