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
// Kurze MPU-Stabilisierung vor der Boot-EZ-Anfahrt
// -----------------------------------------------------
// Kommentarstand: V3
//
// Zweck:
// Direkt nach dem Einschalten darf die automatische EZ-Anfahrt nicht mit
// einem noch wandernden MPU-Komplementaerfilter starten. Diese Funktion
// liest den Sensor fuer eine kurze Zeit ein und bewertet, ob die angezeigte
// EZ in einem kleinen Fenster stabil bleibt.
//
// Live-Test-Hintergrund V3:
// Beim ersten Standard-EZ-Start wurde sichtbar, dass der Istwert auf dem TFT
// direkt nach dem Boot stark von den spaeter stabilen Live-Werten abweichen
// kann. Ursache ist nicht der Motor, sondern die Einschwingphase aus MPU6050,
// Komplementaerfilter und mechanischer Lage. Diese Warte-/Messphase verhindert,
// dass die Anfahrt mit einem Momentanwert startet, der nur aus dem Filterstart
// stammt.
//
// Wichtig:
// - Keine Motorbewegung in dieser Phase.
// - Keine Menue-/Suchlogik in dieser Phase.
// - Die Funktion dient nur dazu, dass die Boot-Anfahrt mit einem plausiblen
//   Istwert startet und nicht mit einem alten Filter-Startwert.
// - Die Anzeige wird absichtlich nur ca. 1x pro Sekunde aktualisiert, damit
//   Soll/Ist/Countdown auf dem kleinen TFT lesbar bleiben und keine
//   Einschalt-Rudimente durch schnelle Vollbild-Updates entstehen.
static bool bootStabilizeElevationSensor() {
  if (!liveMpuReady()) {
    return false;
  }

  Serial.println("BOOT EZ V3: stabilisiere MPU/EZ-Wert vor Standard-Anfahrt...");

  const unsigned long startedAt = millis();
  // V3: 3 Sekunden Stabilisierung statt kurzer Momentaufnahme.
  // Diese Dauer ist ein Erfahrungswert aus dem Live-Test: lang genug, damit
  // der Filter sichtbar Richtung Echtwert kommt, aber kurz genug, damit der
  // Boot-Vorgang nicht unnoetig traege wirkt.
  const unsigned long stabilizeMs = 3000;

  // V3: Display-Drosselung fuer bessere Lesbarkeit.
  // Der TFT-Startbildschirm wird nicht in jedem 10-ms-Zyklus neu aufgebaut,
  // sondern nur ungefaehr einmal pro Sekunde. Dadurch verschwinden
  // Einschalt-Rudimente und der User kann die Werte wirklich lesen.
  unsigned long lastDisplayMs = 0;

  float minEz = 999.0f;
  float maxEz = -999.0f;
  float currentEz = liveGetRelativeAngleDeg();

  while (millis() - startedAt < stabilizeMs) {
    mpuUpdateFilteredAngle();
    elevationUpdate();

    currentEz = liveGetRelativeAngleDeg();
    if (currentEz < minEz) minEz = currentEz;
    if (currentEz > maxEz) maxEz = currentEz;

    const unsigned long now = millis();
    if (now - lastDisplayMs >= 1000) {
      lastDisplayMs = now;
      // moving=false, reached=false, error=false:
      // Die vorhandene Boot-EZ-Anzeige wird bewusst wiederverwendet.
      // So sieht man auf dem TFT bereits den Sollwert und den aktuellen
      // stabilisierenden Istwert, ohne eine neue Display-Seite einzubauen.
      displayShowBootElevationTarget(currentEz, DEFAULT_TARGET_ELEVATION, false, false, false);
    }

    delay(10);
  }

  const float spread = maxEz - minEz;
  Serial.print("BOOT EZ V3: Stabilisierung fertig | Ist=");
  Serial.print(currentEz, 2);
  Serial.print(" deg | Schwankung=");
  Serial.print(spread, 2);
  Serial.println(" deg");

  // Bei zu grosser Schwankung wird nicht hart abgebrochen.
  // Die Anfahrt darf weiterlaufen, aber der serielle Hinweis zeigt klar,
  // dass mechanische Vibration, Sensorlage oder Startbewegung geprueft
  // werden sollten.
  if (spread > 1.5f) {
    Serial.println("BOOT EZ V3: Hinweis - EZ-Wert schwankt noch deutlich.");
    return false;
  }

  return true;
}

// -----------------------------------------------------
// Restzeit fuer den sichtbaren EZ-Start-Countdown
// -----------------------------------------------------
// Kommentarstand: V3
//
// Der Countdown soll fuer den User einfach lesbar sein: 30, 29, 28 ... 0 s.
// Deshalb wird auf ganze Sekunden aufgerundet. So verschwindet die Anzeige
// nicht optisch zu frueh, obwohl intern noch Millisekunden Restzeit laufen.
static long bootElevationRemainingSeconds(unsigned long startedAt) {
  const unsigned long now = millis();
  const unsigned long elapsed = now - startedAt;

  if (elapsed >= BOOT_ELEVATION_DISPLAY_TIMEOUT_MS) {
    return 0;
  }

  const unsigned long remainingMs = BOOT_ELEVATION_DISPLAY_TIMEOUT_MS - elapsed;
  return (long)((remainingMs + 999UL) / 1000UL);
}

// -----------------------------------------------------
// Boot-Anfahrt auf zentrale Standard-Elevation
// -----------------------------------------------------
// Kommentarstand: V3
//
// Zweck:
// Nach jedem Einschalten soll die Elevationsachse zuerst auf einen definierten
// Standard-Sollwert fahren, bevor das normale Hauptmenue erscheint.
// Dadurch startet jeder Test von einer bekannten EZ-Basis und nicht zufaellig
// von der mechanischen Stellung, in der die Antenne zuletzt ausgeschaltet wurde.
//
// Zentraler Sollwert:
// - DEFAULT_TARGET_ELEVATION in settings.cpp
// - aktuell 30.0 Grad
//
// Wichtige Abgrenzung:
// Diese Funktion startet keine Satellitensuche, keine AZ-Bewegung und keine
// AUTO-Zustandsmaschine. Sie bewegt ausschliesslich die Elevationsachse in
// kleinen Pulsen auf den Standard-EZ-Wert.
//
// Sicherheitsverhalten:
// - nur bei bereitem MPU/GY-521
// - die erste Boot-Anfahrt ignoriert bewusst die softwareseitigen EZ-Limits
// - nicht blockierend fuer immer: BOOT_ELEVATION_TIMEOUT_MS bricht ab
// - TFT zeigt waehrenddessen einen Countdown auf 30 Sekunden
// - Elevation wird am Ende immer gestoppt
//
// Live-Test-Hintergrund V3:
// Der Aktuator bewegte sich zwar in Richtung Sollwert, der gefilterte Istwert
// lief aber waehrend der Startphase noch nach. Deshalb reicht ein einmaliges
// Erreichen innerhalb der Toleranz nicht aus. Diese Routine stoppt nach einem
// Treffer, wartet die mechanische/Filter-Beruhigung ab und bewertet erst dann
// den stabilen Wert. So soll verhindert werden, dass die Schuessel scheinbar
// den Standardwinkel erreicht, danach aber im Hauptmenue wieder deutlich
// daneben steht.
//
// V3: Softwarelimits bei der ersten Standard-EZ bewusst aufgehoben.
// Beim Einschalten ist diese Bewegung eine definierte Startpositionierung und
// keine normale Suchbewegung. Falls der fruehe Sensor-/Filterwert ein Limit
// scheinbar schon erreicht hat, soll die Anlage trotzdem in Richtung des
// zentralen Standardwinkels fahren duerfen. Die mechanischen Endabschalter des
// Linearantriebs bleiben weiterhin die harte Schutzebene.
static bool bootDriveElevationToDefaultTarget() {
  if (!liveMpuReady()) {
    Serial.println("BOOT EZ V3: uebersprungen, MPU nicht bereit.");
    return false;
  }

  if (DEFAULT_TARGET_ELEVATION < ELEVATION_MIN_SOFT || DEFAULT_TARGET_ELEVATION > ELEVATION_MAX_SOFT) {
    // V3: Beim allerersten Standard-EZ-Start wird das nicht mehr als
    // Abbruchgrund behandelt. Der zentrale Sollwert soll testweise auch dann
    // angefahren werden duerfen, wenn die normalen Arbeitsgrenzen spaeter noch
    // enger gesetzt werden. Deshalb nur Diagnose ausgeben, aber nicht stoppen.
    Serial.print("BOOT EZ V3: Hinweis - Standardziel ausserhalb normaler Softwarelimits: ");
    Serial.print(DEFAULT_TARGET_ELEVATION, 2);
    Serial.print(" deg | Limits ");
    Serial.print(ELEVATION_MIN_SOFT, 1);
    Serial.print("-");
    Serial.println(ELEVATION_MAX_SOFT, 1);
  }

  Serial.print("BOOT EZ V3: Standard-EZ anfahren | Soll=");
  Serial.print(DEFAULT_TARGET_ELEVATION, 2);
  Serial.print(" deg | Toleranz=");
  Serial.print(BOOT_ELEVATION_TOLERANCE_DEG, 2);
  Serial.print(" deg | Timeout=");
  Serial.print(BOOT_ELEVATION_TIMEOUT_MS / 1000UL);
  Serial.println("s");

  const unsigned long startedAt = millis();
  unsigned long lastDisplayMs = 0;
  unsigned long lastSerialMs = 0;

  // V3: Nachlauf-/Filter-Verifikation nach jedem vermeintlichen Treffer.
  //
  // Grund:
  // Beim Live-Test wurde waehrend der Boot-Anfahrt kurz 28,63 Grad erreicht
  // und wegen der Toleranz als "erreicht" akzeptiert. Nach Motorstopp und
  // Beruhigung wanderte der gefilterte Istwert aber wieder auf ca. 27,7 Grad.
  // Das bedeutet: Der Treffer darf nicht sofort als final gelten, solange der
  // Motor gerade lief oder der MPU-Komplementaerfilter nach Bewegung/Vibration
  // noch nachzieht.
  //
  // Ablauf jetzt:
  // 1. Zielregelung meldet innerhalb Toleranz.
  // 2. Motor sofort stoppen.
  // 3. Kurze Beruhigungszeit mit laufenden MPU-Updates abwarten.
  // 4. Erst den NACH dem Setzen stabil gemessenen EZ-Wert bewerten.
  // 5. Wenn der Wert wieder ausserhalb liegt, wird weiter nachgeregelt.
  // V3: Nachpruefung 2,5 Sekunden.
  // Der Motor ist in dieser Phase aus. Es laufen nur elevationUpdate() und
  // mpuUpdateFilteredAngle(), damit Pulsverwaltung und Sensorfilter sauber
  // auslaufen koennen. Erst danach wird entschieden, ob die Standard-EZ
  // wirklich stabil erreicht wurde.
  const unsigned long verifySettleMs = 2500;

  // Maximal erlaubte Schwankung waehrend der Nachpruefung.
  // Wenn der Wert staerker wandert, ist der Winkel noch nicht stabil genug und
  // die Boot-Anfahrt regelt weiter statt ins Hauptmenue zu wechseln.
  const float verifyMaxSpreadDeg = 0.45f;

  while (millis() - startedAt < BOOT_ELEVATION_TIMEOUT_MS) {
    // Sensor und Pulsverwaltung direkt bedienen, weil das normale Hauptmenue
    // zu diesem Zeitpunkt bewusst noch nicht aktiv angezeigt wird.
    mpuUpdateFilteredAngle();
    elevationUpdate();

    const float currentEz = liveGetRelativeAngleDeg();
    // V3: Erste Standard-EZ-Anfahrt bewusst ohne Softwarelimits.
    // Grund: Diese Anfahrt ist die definierte Startpositionierung direkt nach
    // dem Einschalten. Der MPU-/Filterwert kann in dieser Phase noch grob sein;
    // ein scheinbar erreichtes Softlimit darf diese Startpositionierung nicht
    // verhindern. Normale AUTO-/Suchbewegungen behalten ihre Softlimits.
    const bool reachedCandidate = moveToElevationPulseNoSoftLimits(currentEz, DEFAULT_TARGET_ELEVATION, BOOT_ELEVATION_TOLERANCE_DEG);

    const bool moving = elevationIsPulseActive();
    const unsigned long now = millis();
    const long remainingSeconds = bootElevationRemainingSeconds(startedAt);

    if (now - lastDisplayMs >= 1000 || reachedCandidate) {
      lastDisplayMs = now;
      displayShowBootElevationTarget(currentEz, DEFAULT_TARGET_ELEVATION, moving, false, false, remainingSeconds);
    }

    if (now - lastSerialMs >= 1000 || reachedCandidate) {
      lastSerialMs = now;
      Serial.print("BOOT EZ V3: Ist=");
      Serial.print(currentEz, 2);
      Serial.print(" deg | Soll=");
      Serial.print(DEFAULT_TARGET_ELEVATION, 2);
      Serial.print(" deg | Motor=");
      Serial.print(moving ? "LAEUFT" : "PULS/WAIT");
      Serial.print(" | Rest=");
      Serial.print(remainingSeconds);
      Serial.println("s");
    }

    if (reachedCandidate) {
      elevationStop();

      Serial.println("BOOT EZ V3: Kandidat erreicht, Motor stoppt - pruefe stabilen Istwert...");

      const unsigned long verifyStartedAt = millis();
      unsigned long verifyDisplayMs = 0;
      float verifyMinEz = 999.0f;
      float verifyMaxEz = -999.0f;
      float verifiedEz = liveGetRelativeAngleDeg();

      while (millis() - verifyStartedAt < verifySettleMs) {
        // Motor bleibt aus. Nur Sensor/Filter weiterlaufen lassen.
        elevationUpdate();
        mpuUpdateFilteredAngle();

        verifiedEz = liveGetRelativeAngleDeg();
        if (verifiedEz < verifyMinEz) verifyMinEz = verifiedEz;
        if (verifiedEz > verifyMaxEz) verifyMaxEz = verifiedEz;

        const unsigned long verifyNow = millis();
        if (verifyNow - verifyDisplayMs >= 1000) {
          verifyDisplayMs = verifyNow;
          displayShowBootElevationTarget(verifiedEz, DEFAULT_TARGET_ELEVATION, false, false, false, bootElevationRemainingSeconds(startedAt));
        }

        delay(10);
      }

      const float verifiedError = DEFAULT_TARGET_ELEVATION - verifiedEz;
      const float verifiedAbsError = fabsf(verifiedError);
      const float verifiedSpread = verifyMaxEz - verifyMinEz;

      Serial.print("BOOT EZ V3: Nachpruefung | Ist=");
      Serial.print(verifiedEz, 2);
      Serial.print(" deg | Fehler=");
      Serial.print(verifiedError, 2);
      Serial.print(" deg | Schwankung=");
      Serial.print(verifiedSpread, 2);
      Serial.println(" deg");

      if (verifiedAbsError <= BOOT_ELEVATION_TOLERANCE_DEG && verifiedSpread <= verifyMaxSpreadDeg) {
        displayShowBootElevationTarget(verifiedEz, DEFAULT_TARGET_ELEVATION, false, true, false);
        Serial.println("BOOT EZ V3: Standard-EZ stabil erreicht.");
        delay(900);
        return true;
      }

      // Noch nicht wirklich stabil auf Soll: weiterregeln.
      // Wichtig: Hier NICHT als Erfolg beenden. Genau dadurch wird verhindert,
      // dass das System nach einem kurzzeitig falschen Filterwert ins Hauptmenue
      // geht, obwohl die echte stabile EZ noch deutlich neben 29 Grad liegt.
      Serial.println("BOOT EZ V3: Nachpruefung ausserhalb Toleranz - regle weiter.");
      displayShowBootElevationTarget(verifiedEz, DEFAULT_TARGET_ELEVATION, false, false, false, bootElevationRemainingSeconds(startedAt));
      delay(150);
    }

    delay(10);
  }

  elevationStop();
  const float currentEz = liveGetRelativeAngleDeg();
  displayShowBootElevationTarget(currentEz, DEFAULT_TARGET_ELEVATION, false, false, true);
  // V3: Der Ablaufzeitpunkt wird bewusst als Hinweis behandelt.
  // Die TFT-Anzeige ist gelb/orange, nicht rot, damit der User den Zustand
  // nicht als schweren Fehler interpretiert. Die Motoren sind trotzdem sicher
  // gestoppt und das System geht kontrolliert weiter.
  Serial.print("BOOT EZ V3: ZEIT ERREICHT / HINWEIS | Ist=");
  Serial.print(currentEz, 2);
  Serial.print(" deg | Soll=");
  Serial.print(DEFAULT_TARGET_ELEVATION, 2);
  Serial.println(" deg");
  delay(1500);
  return false;
}

// -----------------------------------------------------
// Manueller Elevations-Startschritt vor dem Hauptmenue
// -----------------------------------------------------
// Kommentarstand: V3
//
// Hintergrund:
// Die automatische Boot-EZ-Anfahrt mit Countdown/Timeout wurde nach den
// Live-Tests bewusst aus dem Startablauf entfernt. Die Elevation muss fuer
// die Satellitensuche nicht exakt getroffen werden; sie soll nur grob in
// einen sinnvollen Startbereich gebracht werden. Der Nutzer kann diesen
// Startwinkel am sichersten selbst anhand der realen Antennenlage und seiner
// Erfahrung einstellen.
//
// Bedienung in dieser Startphase:
// - PLUS halten  -> Elevation hoch / Winkel wird groesser
// - MINUS halten -> Elevation runter / Winkel wird kleiner
// - PLUS+MINUS   -> sofort STOP
// - MODE kurz    -> sofort ins Hauptmenue wechseln
// - nach ca. 15 s -> automatisch weiter ins Hauptmenue
//
// Wichtig V3:
// Diese Startphase liegt vor dem Hauptmenue und vor WLAN/OTA/Webserver. Sie
// startet keine Suche und keine AUTO-Zustandsmaschine. Softwareseitige
// Elevationslimits werden hier bewusst nicht ausgewertet, weil es sich um
// eine manuelle grobe Anfangseinstellung handelt. Die mechanischen
// Endabschalter des Linearantriebs bleiben die harte Schutzebene.
//
// V3-Entscheidung:
// Die Phase hat wieder ein kurzes Zeitfenster, aber keine aggressive
// Fehlerwirkung. Helles Blau und Countdown zeigen nur: "Jetzt kann der Nutzer
// noch manuell korrigieren." Danach geht die Anlage automatisch weiter.
// V3: Das Zeitfenster fuer die manuelle Elevations-Startphase
// steht bewusst nicht mehr lokal in der .ino-Datei.
//
// Der Wert kommt jetzt zentral aus settings.cpp/config.h:
//   BOOT_MANUAL_ELEVATION_WINDOW_MS
//
// Grund:
// Diese Zeit ist ein Projekt-Parameter wie DEFAULT_TARGET_ELEVATION
// und soll spaeter an einer zentralen Stelle angepasst werden koennen.

static void bootManualElevationStart() {
  if (!liveMpuReady()) {
    Serial.println("BOOT EZ V3: manuelle EZ-Startphase uebersprungen, MPU nicht bereit.");
    return;
  }

  Serial.println("BOOT EZ V3: manuelle EZ-Startphase aktiv.");
  Serial.println("BOOT EZ V3: PLUS=hoch | MINUS=runter | PLUS+MINUS=STOP | MODE=weiter");

  // Vor Eintritt in die Handbedienung sicherstellen, dass kein alter Puls
  // oder Restzustand aus einer vorherigen Funktion aktiv ist.
  elevationStop();

  unsigned long lastDisplayMs = 0;
  unsigned long lastSerialMs = 0;
  bool lastModePressed = false;
  const unsigned long startMs = millis();

  while (true) {
    mpuUpdateFilteredAngle();
    elevationUpdate();

    const bool modePressed = (digitalRead(PIN_BTN_MODE) == LOW);
    const bool plusPressed = (digitalRead(PIN_BTN_PLUS) == LOW);
    const bool minusPressed = (digitalRead(PIN_BTN_MINUS) == LOW);

    // MODE-Flanke beendet die Startphase. Eine Flankenerkennung verhindert,
    // dass ein bereits beim Boot gedrueckter Taster sofort durchrutscht.
    if (modePressed && !lastModePressed) {
      elevationStop();
      Serial.println("BOOT EZ V3: manuelle EZ-Startphase beendet -> Hauptmenue.");
      delay(250);
      return;
    }
    lastModePressed = modePressed;

    const unsigned long now = millis();
    const unsigned long elapsedMs = now - startMs;
    if (elapsedMs >= BOOT_MANUAL_ELEVATION_WINDOW_MS) {
      elevationStop();
      Serial.println("BOOT EZ V3: manuelle EZ-Startphase Zeitfenster abgelaufen -> Hauptmenue.");
      delay(150);
      return;
    }

    if (plusPressed && minusPressed) {
      elevationStop();
    } else if (plusPressed) {
      elevationUp();
    } else if (minusPressed) {
      elevationDown();
    } else {
      elevationStop();
    }

    const float currentEz = liveGetRelativeAngleDeg();
    const long remainingSeconds = (long)((BOOT_MANUAL_ELEVATION_WINDOW_MS - elapsedMs + 999UL) / 1000UL);

    if (now - lastDisplayMs >= 250) {
      lastDisplayMs = now;
      displayShowManualElevationStart(currentEz, DEFAULT_TARGET_ELEVATION,
                                      plusPressed, minusPressed, remainingSeconds);
    }

    if (now - lastSerialMs >= 500) {
      lastSerialMs = now;
      Serial.print("BOOT EZ MANUELL V3 | IST=");
      Serial.print(currentEz, 2);
      Serial.print(" deg | EMPFOHLEN=");
      Serial.print(DEFAULT_TARGET_ELEVATION, 2);
      Serial.print(" deg | REST=");
      Serial.print(remainingSeconds);
      Serial.print("s | MOTOR=");
      if (plusPressed && !minusPressed) {
        Serial.println("HOCH");
      } else if (minusPressed && !plusPressed) {
        Serial.println("RUNTER");
      } else {
        Serial.println("STOP");
      }
    }

    delay(20);
  }
}

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

  // Ergebnis des GY-521-Starttests 3 Sekunden auf dem TFT anzeigen.
  // Wichtig: AUTO/Satellitensuche bleibt bei Fehler dauerhaft gesperrt.
  displayShowMpuBootTestResult(liveMpuReady(), liveGetRelativeAngleDeg());

  // Nach dem MPU-Test folgt keine automatische EZ-Anfahrt mehr.
  // Kommentarstand: V3
  //
  // V3-Entscheidung nach den Live-Tests:
  // Der fruehere automatische EZ-Start wurde aus dem Bootablauf entfernt.
  // Stattdessen bekommt der Nutzer ein ruhiges, hellblaues 15-s-Zeitfenster,
  // in dem er die Elevation mit PLUS/MINUS grob auf den empfohlenen Startwert
  // bringen kann. Danach geht die Anlage automatisch weiter.
  //
  // Vorteil:
  // - kein aggressiver Fehler-/Timeout-Bildschirm beim Einschalten
  // - keine automatische Fehlbewegung durch einen noch einpendelnden Sensorwert
  // - der Nutzer kann eigene Erfahrung mit dem Startwinkel einbringen
  // - der Start blockiert nicht dauerhaft, falls keine Taste gedrueckt wird
  if (liveMpuReady()) {
    bootManualElevationStart();
  }

  // Nach der manuellen EZ-Startphase wird wieder der bekannte Hinweis angezeigt:
  // Antenne grob nach Sueden ausrichten.
  // Diese Anzeige erfolgt bewusst vor WLAN/OTA/Webserver, damit waehrend
  // einer moeglichen WLAN-Wartezeit keine Grafikreste sichtbar bleiben.
  if (liveMpuReady()) {
    displayShowSouthAlignPrompt();
  }

  // Nach dem Hinweis wird das echte Hauptmenue angezeigt.
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