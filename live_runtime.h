/*
  SatAlign V3 - Runtime-API
  ------------------------------------------------------------
  Oeffentliche Funktionen fuer Web-UI, TFT und Hauptsketch. Diese API kapselt
  die internen Zustaende, damit Anzeige und Bedienung keine Motorlogik direkt
  nachbauen muessen.
*/

#pragma once

// =====================================================
// Live Runtime - Oeffentliche Schnittstelle
// =====================================================
//
// Diese Header-Datei beschreibt die gesamte von außen sichtbare
// Schnittstelle der zentralen Laufzeitlogik.
//
// WICHTIG:
// Die eigentliche Fachlogik liegt in live_runtime.cpp.
// Diese Datei hier enthaelt nur:
// - Initialisierungsfunktionen
// - zyklischen Runtime-Aufruf
// - Statusabfragen fuer TFT / Web / Hauptsketch
// - Bedienkommandos fuer Taster / Web UI / spaetere Erweiterungen
//
// Ziel dieser Trennung:
// - zentrale Wahrheit in live_runtime.cpp
// - alle anderen Module greifen nur ueber diese klaren Funktionen zu
//
// Das hilft dabei, dass:
// - Web UI
// - TFT-Anzeige
// - Hauptsketch
// - spaetere Testsketche
//
// immer denselben Zustand sehen und dieselben Kommandos verwenden.

// =====================================================
// Initialisierung / Hauptzyklus
// =====================================================

// Initialisiert die komplette Live-Runtime.
//
// Typische Aufgaben:
// - Taster vorbereiten
// - Azimut / Elevation initialisieren
// - MPU6050 starten und kalibrieren
// - interne AUTO-Zustaende zuruecksetzen
// - Kandidatenstatus und AUTO-Grundzustand vorbereiten
// - System in einen definierten Startzustand bringen
//
// Diese Funktion wird einmal in setup() aufgerufen.
void initLiveRuntime();

// Fuehrt einen kompletten Zyklus der Live-Laufzeitlogik aus.
//
// Typischer Inhalt:
// - Taster auswerten
// - AUTO-Zustandsmaschine fortfuehren
// - Kandidatenlogik / AUTO-Suchlogik verarbeiten
// - Sensorwerte aktualisieren
// - Motorstatus pflegen
// - serielle Diagnose erzeugen
//
// Diese Funktion wird zyklisch in loop() aufgerufen.
void runLiveRuntime();

// =====================================================
// Grundstatus
// =====================================================

// true, wenn die MPU betriebsbereit ist und die Runtime insgesamt
// erfolgreich initialisiert wurde.
//
// Praktisch wichtig fuer:
// - Freigabe des AUTO-Modus
// - Warnanzeige auf TFT / Web
// - Testlogik
bool liveMpuReady();

// Liefert den aktuellen relativen Elevationswinkel.
//
// Dieser Wert ist im Projekt besonders wichtig fuer:
// - Anzeige
// - Softlimits
// - Zielanfahrten
// - AUTO-Strategie
float liveGetRelativeAngleDeg();

// Liefert den aktuell gefilterten Winkel des MPU-Filters.
//
// Nuetzlich fuer:
// - Diagnose
// - Web-Status
// - Vergleich mit Referenz- / Relativwinkel
float liveGetFilteredAngleDeg();

// Liefert den aktuell aktiven Hauptmodus als Text,
// typischerweise "AUTO" oder "MANUELL".
const char* liveGetModeText();

// Liefert die aktuell gewaehlte manuelle Achse als Text,
// also z. B. "AZIMUT" oder "ELEVATION".
const char* liveGetAxisText();

// Liefert einen kurzen Text fuer den aktuellen AUTO-Unterzustand.
//
// Typische Beispiele:
// - "GO LIMIT"
// - "SCAN COARSE"
// - "CANDIDATE"
// - "AZ FINE"
// - "EL FINE"
// - "COMPLETE"
// - "FAILED"
const char* liveGetAutoStateText();

// Liefert einen kompakten Status-Text fuer die Azimut-Achse.
const char* liveGetAzimuthStateText();

// Liefert einen kompakten Status-Text fuer die Elevations-Achse.
const char* liveGetElevationStateText();

// Liefert einen allgemeinen Info-Text zum aktuellen Systemzustand.
//
// Dieser Text ist besonders nuetzlich fuer:
// - untere TFT-Zeile
// - Web-Oberflaeche
// - spaetere Diagnose
const char* liveGetInfoText();

// true, wenn die AUTO-Strategie einen gueltigen Peak gefunden hat.
bool liveAutoHasPeak();

// true, wenn die AUTO-Strategie in einem Fehlerzustand ist.
bool liveAutoFailed();

// =====================================================
// Erweiterter Status fuer Kandidatenlogik
// =====================================================
//
// Diese Statusabfragen sind fuer die aktuelle Suchstrategie gedacht.
// Ein gefundener Kandidat wird angehalten und der Nutzer entscheidet:
// PLUS/OK = richtiger Satellit, MINUS/FALSCH = Bereich sperren und weitersuchen.

// true, wenn das System aktuell in einem Kandidaten-Hold steht.
//
// Bedeutung:
// - Suchlauf hat einen ausreichend interessanten Peak gefunden
// - Motoren stehen
// - TV / Benutzer kann pruefen, ob der Satellit richtig oder falsch ist
bool liveIsCandidateHold();

// true, wenn der richtige Satellit bereits bestaetigt wurde.
//
// Dieser Zustand ist wichtig fuer die deutliche Anzeige
// "SATELLIT GEFUNDEN".
bool liveIsSatelliteConfirmed();

// Liefert die Anzahl aktuell gespeicherter Sperrbereiche
// fuer falsche Satelliten.
//
// Nuetzlich fuer:
// - Web-Status
// - serielle Diagnose
// - Web-UI und Debug-Seiten
int liveGetBlockedRangeCount();

// =====================================================
// Web-/Bedienkommandos - Grundfunktionen
// =====================================================
//
// Diese Funktionen bilden die oeffentliche Kommandoschnittstelle
// der Runtime. Alle Bedienwege sollen spaeter nach Moeglichkeit
// ueber diese Funktionen gehen:
// - Tasterlogik
// - Web UI
// - spaetere Testskripte

// Schaltet das System explizit in den manuellen Modus.
void liveCommandEnterManual();

// Oeffnet das Hauptmenue nach dem Boot.
// In diesem Zustand laufen keine Motoren; PLUS/MINUS waehlt Menuepunkte.
void liveCommandOpenMainMenu();

// Startet Menuepunkt 1: automatische Referenzfahrt zur Mitte.
void liveCommandStartCentering();
void liveCommandStartCenterRun();
// Bricht eine laufende Ausrichten-/Centerfahrt aus der Web-UI ab.
void liveCommandAbortCentering();
// Zeigt, ob der Ausrichten-/Center-Modus aktiv ist.
bool liveCenteringActive();
// Zeigt, ob die letzte manuell/Web gestartete Ausrichten-Fahrt erfolgreich beendet wurde.
bool liveCenteringSuccessNoticeActive();

// V3: Live-Texte fuer die Web-UI-Seite Ausrichten/Mitte.
// Diese Funktionen liefern menschenlesbare Phasen- und Infotexte, damit
// web_server.cpp nicht interne Center-Zustaende interpretieren muss.
// Dadurch bleiben TFT, Web-UI und Runtime logisch sauber getrennt.
const char* liveGetCenteringPhaseText();
const char* liveGetCenteringInfoText();

// Startet / aktiviert den AUTO-Modus.
void liveCommandEnterAuto();

// Startet eine manuelle Azimutbewegung nach EAST.
void liveCommandAzEast();

// Startet eine manuelle Azimutbewegung nach WEST.
void liveCommandAzWest();

// Stoppt die Azimutbewegung.
void liveCommandAzStop();

// Startet eine manuelle Elevationsbewegung "hoch".
void liveCommandElUp();

// Startet eine manuelle Elevationsbewegung "runter".
void liveCommandElDown();

// Stoppt die Elevationsbewegung.
void liveCommandElStop();

// Kurze Elevationskorrektur im Setup-/Ausrichten-Kontext.
// Wird von der Web-UI im AUTO SETUP genutzt, ohne in den manuellen Dauerfahrtmodus zu wechseln.
void liveCommandSetupElevationUpPulse();
void liveCommandSetupElevationDownPulse();


// =====================================================
// Web-/Diagnosefunktionen
// =====================================================
// Kommentarstand: V3
//
// Diese Funktionen liefern strukturierte Werte fuer das Web-UI.
// Dadurch muss web_server.cpp keine Status-Texte parsen.
// Das ist wichtig, weil Texte fuer Menschen gedacht sind, waehrend das Web-UI
// fuer Farben, Buttons und Sperrlogik stabile bool-/Zahlenwerte braucht.

// Such-Setup wie am TFT.
// V3: Feinsuche AN/AUS wurde entfernt; PLUS/MINUS korrigiert die Hoehe,
// MODE bzw. Web-Start startet den Suchlauf Mitte -> Ost -> West -> Mitte.
bool liveAutoSetupActive();
bool liveAutoFineSearchEnabled();   // nur noch Kompatibilitaets-/Diagnosewert
void liveCommandToggleAutoFineSearch(); // tut in V3 bewusst nichts mehr
void liveCommandStartAutoFromSetup();
void liveCommandAbortAuto();

// Web-Tastenlogik analog zu den physischen Tasten im manuellen Modus.
// AZ: PLUS/MINUS entsprechen der getesteten Bedienrichtung.
void liveCommandAzButtonPlus();
void liveCommandAzButtonMinus();
void liveCommandElButtonPlus();
void liveCommandElButtonMinus();

// Web-UI Spiegelung der manuellen Start-/Stop-Befehle.
// Diese Funktionen werden nur fuer die Anzeige der Web-Buttons genutzt.
bool liveWebManualAzActive();
bool liveWebManualElActive();
const char* liveWebManualAzDirectionText();
const char* liveWebManualElDirectionText();

// RF-/Signalstatus fuer Web-UI.
float liveGetRfVoltage();
uint16_t liveGetRfRawAdc();
float liveGetRfFilteredAdc();
const char* liveGetRfQualityText();

// V3: Status fuer die versteckte Web-UI-Seite "Signal optimieren".
// Die Seite wird automatisch angezeigt, wenn die Optimierung nach PLUS
// wirklich laeuft. Die Werte dienen nur Anzeige/Diagnose; die eigentliche
// Optimierungslogik bleibt vollstaendig in live_runtime.cpp.
bool liveSignalOptimizationActive();
const char* liveGetSignalOptimizationPhaseText();
const char* liveGetSignalOptimizationInfoText();
float liveGetSignalOptimizationStartAdc();
float liveGetSignalOptimizationBestAdc();
float liveGetSignalOptimizationCurrentAdc();
int liveGetSignalOptimizationStepsInPhase();

// Hallstatus: Referenzlogik entspricht der getesteten Mittenfunktion.
bool liveHallCenter();
bool liveHallEast();
bool liveHallWest();
bool liveRawHallCenter();
bool liveRawHallEast();
bool liveRawHallWest();

// =====================================================
// Web-/Bedienkommandos - Kandidatenlogik
// =====================================================
//
// Diese Funktionen sind fuer die neue Suchlogik gedacht:
//
// Kandidat gefunden  -> Benutzer prueft TV-Bild
// PLUS / Web-Button  -> richtiger Satellit
// MINUS / Web-Button -> falscher Satellit / weiter suchen
// MODE lang / Web    -> AUTO abbrechen

// Markiert den aktuell gehaltenen Kandidaten als "falscher Satellit".
//
// Wirkung in der Runtime typischerweise:
// - Kandidat klassifizieren
// - Sperrbereich anlegen / zusammenfuehren
// - Grobsuche fortsetzen
void liveCommandCandidateFalse();

// Bestaetigt den aktuell gefundenen Satelliten als "richtig".
//
// Typische Wirkung:
// - alle Motoren aus
// - Status "SATELLIT GEFUNDEN"
// - danach kann der Benutzer die Versorgung abschalten
void liveCommandConfirmSatellite();

