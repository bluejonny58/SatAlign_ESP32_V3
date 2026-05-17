/*
  SatAlign V3 - TFT-Anzeige API
  ------------------------------------------------------------
  Deklariert die Anzeige-Funktionen fuer Runtime und Hauptsketch. Das TFT
  zeigt Status; es entscheidet nicht selbst ueber Motorlogik.
*/

#pragma once

#include <Arduino.h>

// Globale Anzeigemodi des TFT.
//
// Diese Werte steuern hauptsächlich:
// - Farbgebung
// - Kopfzeile
// - allgemeine visuelle Einordnung des Systemzustands
enum UiMode {
  // Hauptmenue nach dem Boot, keine automatische Bewegung
  UI_MODE_MENU = 0,

  // Menuepunkt 1: Nach Sueden ausrichten / Ausrichten / Mitte referenzieren
  UI_MODE_CENTER_ALIGN,

  // Manueller Betriebszustand, ruhig / bedienbereit
  UI_MODE_MANUAL,

  // AUTO aktiv und erfolgreich / im automatischen Betriebsrahmen
  UI_MODE_AUTO,

  // Aktive Bewegung / Suchvorgang / laufende Regelung
  UI_MODE_SEARCH,

  // Warn- oder Fehlerzustand
  UI_MODE_WARN
};

// Kompletter Datensatz für einen Display-Renderzyklus.
//
// Diese Struktur wird im Hauptsketch bzw. in der Live-Logik
// mit aktuellen Werten befüllt und anschließend an displayRender()
// übergeben.
//
// WICHTIG:
// Diese Struktur enthält nur Anzeigedaten,
// keine Steuerlogik.
struct DisplayData {
  // Globale Anzeigefarbe / UI-Hauptmodus
  UiMode mode;

  // ---------------------------------------------------
  // RF / Signal
  // ---------------------------------------------------

  // Aktuelle gemessene RF-Spannung am ADC-Pfad.
  // Beispiel: 2.41 V
  float signalVolts;

  // Normierter aktueller Signalwert 0.0 ... 1.0
  // für Balken und farbliche Bewertung.
  float signalNorm;

  // Bester bisher beobachteter normierter Signalwert.
  // Wird auf dem Display typischerweise als Peak-Markierung dargestellt.
  float signalBestNorm;

  // Kurzer Klartext zum Signalstatus:
  // z. B. "Signal gut"
  const char* signalText;

  // ---------------------------------------------------
  // Elevation
  // ---------------------------------------------------

  // Aktuell angezeigter bzw. verwendeter Elevationswert.
  // Beispiel: 29.4
  float elevationDeg;

  // Kurzer Zustands-/Status-Text zur Elevation:
  // z. B. "im Bereich", "faehrt hoch", "MIN LIMIT"
  const char* elevationText;

  // ---------------------------------------------------
  // Azimut
  // ---------------------------------------------------

  // Kurze Richtungsanzeige:
  // "EAST >>", "STOP", "WEST <<"
  const char* azimuthText;

  // Zustandsbeschreibung der Azimut-Achse:
  // z. B. "bewegt sich", "wartet", "scan grob", "Mitte"
  const char* azimuthState;

  // ---------------------------------------------------
  // Untere Infozeile
  // ---------------------------------------------------

  // Freitext / zusammenfassende Info zum aktuellen Systemzustand,
  // z. B. "INFO: Azimut Grobsuche"
  const char* infoText;
};

// Initialisiert das TFT-Display.
//
// Aufgaben typischerweise:
// - Controller initialisieren
// - Rotation setzen
// - Grundlayout vorbereiten
void displayInit();

// Zeigt kurz einen Splashscreen / Startbildschirm an.
//
// Dient vor allem dazu, beim Boot visuell zu sehen,
// dass das Display korrekt gestartet ist.
void displayShowSplash();

// Zeigt das Ergebnis des MPU6050-/GY-521-Boottests kurz auf dem TFT an.
void displayShowMpuBootTestResult(bool ok, float relativeAngleDeg);

// Zeigt einen dauerhaften Startfehler, wenn der MPU6050/GY-521 fehlt
// oder nicht initialisiert werden konnte. Diese Anzeige wird bewusst nicht
// automatisch verlassen: Die Anlage soll stromlos gemacht, der Sensor bzw.
// die Verkabelung geprueft und danach neu gestartet werden.
void displayShowMpuFatalError();

// V3: Die frueheren Boot-EZ-Startanzeigen wurden entfernt.
// Der Bootablauf springt nach erfolgreichem MPU-Test direkt ins Hauptmenue.

// Historischer Sued-/Ausrichtungshinweis.
// Im aktuellen Bootablauf wird diese Anzeige nicht mehr automatisch gezeigt,
// weil der fruehere Start-/EZ-Zwischenschritt ersatzlos entfallen ist.
void displayShowSouthAlignPrompt();

// Zeichnet die obere Mode-Leiste.
void drawModeBar(UiMode mode);

// Zeichnet den Signalbereich inkl. Balken, Spannung und Status.
void drawSignalBlock(float volts, float norm, float bestNorm, const char* signalText);

// Zeichnet die Elevationszeile.
void drawElevationRow(float elevationDeg, const char* elevationText);

// Zeichnet die Azimutzeile.
void drawAzimuthRow(const char* azimuthText, const char* azimuthState);

// Zeichnet die untere Infoleiste.
void drawInfoBar(UiMode mode, const char* infoText);

// Führt einen vollständigen bzw. teilweisen Redraw des Displays aus.
//
// Die Implementierung kann intern entscheiden,
// ob wirklich alles neu gezeichnet wird oder nur die geänderten Bereiche.
void displayRender(const DisplayData& data);