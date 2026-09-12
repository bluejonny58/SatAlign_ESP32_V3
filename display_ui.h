/*
  SatAlign V3 - TFT-Anzeige API
  ------------------------------------------------------------
  Deklariert die Anzeige-Funktionen fuer Runtime und Hauptsketch.
  Das TFT zeigt Status; es entscheidet nicht selbst ueber Motorlogik.
*/

#pragma once

#include <Arduino.h>

enum UiMode {
  UI_MODE_MENU = 0,
  UI_MODE_CENTER_ALIGN,
  UI_MODE_MANUAL,
  UI_MODE_AUTO,
  UI_MODE_SEARCH,
  UI_MODE_WARN
};

struct DisplayData {
  UiMode mode;

  float signalVolts;
  float signalNorm;
  float signalBestNorm;
  const char* signalText;

  float elevationDeg;
  const char* elevationText;

  const char* azimuthText;
  const char* azimuthState;

  const char* infoText;
};

void displayInit();

void displayShowSplash();

/*
  Zeigt während setup() einen einheitlichen Startbildschirm.

  Während dieser Anzeige werden absichtlich keine Bedientasten,
  Menüeinträge oder Tastenhinweise dargestellt.

  step:
    Name des gerade laufenden Startschritts.

  detail:
    Zusätzliche Beschreibung des Startschritts.
*/
void displayShowStartupStatus(
  const char* step,
  const char* detail
);

/*
  Zeigt kurz an, dass die Initialisierung vollständig beendet ist.

  Erst nach dieser Anzeige wird aus dem Hauptsketch das normale
  Hauptmenü aufgerufen.
*/
void displayShowSystemReady();

void displayShowMpuBootTestResult(
  bool ok,
  float relativeAngleDeg
);

void displayShowMpuFatalBootError();

void displayShowSouthAlignPrompt();

void drawModeBar(UiMode mode);

void drawSignalBlock(
  float volts,
  float norm,
  float bestNorm,
  const char* signalText
);

void drawElevationRow(
  float elevationDeg,
  const char* elevationText
);

void drawAzimuthRow(
  const char* azimuthText,
  const char* azimuthState
);

void drawInfoBar(
  UiMode mode,
  const char* infoText
);

void displayRender(
  const DisplayData& data
);