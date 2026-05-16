/*
  SatAlign V3 - Webserver-Schnittstelle
  ------------------------------------------------------------
  Startet und bedient die Web-UI. Die eigentliche Projektlogik liegt in
  live_runtime.cpp; der Webserver reicht nur Befehle weiter und zeigt Status.
*/

#pragma once

// =====================================================
// web_server.h
// =====================================================
//
// Kommentarstand: V3
//
// Dieses Modul kapselt den eingebauten HTTP-Webserver des ESP32.
//
// Aufgaben des Webservers im Projekt:
// - Ausliefern der Browser-Oberfläche
// - Entgegennehmen von Bedienbefehlen aus dem Browser
// - Weiterreichen dieser Befehle an die zentrale Live-Runtime
// - Anzeigen aktueller Zustände wie Modus, AUTO-Phase,
//   Azimut-Status, Elevations-Status und allgemeine Info-Texte
//
// WICHTIG:
// Die eigentliche Fachlogik liegt NICHT hier,
// sondern in live_runtime.cpp.
// Dieses Modul stellt nur die Web-Schnittstelle bereit.
//
// Architektur:
// web_server.cpp   -> HTTP, HTML, CSS, Buttons, Routen und JSON-APIs
// live_runtime.cpp -> Wahrheit ueber aktuellen Zustand und Befehle
//
// Wichtig fuer spaetere Wartung:
// - web_server.cpp darf die Runtime nur fragen oder Befehle weitergeben.
// - web_server.cpp soll keine eigene AUTO-, RF-, Sensor- oder Motorlogik enthalten.
// - Wenn ein Button nicht bedienbar sein darf, muss das im Web optisch und technisch
//   als gesperrter Button umgesetzt werden, nicht nur als Hinweistext.
//
// Dadurch bleibt die Struktur sauber:
// - Der Browser bedient nicht direkt Motoren oder Sensoren
// - sondern immer nur die zentrale Runtime-Logik.

// Initialisiert den eingebauten HTTP-Webserver.
//
// Typische Aufgaben in der Implementierung:
// - HTTP-Routen registrieren, z. B.:
//   - "/"
//   - "/mode/manual"
//   - "/mode/auto"
//   - "/az/east"
//   - "/az/west"
//   - "/el/up"
//   - "/el/down"
// - Handler-Funktionen mit diesen Routen verbinden
// - Server tatsächlich starten
//
// Diese Funktion wird genau einmal in setup() aufgerufen.
//
// WICHTIG:
// Vor dem Aufruf sollte das WLAN bereits initialisiert sein,
// damit die Weboberfläche später im Netzwerk erreichbar ist.
void webServerInit();

// Bearbeitet eingehende HTTP-Anfragen.
//
// Diese Funktion muss zyklisch in loop() aufgerufen werden,
// damit der Webserver:
// - neue Browseranfragen annimmt
// - Button-Klicks verarbeitet
// - Statusseiten ausliefert
// - Weiterleitungen sauber beantwortet
//
// Ohne den regelmäßigen Aufruf von webServerLoop() würde
// die Weboberfläche zwar formal existieren, aber nicht
// zuverlässig reagieren.
//
// Typischer Einsatz:
// - einmal pro loop()-Durchlauf im Hauptsketch
void webServerLoop();
