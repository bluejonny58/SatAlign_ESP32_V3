/*
  SatAlign V3 - WLAN-Verbindung
  ------------------------------------------------------------
  Baut die WLAN-Verbindung auf und sorgt dafuer, dass Web-UI und OTA im
  lokalen Netz erreichbar werden.
*/

/*
  SatAlign ESP32 - wifi_manager.cpp
  ---------------------------------------------------------------------------
  Baut die WLAN-Verbindung auf und fuehrt einfache Reconnect-Versuche durch.
  Die Zugangsdaten liegen in wifi_config.h.
*/
#include <WiFi.h>
#include <Arduino.h>

#include "wifi_config.h"
#include "wifi_manager.h"

// Initialisiert die WLAN-Verbindung.
//
// Ablauf:
// 1. Station-Mode aktivieren
// 2. Hostname setzen
// 3. Verbindung starten
// 4. für eine begrenzte Zeit auf Verbindung warten
//
// WICHTIG:
// Wenn die Verbindung hier fehlschlägt, läuft das System trotzdem weiter.
// Nur die Netzwerk-/Web-Funktionen stehen dann zunächst nicht zur Verfügung.
void wifiInit() {
  Serial.println();
  Serial.println("Verbinde WLAN...");

  // ESP32 als WLAN-Station betreiben
  WiFi.mode(WIFI_STA);

  // V3: WLAN-Schlafmodus deaktivieren.
  // Hintergrund: Bei einigen ESP32/Router-Kombinationen werden mDNS/OTA-
  // Pakete unzuverlaessig sichtbar, wenn der WLAN-Sleep aktiv ist.
  // Die Web-UI kann dann erreichbar sein, waehrend der Arduino-OTA-Port
  // in der IDE nicht oder nur sporadisch angezeigt wird.
  WiFi.setSleep(false);

  // Sichtbarer Geraetename im Netzwerk
  WiFi.setHostname(WIFI_HOSTNAME);
  Serial.print("WLAN Hostname V3: ");
  Serial.println(WIFI_HOSTNAME);

  // Verbindungsaufbau starten
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempt = 0;

  // Begrenzte Wartezeit auf den Verbindungsaufbau
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempt++;

    // Nach ca. 20 Sekunden abbrechen,
    // damit das System nicht endlos im WLAN-Start hängen bleibt.
    if (attempt > 40) {
      Serial.println();
      Serial.println("WLAN Verbindung fehlgeschlagen.");
      return;
    }
  }

  Serial.println();
  Serial.println("WLAN verbunden.");

  // Zu Diagnosezwecken die erhaltene IP-Adresse ausgeben
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.localIP());
}

// Muss zyklisch aufgerufen werden, um Verbindungsverlust zu erkennen
// und bei Bedarf erneut zu verbinden.
//
// Die Funktion ist bewusst einfach gehalten:
// - solange verbunden -> nichts tun
// - wenn getrennt -> in sinnvollen Abständen reconnecten
void wifiLoop() {
  // Zeitstempel des letzten Reconnect-Versuchs
  static unsigned long lastReconnectAttempt = 0;

  // Solange WLAN steht, ist nichts zu tun
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  // Nicht zu häufig reconnecten
  if (millis() - lastReconnectAttempt < 5000) {
    return;
  }

  lastReconnectAttempt = millis();

  Serial.println("WLAN verloren, reconnect...");

  // Vor neuem Verbindungsaufbau einmal sauber trennen
  WiFi.disconnect();

  // Verbindung neu starten
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// Liefert den aktuellen Verbindungsstatus als bool.
bool wifiIsConnected() {
  return WiFi.status() == WL_CONNECTED;
}