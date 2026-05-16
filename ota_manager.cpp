/*
  SatAlign V3 - OTA-Update
  ------------------------------------------------------------
  Initialisiert ArduinoOTA mit Hostname und Passwort. OTA ist fuer Tests und
  Weiterentwicklung wichtig; Web-UI und Motorlogik sind davon getrennt.
*/

/*
  SatAlign ESP32 - ota_manager.cpp
  ---------------------------------------------------------------------------
  ArduinoOTA-Unterstuetzung fuer Uploads ueber WLAN. OTA ist fuer den aktuellen
  Projektstand weiterhin eingebaut und wird zyklisch ueber otaHandle() bedient.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "ota_manager.h"
#include "wifi_manager.h"
#include "wifi_config.h"
#include "secrets.h"

namespace {
  bool otaReady = false;
  bool lastWifiState = false;
  bool waitingNoticePrinted = false;
}

void otaInit() {
  // OTA-Passwort fuer Arduino-IDE / Netzwerk-Port.
  // Der Nutzer verwendet dieses Passwort bereits in der Praxis.
  ArduinoOTA.setPassword(OTA_PASSWORD);

  // V3: Standard-OTA-Port explizit setzen.
  // ArduinoOTA nutzt normalerweise 3232 automatisch. Die explizite Angabe
  // macht die Diagnose im Serial Monitor eindeutiger und verhindert, dass
  // spaeter unklar ist, auf welchem Port der OTA-Dienst erwartet wird.
  ArduinoOTA.setPort(3232);

  // Sichtbarer OTA-Hostname im Netzwerk.
  ArduinoOTA.setHostname(WIFI_HOSTNAME);

  // Startmeldung
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "Sketch" : "Filesystem";
    Serial.println();
    Serial.print("OTA Start: ");
    Serial.println(type);
  });

  // Fortschrittsanzeige
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static int lastPercent = -1;
    const int percent = (total == 0) ? 0 : (int)((progress * 100U) / total);
    if (percent != lastPercent) {
      lastPercent = percent;
      Serial.print("OTA Fortschritt: ");
      Serial.print(percent);
      Serial.println("%");
    }
  });

  // Erfolgsfall
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA Ende. Neustart folgt.");
  });

  // Fehlerfall
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print("OTA Fehler: ");
    if (error == OTA_AUTH_ERROR) Serial.println("Auth fehlgeschlagen");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin fehlgeschlagen");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect fehlgeschlagen");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive fehlgeschlagen");
    else if (error == OTA_END_ERROR) Serial.println("End fehlgeschlagen");
    else Serial.println("Unbekannter Fehler");
  });

  // begin() kann schon jetzt aufgerufen werden; OTA wird dann aktiv,
  // sobald WLAN verbunden ist.
  ArduinoOTA.begin();
  otaReady = true;

  Serial.println("OTA initialisiert. Passwortschutz aktiv.");
  Serial.println("OTA Port V3: 3232");
  Serial.print("OTA Hostname V3: ");
  Serial.println(WIFI_HOSTNAME);
  if (wifiIsConnected()) {
    Serial.print("OTA erreichbar unter IP: ");
    Serial.println(WiFi.localIP());
    waitingNoticePrinted = false;
  } else {
    Serial.println("OTA wartet auf WLAN-Verbindung.");
    waitingNoticePrinted = true;
  }
}

void otaLoop() {
  if (!otaReady) {
    return;
  }

  const bool wifiNow = wifiIsConnected();

  // Statuswechsel WLAN -> OTA-Hinweis sauber seriell ausgeben.
  if (wifiNow != lastWifiState) {
    lastWifiState = wifiNow;

    if (wifiNow) {
      Serial.print("OTA bereit unter IP: ");
      Serial.println(WiFi.localIP());
      waitingNoticePrinted = false;
    } else {
      Serial.println("OTA pausiert: kein WLAN.");
      waitingNoticePrinted = true;
    }
  }

  if (!wifiNow) {
    if (!waitingNoticePrinted) {
      Serial.println("OTA wartet auf WLAN-Verbindung.");
      waitingNoticePrinted = true;
    }
    return;
  }

  ArduinoOTA.handle();
}

bool otaIsReady() {
  return otaReady;
}
