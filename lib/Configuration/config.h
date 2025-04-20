#pragma once

#include <Arduino.h>

#define wifi_ssid "EspDrone"
#define wifi_password "espdrone@123"

extern volatile bool wifi_config_mode;
extern bool serverStarted;

void wifiTask(void *pv);
void startWiFiConfigPortal();
void handleUpdate();
void handleRoot();
void savePID();
void loadPID();
void turnOffWiFi();