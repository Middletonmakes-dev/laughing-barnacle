// WifiPortal.h - AP mode + AsyncWebServer + REST API for configuration
#ifndef WIFI_PORTAL_H
#define WIFI_PORTAL_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "HtConfig.h"
#include "Settings.h"

class WifiPortal {
public:
  WifiPortal();
  ~WifiPortal();

  // Start WiFi AP and web server (can be called multiple times)
  void begin(Settings* settingsPtr);

  // Stop WiFi AP and web server
  void stop();

  // Check if portal is active
  bool isActive() const;

  // Check if a client is connected
  bool hasClient() const;

  // Get elapsed time since activation (for auto-timeout)
  uint32_t getActiveTime() const;

  // Set live tracking data for the web UI bars
  void setLiveData(int panUs, int tiltUs, bool gyroStable, float voltage);

private:
  AsyncWebServer* server;  // Pointer — recreated on each begin()
  Settings* settings;
  bool active;
  uint32_t startTime;
  uint32_t lastClientActivity;

  // Live data for web UI
  volatile int livePanUs;
  volatile int liveTiltUs;
  volatile bool liveGyroStable;
  volatile float liveVoltage;

  // Setup routes
  void setupRoutes();

  // Generate HTML page (stored in PROGMEM)
  static const char INDEX_HTML[];

  // API handlers
  void handleGetSettings(AsyncWebServerRequest* request);
  void handlePostSettings(AsyncWebServerRequest* request, uint8_t* data, size_t len);
  void handleGetState(AsyncWebServerRequest* request);
  void handleReboot(AsyncWebServerRequest* request);
  void handleDefaults(AsyncWebServerRequest* request);
};

#endif // WIFI_PORTAL_H
