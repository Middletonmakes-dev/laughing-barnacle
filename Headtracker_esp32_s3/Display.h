// Display.h - GC9A01 round display rendering for all screens
#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "HtConfig.h"

// Operating modes (matches state machine in design doc)
enum class DisplayMode : uint8_t {
  BOOT,
  NORMAL,
  GAIN,
  WIFI
};

class Display {
public:
  Display();

  // Initialize display hardware and backlight
  void begin();

  // Set backlight brightness (0-255)
  void setBrightness(uint8_t brightness);

  // Draw boot/calibration screen with progress
  void drawBootScreen(float calibProgress, const char* version);

  // Draw normal tracking screen
  // levelXDeg/levelYDeg: spirit-level offsets in degrees (display frame)
  // calGx/calGy/calGz: calibrated gyro rates (deg/s) for debugging
  // gyroStable: calibration status
  // voltage: power supply voltage
  void drawTrackingScreen(float levelXDeg, float levelYDeg,
                          float calGx, float calGy, float calGz,
                          bool gyroStable, float voltage);

  // Draw gain adjustment screen
  void drawGainScreen(float panGain, float tiltGain,
                      bool panActive);  // which axis is being adjusted

  // Draw WiFi configuration screen
  void drawWifiScreen(const char* ssid, const char* ip);

  // Set current mode (controls which screen draws)
  void setMode(DisplayMode mode);
  DisplayMode getMode() const;

private:
  TFT_eSPI tft;
  TFT_eSprite sprite;  // Double-buffered sprite for flicker-free updates
  DisplayMode currentMode;

  // Screen center
  static const int CX = 120;
  static const int CY = 120;
  static const int RADIUS = 120;

  // Colors
  static const uint16_t COLOR_BG       = TFT_BLACK;
  static const uint16_t COLOR_TEXT      = TFT_WHITE;
  static const uint16_t COLOR_DIM       = 0x7BEF;  // Gray
  static const uint16_t COLOR_CROSSHAIR = 0x4208;  // Dark gray
  static const uint16_t COLOR_DOT       = TFT_CYAN;
  static const uint16_t COLOR_STABLE    = TFT_GREEN;
  static const uint16_t COLOR_UNSTABLE  = TFT_RED;
  static const uint16_t COLOR_WIFI      = TFT_BLUE;
  static const uint16_t COLOR_ACCENT    = TFT_ORANGE;
  static const uint16_t COLOR_BAR_BG    = 0x2104;  // Very dark gray

  // Helper functions
  void drawCrosshair(int cx, int cy, int radius);
  void drawTrackingDot(float panDeg, float tiltDeg, int radius);
  void drawStatusDot(bool gyroStable, DisplayMode mode);
  void drawPowerArc(float voltage);
  void drawCircularMask();
};

#endif // DISPLAY_H
