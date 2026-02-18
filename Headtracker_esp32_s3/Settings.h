// Settings.h - NVS persistent settings management
#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <Preferences.h>
#include "HtConfig.h"

struct HeadtrackerSettings {
  // Quick Settings (adjustable via touch in gain mode)
  float panGain;
  float tiltGain;

  // Channel mapping
  uint8_t panChannel;
  uint8_t tiltChannel;

  // Inversion
  bool panInverted;
  bool tiltInverted;

  // Deadband
  float deadbandDegrees;

  // Angle limits (servo protection)
  float maxPanDegrees;
  float maxTiltDegrees;

  // Display
  uint8_t brightness;

  // Buzzer
  bool buzzerEnabled;

  // Madgwick filter tuning
  float madgwickBeta;

  // Validation
  uint32_t magic;
};

struct GyroOffsets {
  float ox, oy, oz;
  uint32_t magic;
};

class Settings {
public:
  Settings();

  // Load settings from NVS (or defaults if invalid)
  void begin();

  // Get current settings (read-only reference)
  const HeadtrackerSettings& get() const;

  // Get mutable reference for modification
  HeadtrackerSettings& getMutable();

  // Save current settings to NVS
  void save();

  // Reset to factory defaults
  void resetToDefaults();

  // Gyro offset persistence
  bool loadGyroOffsets(float &ox, float &oy, float &oz);
  void saveGyroOffsets(float ox, float oy, float oz);

  // Adjust gain values (clamped to valid range)
  void adjustPanGain(float delta);
  void adjustTiltGain(float delta);

private:
  HeadtrackerSettings settings;
  Preferences prefs;

  void loadDefaults();
  bool validateSettings();
};

#endif // SETTINGS_H
