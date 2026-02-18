// Settings.cpp - NVS persistent settings implementation
#include "Settings.h"

Settings::Settings() {
  loadDefaults();
}

void Settings::begin() {
  prefs.begin("ht_config", false);

  // Check magic number
  uint32_t storedMagic = prefs.getUInt("magic", 0);
  if (storedMagic != HtConfig::SETTINGS_MAGIC) {
    Serial.println("NVS: No valid settings found, loading defaults");
    prefs.end();
    loadDefaults();
    save();
    return;
  }

  // Load all settings
  settings.panGain        = prefs.getFloat("panGain", HtConfig::DEFAULT_PAN_GAIN);
  settings.tiltGain       = prefs.getFloat("tiltGain", HtConfig::DEFAULT_TILT_GAIN);
  settings.panChannel     = prefs.getUChar("panCh", HtConfig::DEFAULT_PAN_CHANNEL);
  settings.tiltChannel    = prefs.getUChar("tiltCh", HtConfig::DEFAULT_TILT_CHANNEL);
  settings.panInverted    = prefs.getBool("panInv", HtConfig::DEFAULT_PAN_INVERTED);
  settings.tiltInverted   = prefs.getBool("tiltInv", HtConfig::DEFAULT_TILT_INVERTED);
  settings.deadbandDegrees = prefs.getFloat("deadband", HtConfig::DEFAULT_DEADBAND_DEG);
  settings.maxPanDegrees  = prefs.getFloat("maxPan", HtConfig::DEFAULT_MAX_PAN_DEG);
  settings.maxTiltDegrees = prefs.getFloat("maxTilt", HtConfig::DEFAULT_MAX_TILT_DEG);
  settings.brightness     = prefs.getUChar("bright", HtConfig::DEFAULT_BRIGHTNESS);
  settings.buzzerEnabled  = prefs.getBool("buzzer", HtConfig::DEFAULT_BUZZER_ENABLED);
  settings.madgwickBeta   = prefs.getFloat("mBeta", HtConfig::DEFAULT_MADGWICK_BETA);
  settings.magic          = HtConfig::SETTINGS_MAGIC;

  prefs.end();

  if (!validateSettings()) {
    Serial.println("NVS: Settings validation failed, resetting to defaults");
    loadDefaults();
    save();
    return;
  }

  Serial.println("NVS: Settings loaded successfully");
  Serial.print("  Pan gain: "); Serial.println(settings.panGain);
  Serial.print("  Tilt gain: "); Serial.println(settings.tiltGain);
  Serial.print("  Pan ch: "); Serial.println(settings.panChannel);
  Serial.print("  Tilt ch: "); Serial.println(settings.tiltChannel);
}

const HeadtrackerSettings& Settings::get() const {
  return settings;
}

HeadtrackerSettings& Settings::getMutable() {
  return settings;
}

void Settings::save() {
  prefs.begin("ht_config", false);
  prefs.putUInt("magic", HtConfig::SETTINGS_MAGIC);
  prefs.putFloat("panGain", settings.panGain);
  prefs.putFloat("tiltGain", settings.tiltGain);
  prefs.putUChar("panCh", settings.panChannel);
  prefs.putUChar("tiltCh", settings.tiltChannel);
  prefs.putBool("panInv", settings.panInverted);
  prefs.putBool("tiltInv", settings.tiltInverted);
  prefs.putFloat("deadband", settings.deadbandDegrees);
  prefs.putFloat("maxPan", settings.maxPanDegrees);
  prefs.putFloat("maxTilt", settings.maxTiltDegrees);
  prefs.putUChar("bright", settings.brightness);
  prefs.putBool("buzzer", settings.buzzerEnabled);
  prefs.putFloat("mBeta", settings.madgwickBeta);
  prefs.end();
  Serial.println("NVS: Settings saved");
}

void Settings::resetToDefaults() {
  loadDefaults();
  save();
  Serial.println("NVS: Factory reset complete");
}

bool Settings::loadGyroOffsets(float &ox, float &oy, float &oz) {
  Preferences gyroPrefs;
  gyroPrefs.begin("ht_gyro", true);  // read-only
  uint32_t magic = gyroPrefs.getUInt("magic", 0);
  if (magic != HtConfig::GYRO_MAGIC) {
    gyroPrefs.end();
    return false;
  }
  ox = gyroPrefs.getFloat("ox", 0.0f);
  oy = gyroPrefs.getFloat("oy", 0.0f);
  oz = gyroPrefs.getFloat("oz", 0.0f);
  gyroPrefs.end();

  Serial.print("NVS: Gyro offsets loaded: ");
  Serial.print(ox); Serial.print(", ");
  Serial.print(oy); Serial.print(", ");
  Serial.println(oz);
  return true;
}

void Settings::saveGyroOffsets(float ox, float oy, float oz) {
  Preferences gyroPrefs;
  gyroPrefs.begin("ht_gyro", false);
  gyroPrefs.putUInt("magic", HtConfig::GYRO_MAGIC);
  gyroPrefs.putFloat("ox", ox);
  gyroPrefs.putFloat("oy", oy);
  gyroPrefs.putFloat("oz", oz);
  gyroPrefs.end();
}

void Settings::adjustPanGain(float delta) {
  settings.panGain += delta;
  if (settings.panGain < HtConfig::GAIN_MIN) settings.panGain = HtConfig::GAIN_MIN;
  if (settings.panGain > HtConfig::GAIN_MAX) settings.panGain = HtConfig::GAIN_MAX;
  // Round to 1 decimal place to avoid float drift
  settings.panGain = roundf(settings.panGain * 10.0f) / 10.0f;
}

void Settings::adjustTiltGain(float delta) {
  settings.tiltGain += delta;
  if (settings.tiltGain < HtConfig::GAIN_MIN) settings.tiltGain = HtConfig::GAIN_MIN;
  if (settings.tiltGain > HtConfig::GAIN_MAX) settings.tiltGain = HtConfig::GAIN_MAX;
  settings.tiltGain = roundf(settings.tiltGain * 10.0f) / 10.0f;
}

// ========== Private ==========

void Settings::loadDefaults() {
  settings.panGain        = HtConfig::DEFAULT_PAN_GAIN;
  settings.tiltGain       = HtConfig::DEFAULT_TILT_GAIN;
  settings.panChannel     = HtConfig::DEFAULT_PAN_CHANNEL;
  settings.tiltChannel    = HtConfig::DEFAULT_TILT_CHANNEL;
  settings.panInverted    = HtConfig::DEFAULT_PAN_INVERTED;
  settings.tiltInverted   = HtConfig::DEFAULT_TILT_INVERTED;
  settings.deadbandDegrees = HtConfig::DEFAULT_DEADBAND_DEG;
  settings.maxPanDegrees  = HtConfig::DEFAULT_MAX_PAN_DEG;
  settings.maxTiltDegrees = HtConfig::DEFAULT_MAX_TILT_DEG;
  settings.brightness     = HtConfig::DEFAULT_BRIGHTNESS;
  settings.buzzerEnabled  = HtConfig::DEFAULT_BUZZER_ENABLED;
  settings.madgwickBeta   = HtConfig::DEFAULT_MADGWICK_BETA;
  settings.magic          = HtConfig::SETTINGS_MAGIC;
}

bool Settings::validateSettings() {
  if (settings.panGain < HtConfig::GAIN_MIN || settings.panGain > HtConfig::GAIN_MAX) return false;
  if (settings.tiltGain < HtConfig::GAIN_MIN || settings.tiltGain > HtConfig::GAIN_MAX) return false;
  if (settings.panChannel > 15) return false;
  if (settings.tiltChannel > 15) return false;
  if (settings.deadbandDegrees < 0.0f || settings.deadbandDegrees > 10.0f) return false;
  if (settings.maxPanDegrees < 30.0f || settings.maxPanDegrees > 180.0f) return false;
  if (settings.maxTiltDegrees < 30.0f || settings.maxTiltDegrees > 180.0f) return false;
  if (settings.madgwickBeta < 0.01f || settings.madgwickBeta > 0.5f) return false;
  return true;
}
