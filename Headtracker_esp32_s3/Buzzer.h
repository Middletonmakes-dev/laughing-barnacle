// Buzzer.h - LEDC PWM tone pattern player for passive piezo buzzer
#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include "HtConfig.h"

// Predefined tone patterns
enum class BuzzerPattern : uint8_t {
  STARTUP_COMPLETE,    // Rising triple beep: 1500, 2000, 2500 Hz
  GYRO_CALIBRATED,     // Rising double: 1500 -> 2500 Hz
  RECENTER,            // Single beep: 2000 Hz
  ENTER_GAIN,          // Double beep: 2000, 2000 Hz
  GAIN_INCREASE,       // Short rising tone (pitch varies with gain)
  GAIN_DECREASE,       // Short falling tone (pitch varies with gain)
  SAVE_EXIT_GAIN,      // Beep-pause-long: 2000, pause, 2500 Hz
  ENTER_WIFI,          // Rising triple: 1500, 2000, 2500 Hz
  EXIT_WIFI,           // Falling double: 2500, 1500 Hz
  LOW_BATTERY          // Double low: 800, 800 Hz
};

// Single tone step in a pattern
struct ToneStep {
  uint16_t frequency;  // Hz (0 = silence/pause)
  uint16_t duration;   // ms
};

class Buzzer {
public:
  Buzzer();

  // Initialize LEDC channel for buzzer
  void begin();

  // Play a predefined pattern (non-blocking, queued)
  void play(BuzzerPattern pattern);

  // Play a single tone at given frequency for gain feedback
  void playGainTone(float currentGain);

  // Update buzzer state - call from Core 0 loop
  // Handles non-blocking pattern playback
  void update();

  // Check if currently playing
  bool isPlaying() const;

  // Enable/disable buzzer (mute)
  void setEnabled(bool enabled);

private:
  bool enabled;
  bool playing;

  // Pattern playback state
  const ToneStep* currentPattern;
  uint8_t patternLength;
  uint8_t patternIndex;
  uint32_t stepStartTime;

  // Predefined patterns (stored in flash)
  static const ToneStep PAT_STARTUP[];
  static const ToneStep PAT_GYRO_CAL[];
  static const ToneStep PAT_RECENTER[];
  static const ToneStep PAT_ENTER_GAIN[];
  static const ToneStep PAT_SAVE_EXIT[];
  static const ToneStep PAT_ENTER_WIFI[];
  static const ToneStep PAT_EXIT_WIFI[];
  static const ToneStep PAT_LOW_BATTERY[];

  void startTone(uint16_t frequency);
  void stopTone();
  void startPattern(const ToneStep* pattern, uint8_t length);
};

#endif // BUZZER_H
