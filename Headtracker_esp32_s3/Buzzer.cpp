// Buzzer.cpp - LEDC PWM tone pattern player implementation
#include "Buzzer.h"

// ========== Predefined Tone Patterns ==========

// Startup complete: rising triple beep
const ToneStep Buzzer::PAT_STARTUP[] = {
  {1500, 80}, {0, 30}, {2000, 80}, {0, 30}, {2500, 80}
};

// Gyro calibrated: rising double
const ToneStep Buzzer::PAT_GYRO_CAL[] = {
  {1500, 100}, {0, 30}, {2500, 150}
};

// Recenter: single beep
const ToneStep Buzzer::PAT_RECENTER[] = {
  {2000, 100}
};

// Enter gain mode: double beep
const ToneStep Buzzer::PAT_ENTER_GAIN[] = {
  {2000, 80}, {0, 50}, {2000, 80}
};

// Save + exit gain: beep-pause-long
const ToneStep Buzzer::PAT_SAVE_EXIT[] = {
  {2000, 80}, {0, 200}, {2500, 300}
};

// Enter WiFi: rising triple
const ToneStep Buzzer::PAT_ENTER_WIFI[] = {
  {1500, 100}, {0, 50}, {2000, 100}, {0, 50}, {2500, 100}
};

// Exit WiFi: falling double
const ToneStep Buzzer::PAT_EXIT_WIFI[] = {
  {2500, 100}, {0, 50}, {1500, 100}
};

// Low battery: double low
const ToneStep Buzzer::PAT_LOW_BATTERY[] = {
  {800, 200}, {0, 100}, {800, 200}
};

// ========== Implementation ==========

Buzzer::Buzzer()
  : enabled(true), playing(false),
    currentPattern(nullptr), patternLength(0),
    patternIndex(0), stepStartTime(0) {
}

void Buzzer::begin() {
  // Configure LEDC for buzzer output (v2.x API: channel-based)
  ledcSetup(HtConfig::BUZZER_LEDC_CHANNEL, 2000, 8);  // Channel 0, 2kHz, 8-bit
  ledcAttachPin(HtConfig::BUZZER_PIN, HtConfig::BUZZER_LEDC_CHANNEL);
  ledcWrite(HtConfig::BUZZER_LEDC_CHANNEL, 0);  // Start silent
  Serial.println("Buzzer initialized on GPIO" + String(HtConfig::BUZZER_PIN));
}

void Buzzer::play(BuzzerPattern pattern) {
  if (!enabled) return;

  switch (pattern) {
    case BuzzerPattern::STARTUP_COMPLETE:
      startPattern(PAT_STARTUP, 5);
      break;
    case BuzzerPattern::GYRO_CALIBRATED:
      startPattern(PAT_GYRO_CAL, 3);
      break;
    case BuzzerPattern::RECENTER:
      startPattern(PAT_RECENTER, 1);
      break;
    case BuzzerPattern::ENTER_GAIN:
      startPattern(PAT_ENTER_GAIN, 3);
      break;
    case BuzzerPattern::SAVE_EXIT_GAIN:
      startPattern(PAT_SAVE_EXIT, 3);
      break;
    case BuzzerPattern::ENTER_WIFI:
      startPattern(PAT_ENTER_WIFI, 5);
      break;
    case BuzzerPattern::EXIT_WIFI:
      startPattern(PAT_EXIT_WIFI, 3);
      break;
    case BuzzerPattern::LOW_BATTERY:
      startPattern(PAT_LOW_BATTERY, 3);
      break;
    default:
      break;
  }
}

void Buzzer::playGainTone(float currentGain) {
  if (!enabled) return;

  // Frequency proportional to gain value: 500 + (gain * 1000) Hz
  uint16_t freq = (uint16_t)(500.0f + currentGain * 1000.0f);
  if (freq > 4000) freq = 4000;

  startTone(freq);
  stepStartTime = millis();
  playing = true;
  // Will auto-stop after 50ms in update()
  currentPattern = nullptr;
  patternLength = 0;
  patternIndex = 0;
}

void Buzzer::update() {
  if (!playing) return;

  uint32_t now = millis();

  // Single gain tone mode (no pattern)
  if (currentPattern == nullptr) {
    if (now - stepStartTime >= 50) {
      stopTone();
      playing = false;
    }
    return;
  }

  // Pattern playback mode
  if (patternIndex >= patternLength) {
    stopTone();
    playing = false;
    return;
  }

  const ToneStep& step = currentPattern[patternIndex];

  if (now - stepStartTime >= step.duration) {
    // Move to next step
    patternIndex++;
    if (patternIndex >= patternLength) {
      stopTone();
      playing = false;
      return;
    }

    // Start next step
    stepStartTime = now;
    const ToneStep& nextStep = currentPattern[patternIndex];
    if (nextStep.frequency > 0) {
      startTone(nextStep.frequency);
    } else {
      stopTone();
    }
  }
}

bool Buzzer::isPlaying() const {
  return playing;
}

void Buzzer::setEnabled(bool en) {
  enabled = en;
  if (!en) {
    stopTone();
    playing = false;
  }
}

// ========== Private ==========

void Buzzer::startTone(uint16_t frequency) {
  ledcWriteTone(HtConfig::BUZZER_LEDC_CHANNEL, frequency);
  ledcWrite(HtConfig::BUZZER_LEDC_CHANNEL, 128);  // 50% duty cycle
}

void Buzzer::stopTone() {
  ledcWrite(HtConfig::BUZZER_LEDC_CHANNEL, 0);
}

void Buzzer::startPattern(const ToneStep* pattern, uint8_t length) {
  currentPattern = pattern;
  patternLength = length;
  patternIndex = 0;
  stepStartTime = millis();
  playing = true;

  // Start first step immediately
  if (length > 0 && pattern[0].frequency > 0) {
    startTone(pattern[0].frequency);
  }
}
