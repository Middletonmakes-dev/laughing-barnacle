// TouchInput.cpp - CST816S capacitive touch gesture engine
#include "TouchInput.h"

TouchInput::TouchInput()
  : touching(false), touchX(0), touchY(0),
    touchStartTime(0), lastTapTime(0),
    longPressShortFired(false), longPressLongFired(false),
    lastGestureId(GEST_NONE), gestureConsumed(false) {
}

bool TouchInput::begin() {
  // Hardware reset the touch controller via TP_RST (GPIO13)
  pinMode(HtConfig::TOUCH_RST_PIN, OUTPUT);
  digitalWrite(HtConfig::TOUCH_RST_PIN, LOW);
  delay(10);
  digitalWrite(HtConfig::TOUCH_RST_PIN, HIGH);
  delay(50);  // Wait for CST816S to boot after reset

  // Configure interrupt pin (optional, for future IRQ-driven reads)
  pinMode(HtConfig::TOUCH_INT_PIN, INPUT);

  // Check chip ID
  uint8_t chipId = readRegister(REG_CHIP_ID);
  if (chipId == 0xFF || chipId == 0x00) {
    Serial.println("CST816S not detected");
    return false;
  }
  Serial.print("CST816S chip ID: 0x");
  Serial.println(chipId, HEX);

  // Disable auto-sleep so touch is always responsive
  writeRegister(REG_DIS_AUTOSLEEP, 0x01);

  // Enable motion mask: continuous report + double-tap + long-press
  writeRegister(REG_MOTION_MASK, 0x07);

  // IRQ control: trigger on touch
  writeRegister(REG_IRQ_CTL, 0x41);

  Serial.println("CST816S initialized");
  return true;
}

TouchGesture TouchInput::update() {
  // Read finger count and gesture
  uint8_t fingerNum = readRegister(REG_FINGER_NUM);
  uint8_t gestureId = readRegister(REG_GESTURE_ID);

  // Read position
  uint8_t xH = readRegister(REG_XPOS_H) & 0x0F;
  uint8_t xL = readRegister(REG_XPOS_L);
  uint8_t yH = readRegister(REG_YPOS_H) & 0x0F;
  uint8_t yL = readRegister(REG_YPOS_L);
  touchX = (xH << 8) | xL;
  touchY = (yH << 8) | yL;

  uint32_t now = millis();
  bool currentlyTouching = (fingerNum > 0);

  // ---- CRITICAL: De-duplicate CST816S gesture register ----
  // The CST816S LATCHES the gesture ID — it keeps returning the same
  // value until a new gesture overwrites it. We must only fire once
  // per unique gesture transition.
  bool newGesture = false;
  if (gestureId != GEST_NONE && gestureId != lastGestureId) {
    newGesture = true;
    gestureConsumed = false;
  }
  // When finger lifts and gesture goes back to NONE, reset for next touch
  if (gestureId == GEST_NONE && lastGestureId != GEST_NONE) {
    gestureConsumed = false;
  }
  lastGestureId = gestureId;

  // Touch just started
  if (currentlyTouching && !touching) {
    touching = true;
    touchStartTime = now;
    longPressShortFired = false;
    longPressLongFired = false;
  }

  // Touch released
  if (!currentlyTouching && touching) {
    touching = false;
    uint32_t duration = now - touchStartTime;

    // Reject phantom touches shorter than minimum duration
    if (duration < HtConfig::TOUCH_IGNORE_DURATION_MS) {
      return TouchGesture::NONE;
    }
  }

  // While touching, check for long press thresholds
  // IMPORTANT: Check 3s FIRST, then 1.5s — both fire on the same hold,
  // the caller decides what to do based on current mode.
  if (touching) {
    uint32_t duration = now - touchStartTime;

    // 3-second long press (WiFi mode) — fires INSTEAD of 1.5s if held long enough
    if (duration >= HtConfig::LONG_PRESS_WIFI_MS && !longPressLongFired) {
      longPressLongFired = true;
      longPressShortFired = true;  // Suppress the short press too
      return TouchGesture::LONG_PRESS_LONG;
    }

    // 1.5-second long press (gain mode)
    if (duration >= HtConfig::LONG_PRESS_GAIN_MS && !longPressShortFired) {
      longPressShortFired = true;
      return TouchGesture::LONG_PRESS_SHORT;
    }
  }

  // Handle hardware-detected gestures — only fire ONCE per gesture
  if (newGesture && !gestureConsumed) {
    gestureConsumed = true;

    switch (gestureId) {
      case GEST_DOUBLE_TAP:
        return TouchGesture::DOUBLE_TAP;

      case GEST_SWIPE_UP:
        return TouchGesture::SWIPE_UP;

      case GEST_SWIPE_DOWN:
        return TouchGesture::SWIPE_DOWN;

      case GEST_SWIPE_LEFT:
        return TouchGesture::SWIPE_LEFT;

      case GEST_SWIPE_RIGHT:
        return TouchGesture::SWIPE_RIGHT;

      default:
        break;
    }
  }

  return TouchGesture::NONE;
}

bool TouchInput::isTouching() const {
  return touching;
}

void TouchInput::getPosition(int &x, int &y) const {
  x = touchX;
  y = touchY;
}

// ========== I2C Helpers ==========

uint8_t TouchInput::readRegister(uint8_t reg) {
  Wire.beginTransmission(HtConfig::TOUCH_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }
  Wire.requestFrom(HtConfig::TOUCH_I2C_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

void TouchInput::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(HtConfig::TOUCH_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
