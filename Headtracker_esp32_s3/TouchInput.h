// TouchInput.h - CST816S capacitive touch gesture engine
#ifndef TOUCH_INPUT_H
#define TOUCH_INPUT_H

#include <Arduino.h>
#include <Wire.h>
#include "HtConfig.h"

// Gesture types detected by the touch engine
enum class TouchGesture : uint8_t {
  NONE = 0,
  DOUBLE_TAP,
  LONG_PRESS_SHORT,  // 1.5s - enter/exit gain mode
  LONG_PRESS_LONG,   // 3.0s - enter/exit WiFi mode
  SWIPE_UP,
  SWIPE_DOWN,
  SWIPE_LEFT,
  SWIPE_RIGHT
};

class TouchInput {
public:
  TouchInput();

  // Initialize CST816S on shared I2C bus
  bool begin();

  // Poll touch state - call at 50Hz from Core 0
  // Returns detected gesture (if any)
  TouchGesture update();

  // Check if screen is currently being touched
  bool isTouching() const;

  // Get last touch position (0-240 range)
  void getPosition(int &x, int &y) const;

private:
  // CST816S registers
  static const uint8_t REG_GESTURE_ID  = 0x01;
  static const uint8_t REG_FINGER_NUM  = 0x02;
  static const uint8_t REG_XPOS_H      = 0x03;
  static const uint8_t REG_XPOS_L      = 0x04;
  static const uint8_t REG_YPOS_H      = 0x05;
  static const uint8_t REG_YPOS_L      = 0x06;
  static const uint8_t REG_CHIP_ID     = 0xA7;
  static const uint8_t REG_MOTION_MASK = 0xEC;
  static const uint8_t REG_IRQ_CTL     = 0xFA;
  static const uint8_t REG_DIS_AUTOSLEEP = 0xFE;

  // CST816S gesture IDs from hardware
  static const uint8_t GEST_NONE       = 0x00;
  static const uint8_t GEST_SWIPE_UP   = 0x01;
  static const uint8_t GEST_SWIPE_DOWN = 0x02;
  static const uint8_t GEST_SWIPE_LEFT = 0x03;
  static const uint8_t GEST_SWIPE_RIGHT = 0x04;
  static const uint8_t GEST_SINGLE_TAP = 0x05;
  static const uint8_t GEST_DOUBLE_TAP = 0x0B;
  static const uint8_t GEST_LONG_PRESS = 0x0C;

  // Touch state tracking
  bool touching;
  int touchX, touchY;
  uint32_t touchStartTime;
  uint32_t lastTapTime;
  bool longPressShortFired;  // 1.5s threshold already fired
  bool longPressLongFired;   // 3.0s threshold already fired

  // Gesture de-duplication (CST816S latches gesture ID in register)
  uint8_t lastGestureId;
  bool gestureConsumed;

  // I2C helpers
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
};

#endif // TOUCH_INPUT_H
