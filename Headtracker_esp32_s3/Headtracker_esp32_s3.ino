// Headtracker ESP32-S3 firmware
// Waveshare ESP32-S3-Touch-LCD-1.28 + HappyModel EP2 (ELRS TX)
// Version 1.0.0
//
// Dual-core architecture:
//   Core 1: Tracking pipeline (IMU -> GyroCal -> Madgwick -> CRSF TX) @ 100 Hz
//   Core 0: Display, touch, buzzer, battery, WiFi (non-critical tasks)
//
// Board: "ESP32S3 Dev Module"
//   Flash Size: 16MB, PSRAM: OPI PSRAM, USB CDC On Boot: Disabled

#include <Arduino.h>
#include <Wire.h>
#include "HtConfig.h"
#include "OrientationCore.h"
#include "ElrsTx.h"
#include "ImuDriver.h"
#include "Display.h"
#include "TouchInput.h"
#include "Buzzer.h"
#include "Settings.h"
#include "BatteryMonitor.h"
#include "WifiPortal.h"

// ========== Global Objects ==========
ImuDriver imu;
GyroCal gyroCal;
Orientation orientation;
ElrsTx elrsTx;
Display display;
TouchInput touchInput;
Buzzer buzzer;
Settings settings;
BatteryMonitor battery;
WifiPortal wifiPortal;

// ========== Operating Mode State Machine ==========
enum class AppMode : uint8_t {
  BOOT,
  NORMAL,
  GAIN,
  WIFI
};

volatile AppMode currentMode = AppMode::BOOT;

// ========== Shared Tracking Data (Core 1 -> Core 0) ==========
struct SharedTrackingData {
  float pan, tilt;                // Current angles (radians, unwrapped)
  float calGx, calGy, calGz;      // Calibrated gyro (deg/s) for on-screen debugging
  int panChannel_us;              // Current pan microseconds
  int tiltChannel_us;             // Current tilt microseconds
  bool gyroStable;                // Gyro calibration status
  volatile uint32_t frameCount;   // For timing diagnostics
};

SharedTrackingData sharedData = {0, 0, 0, 0, 0, 1500, 1500, false, 0};
SemaphoreHandle_t dataMutex;

// ========== I2C Mutex (shared bus: IMU + Touch) ==========
SemaphoreHandle_t i2cMutex;

// ========== State Variables ==========
bool imuReady = false;
bool initialCenterDone = false;
uint32_t lastGyroSaveTime = 0;

// ========== IMU Debug Output ==========
// Set to true to print raw IMU + pan/tilt angles at 10Hz for diagnostics
// Disable for production to reduce serial load on Core 1
static const bool IMU_DEBUG_ENABLED = true;
static uint32_t imuDebugCounter = 0;

// ========== Angle Processing ==========

float clampAngle(float angleRad, float maxDegrees) {
  float angleDeg = angleRad * HtConfig::RAD2DEG;
  if (angleDeg > maxDegrees) angleDeg = maxDegrees;
  else if (angleDeg < -maxDegrees) angleDeg = -maxDegrees;
  return angleDeg * HtConfig::DEG2RAD;
}

float applyDeadband(float angleRad, float deadbandDeg) {
  float angleDeg = angleRad * HtConfig::RAD2DEG;
  if (fabs(angleDeg) < deadbandDeg) return 0.0f;
  return angleRad;
}

int angleToMicroseconds(float angleRad, float gain, bool invert, float deadbandDeg) {
  angleRad = applyDeadband(angleRad, deadbandDeg);

  // Apply gain
  angleRad *= gain;

  float us = HtConfig::RC_CENTER_US + angleRad * HtConfig::RC_SCALE_FACTOR;

  if (invert) {
    us = HtConfig::RC_CENTER_US - (us - HtConfig::RC_CENTER_US);
  }

  if (us < HtConfig::RC_MIN_US) us = HtConfig::RC_MIN_US;
  if (us > HtConfig::RC_MAX_US) us = HtConfig::RC_MAX_US;

  return (int)us;
}

// ========== Core 1: Tracking Task (highest priority, isolated) ==========

void trackingTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS((uint32_t)HtConfig::LOOP_PERIOD_MS);

  Serial.println("[Core 1] Tracking task started");

  while (true) {
    if (imuReady) {
      float gx, gy, gz, ax, ay, az;

      // Read IMU with I2C mutex
      bool readOk = false;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        readOk = imu.read(gx, gy, gz, ax, ay, az);
        xSemaphoreGive(i2cMutex);
      }

      if (readOk) {
        // Save raw values BEFORE calibration for debug output
        float rawGx = gx, rawGy = gy, rawGz = gz;
        float rawAx = ax, rawAy = ay, rawAz = az;

        // Gyro calibration
        gyroCal.apply(gx, gy, gz);
        gyroCal.get(gx, gy, gz);

        // Initial center after startup delay
        // Don't require perfect calibration — just need the filter to settle.
        // Gyro offsets from NVS seed the calibration, so it's usually close.
        // A manual double-tap recenter fixes any residual drift.
        uint32_t now = millis();
        if (!initialCenterDone && now > HtConfig::STARTUP_DELAY_MS) {
          orientation.reLevel();
          initialCenterDone = true;
          Serial.println("[Core 1] Initial re-level completed");
        }

        // Update Madgwick filter
        orientation.update(gx, gy, gz, ax, ay, az);

        // ---- IMU Debug Output (10Hz) ----
        // Shows raw gyro, calibrated gyro (after offset subtraction), offsets, and pan/tilt.
        // When stationary, calibrated values (cal) should be near zero if offsets are correct.
        if (IMU_DEBUG_ENABLED && (++imuDebugCounter % 10 == 0)) {
          float pan_t, tilt_t, unused_t;
          orientation.getAngles(pan_t, tilt_t, unused_t);
          float levelX_t, levelY_t;
          orientation.getLevelVector(levelX_t, levelY_t);
          float ox, oy, oz;
          gyroCal.getOffsets(ox, oy, oz);
          Serial.printf("IMU| raw:%6.1f %6.1f %6.1f | cal:%6.2f %6.2f %6.2f | off:%6.1f %6.1f %6.1f | Pan:%6.1f Tilt:%6.1f | Lx:%6.1f Ly:%6.1f | %s\n",
            rawGx, rawGy, rawGz,
            gx, gy, gz,
            ox, oy, oz,
            pan_t * HtConfig::RAD2DEG, tilt_t * HtConfig::RAD2DEG,
            levelX_t * HtConfig::RAD2DEG, levelY_t * HtConfig::RAD2DEG,
            gyroCal.isStable() ? "STABLE" : "cal...");
        }
      }

      // Get angles and build CRSF output
      float pan, tilt, unused;
      orientation.getAngles(pan, tilt, unused);

      const HeadtrackerSettings& s = settings.get();

      // Clamp angles for servo protection
      pan  = clampAngle(pan, s.maxPanDegrees);
      tilt = clampAngle(tilt, s.maxTiltDegrees);

      // Map to RC channels using current settings
      int panUs  = angleToMicroseconds(pan, s.panGain, s.panInverted, s.deadbandDegrees);
      int tiltUs = angleToMicroseconds(tilt, s.tiltGain, s.tiltInverted, s.deadbandDegrees);

      // Send CRSF frame
      elrsTx.setAllCenter();
      elrsTx.setChannel(s.panChannel, panUs);
      elrsTx.setChannel(s.tiltChannel, tiltUs);
      elrsTx.sendChannels();

      // Update shared data for Core 0
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        sharedData.pan = pan;
        sharedData.tilt = tilt;
        sharedData.calGx = gx;
        sharedData.calGy = gy;
        sharedData.calGz = gz;
        sharedData.panChannel_us = panUs;
        sharedData.tiltChannel_us = tiltUs;
        sharedData.gyroStable = gyroCal.isStable();
        sharedData.frameCount++;
        xSemaphoreGive(dataMutex);
      }

      // Periodic gyro offset save to NVS
      uint32_t now2 = millis();
      if (gyroCal.isStable() && (now2 - lastGyroSaveTime >= HtConfig::GYRO_NVS_SAVE_INTERVAL_MS)) {
        lastGyroSaveTime = now2;
        float ox, oy, oz;
        gyroCal.getOffsets(ox, oy, oz);
        settings.saveGyroOffsets(ox, oy, oz);
      }

      // Process incoming CRSF
      elrsTx.processIncoming();
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// ========== Core 0: Display Task ==========

void displayTask(void* parameter) {
  Serial.println("[Core 0] Display task started");

  while (true) {
    float pan, tilt;
    float calGx, calGy, calGz;
    int panUs, tiltUs;
    bool gyroStable;

    // Read shared data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      pan = sharedData.pan;
      tilt = sharedData.tilt;
      calGx = sharedData.calGx;
      calGy = sharedData.calGy;
      calGz = sharedData.calGz;
      panUs = sharedData.panChannel_us;
      tiltUs = sharedData.tiltChannel_us;
      gyroStable = sharedData.gyroStable;
      xSemaphoreGive(dataMutex);
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    float levelX, levelY;
    orientation.getLevelVector(levelX, levelY);
    float levelXDeg = levelX * HtConfig::RAD2DEG;
    float levelYDeg = levelY * HtConfig::RAD2DEG;

    const HeadtrackerSettings& s = settings.get();

    switch (currentMode) {
      case AppMode::BOOT: {
        // Show splash progress bar over SPLASH_MIN_MS duration
        float progress = min(1.0f, (float)millis() / (float)HtConfig::SPLASH_MIN_MS);
        display.drawBootScreen(progress, "v1.0.0");

        // Transition to NORMAL only after BOTH conditions met:
        // 1. Initial recenter is done (gyro offsets applied)
        // 2. Splash screen minimum time has elapsed
        if (initialCenterDone && millis() >= HtConfig::SPLASH_MIN_MS) {
          currentMode = AppMode::NORMAL;
          display.setMode(DisplayMode::NORMAL);
          buzzer.play(BuzzerPattern::STARTUP_COMPLETE);
        }
        break;
      }

      case AppMode::NORMAL:
        display.drawTrackingScreen(levelXDeg, levelYDeg,
                                    calGx, calGy, calGz,
                                    gyroStable, battery.getVoltage());
        break;

      case AppMode::GAIN:
        display.drawGainScreen(s.panGain, s.tiltGain, true);
        break;

      case AppMode::WIFI:
        display.drawWifiScreen(HtConfig::WIFI_SSID, "10.0.0.1");

        // Update live data for web UI
        if (wifiPortal.isActive()) {
          wifiPortal.setLiveData(panUs, tiltUs, gyroStable, battery.getVoltage());

          // Auto-timeout
          if (wifiPortal.getActiveTime() >= HtConfig::WIFI_TIMEOUT_MS) {
            wifiPortal.stop();
            currentMode = AppMode::NORMAL;
            display.setMode(DisplayMode::NORMAL);
            buzzer.play(BuzzerPattern::EXIT_WIFI);
            Serial.println("WiFi auto-timeout, returning to NORMAL");
          }
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(HtConfig::DISPLAY_PERIOD_MS));
  }
}

// ========== Core 0: Touch + Buzzer + Battery Task ==========

void inputTask(void* parameter) {
  Serial.println("[Core 0] Input task started");

  uint32_t lastBatteryTime = 0;
  uint32_t lastBatteryWarnTime = 0;

  while (true) {
    uint32_t now = millis();

    // ---- Touch polling (with I2C mutex) ----
    TouchGesture gesture = TouchGesture::NONE;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      gesture = touchInput.update();
      xSemaphoreGive(i2cMutex);
    }

    // ---- Process gesture based on current mode ----
    if (gesture != TouchGesture::NONE) {
      switch (currentMode) {
        case AppMode::NORMAL:
          if (gesture == TouchGesture::DOUBLE_TAP) {
            orientation.reLevel();
            buzzer.play(BuzzerPattern::RECENTER);
            Serial.println("Recenter (double-tap)");
          }
          else if (gesture == TouchGesture::LONG_PRESS_LONG) {
            // Enter WiFi mode
            currentMode = AppMode::WIFI;
            display.setMode(DisplayMode::WIFI);
            wifiPortal.begin(&settings);
            buzzer.play(BuzzerPattern::ENTER_WIFI);
            Serial.println("Entering WiFi mode");
          }
          else if (gesture == TouchGesture::LONG_PRESS_SHORT) {
            // Enter Gain mode
            currentMode = AppMode::GAIN;
            display.setMode(DisplayMode::GAIN);
            buzzer.play(BuzzerPattern::ENTER_GAIN);
            Serial.println("Entering Gain mode");
          }
          break;

        case AppMode::GAIN:
          if (gesture == TouchGesture::LONG_PRESS_LONG) {
            // Long hold from NORMAL passed through GAIN -> escalate to WiFi
            currentMode = AppMode::WIFI;
            display.setMode(DisplayMode::WIFI);
            wifiPortal.begin(&settings);
            buzzer.play(BuzzerPattern::ENTER_WIFI);
            Serial.println("Escalated to WiFi mode from Gain");
          }
          else if (gesture == TouchGesture::LONG_PRESS_SHORT) {
            // Save and exit to normal
            settings.save();
            currentMode = AppMode::NORMAL;
            display.setMode(DisplayMode::NORMAL);
            buzzer.play(BuzzerPattern::SAVE_EXIT_GAIN);
            Serial.println("Gain saved, returning to NORMAL");
          }
          else if (gesture == TouchGesture::SWIPE_RIGHT) {
            settings.adjustPanGain(HtConfig::GAIN_STEP);
            buzzer.playGainTone(settings.get().panGain);
            Serial.print("Pan gain: "); Serial.println(settings.get().panGain);
          }
          else if (gesture == TouchGesture::SWIPE_LEFT) {
            settings.adjustPanGain(-HtConfig::GAIN_STEP);
            buzzer.playGainTone(settings.get().panGain);
            Serial.print("Pan gain: "); Serial.println(settings.get().panGain);
          }
          else if (gesture == TouchGesture::SWIPE_UP) {
            settings.adjustTiltGain(HtConfig::GAIN_STEP);
            buzzer.playGainTone(settings.get().tiltGain);
            Serial.print("Tilt gain: "); Serial.println(settings.get().tiltGain);
          }
          else if (gesture == TouchGesture::SWIPE_DOWN) {
            settings.adjustTiltGain(-HtConfig::GAIN_STEP);
            buzzer.playGainTone(settings.get().tiltGain);
            Serial.print("Tilt gain: "); Serial.println(settings.get().tiltGain);
          }
          break;

        case AppMode::WIFI:
          if (gesture == TouchGesture::LONG_PRESS_LONG) {
            // Exit WiFi mode
            wifiPortal.stop();
            currentMode = AppMode::NORMAL;
            display.setMode(DisplayMode::NORMAL);
            buzzer.play(BuzzerPattern::EXIT_WIFI);
            Serial.println("Exiting WiFi mode");
          }
          break;

        default:
          break;
      }
    }

    // ---- Buzzer update (non-blocking pattern playback) ----
    buzzer.update();

    // ---- Battery monitoring (1 Hz) ----
    if (now - lastBatteryTime >= HtConfig::BATTERY_PERIOD_MS) {
      lastBatteryTime = now;
      battery.update();

      // Low battery warning beep every 30 seconds
      if (battery.isLow() && (now - lastBatteryWarnTime >= 30000)) {
        lastBatteryWarnTime = now;
        buzzer.play(BuzzerPattern::LOW_BATTERY);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(HtConfig::TOUCH_PERIOD_MS));
  }
}

// ========== Setup ==========

void setup() {
  // Initialize debug serial (CH343P USB-UART)
  Serial.begin(HtConfig::DEBUG_BAUD);
  delay(500);

  Serial.println();
  Serial.println("=================================");
  Serial.println("ESP32-S3 Headtracker v1.0.0");
  Serial.println("Waveshare ESP32-S3-Touch-LCD-1.28");
  Serial.println("=================================");

  // Create synchronization primitives
  dataMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();

  // Initialize I2C bus (shared: IMU + Touch)
  Wire.begin(HtConfig::I2C_SDA, HtConfig::I2C_SCL);
  Wire.setClock(400000);  // 400 kHz fast mode

  // Load settings from NVS
  Serial.print("Loading settings... ");
  settings.begin();

  // Initialize display
  Serial.print("Initializing display... ");
  display.begin();
  display.setBrightness(settings.get().brightness);
  display.setMode(DisplayMode::BOOT);
  display.drawBootScreen(0.0f, "v1.0.0");
  Serial.println("OK");

  // Initialize IMU
  Serial.print("Initializing QMI8658... ");
  imuReady = imu.begin();
  if (!imuReady) {
    Serial.println("FAILED!");
    display.drawBootScreen(0.0f, "IMU ERROR");
    while (1) { delay(1000); }
  }
  Serial.println("OK");

  // Try to load stored gyro offsets for fast boot
  float ox, oy, oz;
  if (settings.loadGyroOffsets(ox, oy, oz)) {
    gyroCal.setOffsets(ox, oy, oz, false);  // Seed offsets, still verify
    Serial.println("Gyro offsets seeded from NVS");
  }

  // Initialize orientation filter (uses beta from settings)
  Serial.print("Initializing orientation filter... ");
  orientation.begin(HtConfig::SAMPLE_PERIOD_SEC);
  Serial.println("OK");

  // Initialize ELRS CRSF TX
  Serial.print("Initializing CRSF @ 921600 baud (GPIO15 TX, GPIO16 RX)... ");
  elrsTx.begin();
  Serial.println("OK");

  // Initialize touch
  Serial.print("Initializing CST816S touch... ");
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    touchInput.begin();
    xSemaphoreGive(i2cMutex);
  }
  Serial.println("OK");

  // Initialize buzzer
  Serial.print("Initializing buzzer... ");
  buzzer.begin();
  buzzer.setEnabled(settings.get().buzzerEnabled);
  Serial.println("OK");

  // Initialize battery monitor
  Serial.print("Initializing battery monitor... ");
  battery.begin();
  Serial.println("OK");

  Serial.println();
  Serial.println("System ready! Starting dual-core tasks...");
  Serial.println("Calibrating gyro - keep tracker still...");

  lastGyroSaveTime = millis();

  // ========== Launch FreeRTOS Tasks ==========

  // Core 1: Tracking (highest priority, dedicated core)
  xTaskCreatePinnedToCore(
    trackingTask,
    "Tracking",
    HtConfig::TRACKING_TASK_STACK,
    NULL,
    HtConfig::TRACKING_TASK_PRIORITY,
    NULL,
    HtConfig::TRACKING_TASK_CORE
  );

  // Core 0: Display
  xTaskCreatePinnedToCore(
    displayTask,
    "Display",
    HtConfig::DISPLAY_TASK_STACK,
    NULL,
    HtConfig::DISPLAY_TASK_PRIORITY,
    NULL,
    HtConfig::DISPLAY_TASK_CORE
  );

  // Core 0: Input (touch + buzzer + battery)
  xTaskCreatePinnedToCore(
    inputTask,
    "Input",
    HtConfig::TOUCH_TASK_STACK,
    NULL,
    HtConfig::TOUCH_TASK_PRIORITY,
    NULL,
    HtConfig::TOUCH_TASK_CORE
  );
}

// ========== Main Loop (unused - everything runs in FreeRTOS tasks) ==========

void loop() {
  // Nothing here - all work done in pinned tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
