// HtConfig.h - Configuration constants and pin definitions for ESP32-S3
// Waveshare ESP32-S3-Touch-LCD-1.28
#ifndef HT_CONFIG_H
#define HT_CONFIG_H

#include <Arduino.h>

namespace HtConfig {

  // ========== Pin Definitions (Waveshare ESP32-S3-Touch-LCD-1.28) ==========

  // I2C bus (shared: QMI8658 IMU + CST816S touch)
  const int I2C_SDA = 6;
  const int I2C_SCL = 7;

  // LCD SPI pins (GC9A01) - matches Setup302_Waveshare_ESP32S3_GC9A01.h
  const int LCD_SCLK = 10;
  const int LCD_MOSI = 11;
  const int LCD_MISO = 12;  // GPIO12 = LCD_MISO (not RST!)
  const int LCD_CS   = 9;
  const int LCD_DC   = 8;
  const int LCD_RST  = 14;  // GPIO14 = LCD_RST (wiki confirmed)
  const int LCD_BL   = 2;   // Backlight PWM via MOSFET

  // Touch controller (CST816S) control pins
  const int TOUCH_RST_PIN = 13;  // GPIO13 = TP_RST
  const int TOUCH_INT_PIN = 5;   // GPIO5  = TP_INT

  // IMU (QMI8658) interrupt pins
  const int IMU_INT1_PIN = 4;    // GPIO4 = QMI8658 INT1
  const int IMU_INT2_PIN = 3;    // GPIO3 = QMI8658 INT2

  // CRSF UART (SH1.0 connector -> EP2 ELRS module)
  const int CRSF_TX_PIN = 15;  // GPIO15 -> EP2 RX
  const int CRSF_RX_PIN = 16;  // GPIO16 <- EP2 TX

  // Buzzer (SH1.0 connector)
  const int BUZZER_PIN = 17;   // GPIO17 -> passive piezo

  // Battery / VSYS ADC
  const int BATTERY_ADC_PIN = 1;  // GPIO1 (200K/100K divider)

  // Debug serial (CH343P USB-UART, directly on board)
  const int DEBUG_TX = 43;
  const int DEBUG_RX = 44;

  // Spare pins on SH1.0 connector
  const int SPARE_GPIO18 = 18;
  const int SPARE_GPIO21 = 21;
  const int SPARE_GPIO33 = 33;

  // ========== I2C Addresses ==========
  const uint8_t IMU_I2C_ADDR   = 0x6B;  // QMI8658C (can be 0x6A or 0x6B)
  const uint8_t TOUCH_I2C_ADDR = 0x15;  // CST816S

  // ========== Serial Configuration ==========
  const uint32_t DEBUG_BAUD = 115200;
  const uint32_t CRSF_BAUD  = 921600;

  // ========== Timing Configuration ==========
  const float    LOOP_PERIOD_MS   = 10.0f;   // 100 Hz tracking loop (Core 1)
  const float    STATE_PERIOD_MS  = 50.0f;   // 20 Hz state output
  const uint32_t DISPLAY_PERIOD_MS = 33;     // ~30 Hz display refresh (Core 0)
  const uint32_t TOUCH_PERIOD_MS   = 20;     // 50 Hz touch polling (Core 0)
  const uint32_t BATTERY_PERIOD_MS = 1000;   // 1 Hz battery ADC (Core 0)
  const uint32_t STARTUP_DELAY_MS  = 4500;   // Wait before initial center (must exceed GYRO_MEDIUM_SAMPLES period)
  const uint32_t SPLASH_MIN_MS     = 5000;   // Minimum splash screen display time (slightly > STARTUP_DELAY)
  const uint32_t WIFI_TIMEOUT_MS   = 300000; // 5 minute WiFi auto-timeout

  // ========== ELRS/CRSF Channel Mapping ==========
  const int CHANNEL_YAW   = 5;   // CH6 (array index 5)
  const int CHANNEL_PITCH = 6;   // CH7 (array index 6)
  const int CHANNEL_ROLL  = 7;   // CH8 (array index 7) - unused but available

  // ========== Orientation Filter Parameters ==========
  const float MADGWICK_BETA     = 0.05f;
  const float SAMPLE_PERIOD_SEC = LOOP_PERIOD_MS / 1000.0f;

  // ========== Gyro Calibration Parameters ==========
  // QMI8658 can have large initial bias (up to ~20 deg/s on some axes).
  // Three-phase calibration: fast -> medium -> slow
  //   Fast:   aggressively find the bias from scratch (or correct stale NVS offsets)
  //   Medium: refine to sub-degree accuracy (critical for yaw/pan drift)
  //   Slow:   fine-track thermal drift during operation
  const float    GYRO_STATIONARY_THRESHOLD = 25.0f;  // deg/s - must exceed worst-case raw bias
  const float    GYRO_STABLE_ERROR_MAX     = 0.5f;   // deg/s - tighter threshold reduces pan drift
  const uint32_t GYRO_STABLE_SAMPLES       = 400;    // 4s at 100Hz - allow medium phase to finish
  const float    GYRO_INITIAL_ALPHA        = 0.05f;  // Phase 1: fast convergence (0-2s)
  const float    GYRO_MEDIUM_ALPHA         = 0.01f;  // Phase 2: medium refinement (2-4s)
  const float    GYRO_NORMAL_ALPHA         = 0.003f; // Phase 3: slow tracking (4s+)
  const uint32_t GYRO_FAST_SAMPLES         = 200;    // 2s of fast convergence at boot
  const uint32_t GYRO_MEDIUM_SAMPLES       = 400;    // 2s more of medium convergence
  const uint32_t GYRO_NVS_SAVE_INTERVAL_MS = 60000;  // Save offsets every 60s

  // ========== RC Channel Limits ==========
  const int   RC_MIN_US      = 988;
  const int   RC_MAX_US      = 2012;
  const int   RC_CENTER_US   = 1500;
  const float RC_SCALE_FACTOR = 512.0f;  // ~9 us/degree

  // ========== Default Settings (overridden by NVS) ==========
  const float DEFAULT_PAN_GAIN       = 1.0f;
  const float DEFAULT_TILT_GAIN      = 1.0f;
  const float DEFAULT_DEADBAND_DEG   = 2.0f;
  const float DEFAULT_MAX_PAN_DEG    = 90.0f;
  const float DEFAULT_MAX_TILT_DEG   = 90.0f;
  const float DEFAULT_MADGWICK_BETA  = 0.05f;
  const uint8_t DEFAULT_BRIGHTNESS   = 128;
  const bool  DEFAULT_PAN_INVERTED   = true;
  const bool  DEFAULT_TILT_INVERTED  = false;
  const bool  DEFAULT_BUZZER_ENABLED = true;
  const uint8_t DEFAULT_PAN_CHANNEL  = 5;   // CH6
  const uint8_t DEFAULT_TILT_CHANNEL = 6;   // CH7

  // Gain adjustment limits
  const float GAIN_MIN  = 0.1f;
  const float GAIN_MAX  = 3.0f;
  const float GAIN_STEP = 0.1f;

  // ========== Angle Limits (servo protection) ==========
  const float MAX_YAW_DEGREES   = 90.0f;
  const float MAX_PITCH_DEGREES = 90.0f;
  const float MAX_ROLL_DEGREES  = 90.0f;

  // ========== Touch Parameters ==========
  const uint32_t TOUCH_MIN_DURATION_MS    = 80;   // Minimum touch to register
  const uint32_t TOUCH_IGNORE_DURATION_MS = 50;   // Ignore phantom touches
  const uint32_t LONG_PRESS_GAIN_MS       = 1500; // Enter/exit gain mode
  const uint32_t LONG_PRESS_WIFI_MS       = 3000; // Enter/exit WiFi mode

  // ========== Display ==========
  const int LCD_WIDTH  = 240;
  const int LCD_HEIGHT = 240;

  // ========== Spirit-level UI mapping ==========
  // Maps board-frame level angles to display X/Y bubble movement.
  // Keep this separate from ELRS pan/tilt mapping.
  const bool  LEVEL_UI_SWAP_AXES = false;
  const float LEVEL_UI_X_SIGN    = 1.0f;
  const float LEVEL_UI_Y_SIGN    = -1.0f;
  const float LEVEL_UI_DEADBAND_DEG = 1.0f;
  const float LEVEL_UI_EDGE_LOCK_ENTER_DEG = 75.0f;
  const float LEVEL_UI_EDGE_LOCK_EXIT_DEG  = 68.0f;

  // ========== LEDC Channels (v2.x API: channel-based) ==========
  const int BUZZER_LEDC_CHANNEL    = 0;
  const int BACKLIGHT_LEDC_CHANNEL = 1;

  // ========== Battery ADC ==========
  // Voltage divider: 200K/100K -> multiply ADC voltage by 3
  const float BATTERY_DIVIDER_RATIO = 3.0f;
  const float BATTERY_ADC_VREF      = 3.3f;
  const int   BATTERY_ADC_RESOLUTION = 4096;

  // ========== WiFi AP Configuration ==========
  const char WIFI_SSID[]     = "HT-Config";
  const char WIFI_PASSWORD[] = "headtrack";
  // IP: 10.0.0.1 (set in WifiPortal)

  // ========== NVS Magic Numbers ==========
  const uint32_t SETTINGS_MAGIC = 0x48545632;  // "HTV2"
  const uint32_t GYRO_MAGIC     = 0x4F464653;  // "OFFS"

  // ========== FreeRTOS Task Config ==========
  const int TRACKING_TASK_CORE     = 1;
  const int TRACKING_TASK_PRIORITY = 5;    // Highest
  const int TRACKING_TASK_STACK    = 4096;

  const int DISPLAY_TASK_CORE     = 0;
  const int DISPLAY_TASK_PRIORITY = 2;
  const int DISPLAY_TASK_STACK    = 8192;

  const int TOUCH_TASK_CORE     = 0;
  const int TOUCH_TASK_PRIORITY = 2;
  const int TOUCH_TASK_STACK    = 4096;

  const int BUZZER_TASK_CORE     = 0;
  const int BUZZER_TASK_PRIORITY = 1;
  const int BUZZER_TASK_STACK    = 2048;

  // ========== Math Constants ==========
  const float DEG2RAD = PI / 180.0f;
  const float RAD2DEG = 180.0f / PI;

  // ========== Backward compat aliases ==========
  const float DEADBAND_DEGREES = DEFAULT_DEADBAND_DEG;
}

#endif // HT_CONFIG_H
