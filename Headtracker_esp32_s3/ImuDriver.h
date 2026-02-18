// ImuDriver.h - QMI8658C IMU abstraction layer
// Replaces LSM6DS3 from XIAO build
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include "HtConfig.h"

class ImuDriver {
public:
  ImuDriver();

  // Initialize QMI8658 on I2C bus
  // Call Wire.begin() before this
  bool begin();

  // Read accelerometer and gyroscope data
  // Accel: g's, Gyro: deg/s (matching XIAO interface)
  bool read(float &gx, float &gy, float &gz,
            float &ax, float &ay, float &az);

  // Check if IMU is responding
  bool isConnected();

private:
  // QMI8658 register addresses
  static const uint8_t REG_WHO_AM_I    = 0x00;
  static const uint8_t REG_CTRL1       = 0x02;  // SPI/sensor enable
  static const uint8_t REG_CTRL2       = 0x03;  // Accelerometer settings
  static const uint8_t REG_CTRL3       = 0x04;  // Gyroscope settings
  static const uint8_t REG_CTRL5       = 0x06;  // Low-pass filter
  static const uint8_t REG_CTRL7       = 0x08;  // Enable sensors
  static const uint8_t REG_AX_L        = 0x35;  // Accel X low byte
  static const uint8_t REG_GX_L        = 0x3B;  // Gyro X low byte

  // Expected WHO_AM_I value
  static const uint8_t QMI8658_WHO_AM_I = 0x05;

  // Scale factors based on configured ranges
  float accelScale;  // Convert raw to g's
  float gyroScale;   // Convert raw to deg/s

  // I2C helpers
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t count);
};

#endif // IMU_DRIVER_H
