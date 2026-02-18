// ImuDriver.cpp - QMI8658C IMU driver implementation
#include "ImuDriver.h"

ImuDriver::ImuDriver() : accelScale(0.0f), gyroScale(0.0f) {
}

bool ImuDriver::begin() {
  // Check WHO_AM_I
  uint8_t id = readRegister(REG_WHO_AM_I);
  if (id != QMI8658_WHO_AM_I) {
    Serial.print("QMI8658 WHO_AM_I mismatch: 0x");
    Serial.println(id, HEX);
    return false;
  }

  // CTRL1: Address auto-increment enabled
  writeRegister(REG_CTRL1, 0x60);

  // CTRL2: Accelerometer config
  // Bits [6:4] = full scale: 000 = +/-2g, 001 = +/-4g, 010 = +/-8g, 011 = +/-16g
  // Bits [3:0] = ODR: 0111 = 128Hz (good match for 100Hz loop)
  // Using +/-8g range, 128Hz ODR
  writeRegister(REG_CTRL2, 0x27);  // 8g, 128Hz
  accelScale = 8.0f / 32768.0f;    // +/-8g range

  // CTRL3: Gyroscope config
  // Bits [6:4] = full scale: 000 = +/-16dps, ..., 011 = +/-256dps, 100 = +/-512dps,
  //                           101 = +/-1024dps, 110 = +/-2048dps
  // Bits [3:0] = ODR: 0111 = 128Hz
  // Using +/-2048 deg/s range, 128Hz ODR
  writeRegister(REG_CTRL3, 0x67);  // 2048dps, 128Hz
  gyroScale = 2048.0f / 32768.0f;  // +/-2048 deg/s range

  // CTRL5: Low-pass filter enable for both accel and gyro
  writeRegister(REG_CTRL5, 0x11);  // LPF enabled, moderate bandwidth

  // CTRL7: Enable accelerometer and gyroscope
  writeRegister(REG_CTRL7, 0x03);  // aEN=1, gEN=1

  delay(50); // Wait for sensors to start

  Serial.println("QMI8658 initialized successfully");
  return true;
}

bool ImuDriver::read(float &gx, float &gy, float &gz,
                     float &ax, float &ay, float &az) {
  // Read 12 bytes: 6 bytes accel + 6 bytes gyro
  // Accel starts at REG_AX_L (0x35), Gyro at REG_GX_L (0x3B)
  uint8_t accelBuf[6];
  uint8_t gyroBuf[6];

  readRegisters(REG_AX_L, accelBuf, 6);
  readRegisters(REG_GX_L, gyroBuf, 6);

  // Parse accelerometer (little-endian signed 16-bit)
  int16_t rawAx = (int16_t)((accelBuf[1] << 8) | accelBuf[0]);
  int16_t rawAy = (int16_t)((accelBuf[3] << 8) | accelBuf[2]);
  int16_t rawAz = (int16_t)((accelBuf[5] << 8) | accelBuf[4]);

  // Parse gyroscope (little-endian signed 16-bit)
  int16_t rawGx = (int16_t)((gyroBuf[1] << 8) | gyroBuf[0]);
  int16_t rawGy = (int16_t)((gyroBuf[3] << 8) | gyroBuf[2]);
  int16_t rawGz = (int16_t)((gyroBuf[5] << 8) | gyroBuf[4]);

  // Convert to physical units
  ax = rawAx * accelScale;
  ay = rawAy * accelScale;
  az = rawAz * accelScale;

  gx = rawGx * gyroScale;
  gy = rawGy * gyroScale;
  gz = rawGz * gyroScale;

  // Validate readings
  if (isnan(ax) || isnan(ay) || isnan(az) ||
      isnan(gx) || isnan(gy) || isnan(gz)) {
    return false;
  }

  return true;
}

bool ImuDriver::isConnected() {
  uint8_t id = readRegister(REG_WHO_AM_I);
  return (id == QMI8658_WHO_AM_I);
}

// ========== I2C Helpers ==========

uint8_t ImuDriver::readRegister(uint8_t reg) {
  Wire.beginTransmission(HtConfig::IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(HtConfig::IMU_I2C_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

void ImuDriver::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(HtConfig::IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void ImuDriver::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t count) {
  Wire.beginTransmission(HtConfig::IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(HtConfig::IMU_I2C_ADDR, count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}
