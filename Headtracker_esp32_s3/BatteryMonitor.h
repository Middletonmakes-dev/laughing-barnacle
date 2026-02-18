// BatteryMonitor.h - ADC voltage monitoring for VSYS rail
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include "HtConfig.h"

class BatteryMonitor {
public:
  BatteryMonitor();

  // Initialize ADC pin
  void begin();

  // Read and update voltage (call at 1Hz)
  void update();

  // Get smoothed voltage reading
  float getVoltage() const;

  // Get estimated percentage (rough, based on 5V supply)
  int getPercentage() const;

  // Check if voltage is below warning threshold
  bool isLow() const;

private:
  float voltage;
  float smoothedVoltage;

  // Averaging
  static const int AVG_SAMPLES = 8;
  float samples[AVG_SAMPLES];
  int sampleIndex;
  bool bufferFull;

  static const float LOW_VOLTAGE_THRESHOLD;
};

#endif // BATTERY_MONITOR_H
