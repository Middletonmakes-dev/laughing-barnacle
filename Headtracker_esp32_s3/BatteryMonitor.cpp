// BatteryMonitor.cpp - ADC voltage monitoring implementation
#include "BatteryMonitor.h"

const float BatteryMonitor::LOW_VOLTAGE_THRESHOLD = 3.5f;

BatteryMonitor::BatteryMonitor()
  : voltage(0.0f), smoothedVoltage(0.0f),
    sampleIndex(0), bufferFull(false) {
  for (int i = 0; i < AVG_SAMPLES; i++) {
    samples[i] = 0.0f;
  }
}

void BatteryMonitor::begin() {
  pinMode(HtConfig::BATTERY_ADC_PIN, INPUT);
  analogSetAttenuation(ADC_11db);  // Full 3.3V range

  // Take initial reading
  update();

  Serial.print("Battery monitor initialized, voltage: ");
  Serial.print(smoothedVoltage);
  Serial.println("V");
}

void BatteryMonitor::update() {
  // Read ADC
  int raw = analogRead(HtConfig::BATTERY_ADC_PIN);

  // Convert to actual voltage
  // ADC reads through 200K/100K divider, so actual = ADC_voltage * 3
  voltage = (float)raw / HtConfig::BATTERY_ADC_RESOLUTION
            * HtConfig::BATTERY_ADC_VREF
            * HtConfig::BATTERY_DIVIDER_RATIO;

  // Add to averaging buffer
  samples[sampleIndex] = voltage;
  sampleIndex = (sampleIndex + 1) % AVG_SAMPLES;
  if (sampleIndex == 0) bufferFull = true;

  // Calculate average
  int count = bufferFull ? AVG_SAMPLES : sampleIndex;
  if (count == 0) count = 1;

  float sum = 0.0f;
  for (int i = 0; i < count; i++) {
    sum += samples[i];
  }
  smoothedVoltage = sum / count;
}

float BatteryMonitor::getVoltage() const {
  return smoothedVoltage;
}

int BatteryMonitor::getPercentage() const {
  // Rough percentage for 5V goggle power supply
  // 5.0V = 100%, 4.0V = 50%, 3.5V = 0%
  if (smoothedVoltage >= 5.0f) return 100;
  if (smoothedVoltage <= 3.5f) return 0;

  return (int)((smoothedVoltage - 3.5f) / 1.5f * 100.0f);
}

bool BatteryMonitor::isLow() const {
  return smoothedVoltage < LOW_VOLTAGE_THRESHOLD && smoothedVoltage > 0.5f;
}
