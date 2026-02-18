// ElrsTx.h - CRSF/ELRS TX communication handler
// Adapted for ESP32-S3: configurable UART pins via HardwareSerial
#ifndef ELRS_TX_H
#define ELRS_TX_H

#include <Arduino.h>
#include "HtConfig.h"

class ElrsTx {
public:
  ElrsTx();

  // Initialize CRSF serial port on ESP32 configurable pins
  void begin();

  // Set RC channel value in microseconds (1000-2000)
  void setChannel(int channel, int value_us);

  // Set all channels to center (1500 us)
  void setAllCenter();

  // Build and send CRSF RC_CHANNELS_PACKED frame
  void sendChannels();

  // Optional: Send ELRS status request
  void sendStatusRequest();

  // Process any incoming CRSF data (for debugging)
  void processIncoming();

private:
  int rcChannels[16];         // RC channel values in microseconds
  uint8_t frameBuffer[26];    // CRSF frame buffer

  // Convert microseconds to CRSF ticks (0-2047)
  uint16_t usToCrsfTicks(int us);

  // Calculate CRC-8 DVB-S2 for CRSF
  uint8_t calculateCrc8(const uint8_t *data, size_t len);

  // Build RC payload using Devana's bit/byte ordering
  void buildRcPayload(uint8_t payload[22]);

  // Build complete CRSF frame
  void buildRcFrame();
};

#endif // ELRS_TX_H
