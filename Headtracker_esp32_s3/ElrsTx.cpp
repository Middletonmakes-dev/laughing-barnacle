// ElrsTx.cpp - Implementation of CRSF/ELRS TX communication
// Adapted for ESP32-S3: uses HardwareSerial1 on configurable GPIO pins
#include "ElrsTx.h"

// ELRS status request frame (optional, from Buddy log analysis)
static const uint8_t CRSF_ELRS_STATUS_REQ[] = {
  0xC8, 0x06, 0x2D, 0xEE, 0xEF, 0x00, 0x00, 0xCB
};

ElrsTx::ElrsTx() {
  // Initialize all channels to center position
  for (int i = 0; i < 16; i++) {
    rcChannels[i] = HtConfig::RC_CENTER_US;
  }
}

void ElrsTx::begin() {
  // ESP32-S3: Serial1 with configurable TX/RX pins
  Serial1.begin(HtConfig::CRSF_BAUD, SERIAL_8N1, HtConfig::CRSF_RX_PIN, HtConfig::CRSF_TX_PIN);
}

void ElrsTx::setChannel(int channel, int value_us) {
  if (channel >= 0 && channel < 16) {
    // Clamp to valid range
    if (value_us < HtConfig::RC_MIN_US) value_us = HtConfig::RC_MIN_US;
    if (value_us > HtConfig::RC_MAX_US) value_us = HtConfig::RC_MAX_US;
    rcChannels[channel] = value_us;
  }
}

void ElrsTx::setAllCenter() {
  for (int i = 0; i < 16; i++) {
    rcChannels[i] = HtConfig::RC_CENTER_US;
  }
}

uint16_t ElrsTx::usToCrsfTicks(int us) {
  // Map 1000-2000 us to 0-2047 CRSF ticks
  // Formula: ticks = (us - 1500) * 8/5 + 992
  float ticks = (float)(us - 1500) * 8.0f / 5.0f + 992.0f;

  if (ticks < 0) ticks = 0;
  if (ticks > 2047) ticks = 2047;

  return (uint16_t)(ticks + 0.5f);
}

uint8_t ElrsTx::calculateCrc8(const uint8_t *data, size_t len) {
  // CRC-8 DVB-S2 polynomial 0xD5
  uint8_t crc = 0x00;

  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

void ElrsTx::buildRcPayload(uint8_t payload[22]) {
  // Build 176 bits (16 channels x 11 bits each)
  // Using Devana's bit/byte ordering: reverse channel order, MSB-first per channel

  uint8_t bits[176];
  int bitIndex = 0;

  // Reverse channel order (15 down to 0)
  for (int chIdx = 15; chIdx >= 0; chIdx--) {
    uint16_t ticks = usToCrsfTicks(rcChannels[chIdx]);

    // Pack 11 bits MSB-first
    for (int b = 10; b >= 0; b--) {
      bits[bitIndex++] = (ticks >> b) & 0x01;
    }
  }

  // Pack bits into bytes
  uint8_t temp[22];
  for (int i = 0; i < 22; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
      v = (v << 1) | bits[i * 8 + b];
    }
    temp[i] = v;
  }

  // Final payload is reversed byte order
  for (int i = 0; i < 22; i++) {
    payload[i] = temp[21 - i];
  }
}

void ElrsTx::buildRcFrame() {
  frameBuffer[0] = 0xC8;  // CRSF sync byte / device address
  frameBuffer[1] = 24;    // Frame length: type(1) + payload(22) + crc(1)
  frameBuffer[2] = 0x16;  // Type: RC_CHANNELS_PACKED

  uint8_t payload[22];
  buildRcPayload(payload);
  memcpy(&frameBuffer[3], payload, 22);

  // Calculate CRC over type + payload
  frameBuffer[25] = calculateCrc8(&frameBuffer[2], 23);
}

void ElrsTx::sendChannels() {
  buildRcFrame();
  Serial1.write(frameBuffer, 26);
  Serial1.flush();
}

void ElrsTx::sendStatusRequest() {
  Serial1.write(CRSF_ELRS_STATUS_REQ, sizeof(CRSF_ELRS_STATUS_REQ));
  Serial1.flush();
}

void ElrsTx::processIncoming() {
  // Read and discard incoming CRSF data
  while (Serial1.available() > 0) {
    Serial1.read();
  }
}
