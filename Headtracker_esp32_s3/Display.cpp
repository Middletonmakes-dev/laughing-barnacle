// Display.cpp - GC9A01 round display rendering implementation
#include "Display.h"

Display::Display() : tft(), sprite(&tft), currentMode(DisplayMode::BOOT),
                     levelEdgeLock(false), levelLockXDeg(0.0f), levelLockYDeg(0.0f) {
}

void Display::begin() {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(COLOR_BG);

  // Create full-screen sprite for double buffering
  sprite.createSprite(HtConfig::LCD_WIDTH, HtConfig::LCD_HEIGHT);
  sprite.setTextDatum(MC_DATUM);

  // Initialize backlight via LEDC (v2.x API: channel-based)
  ledcSetup(HtConfig::BACKLIGHT_LEDC_CHANNEL, 5000, 8);  // Channel 1, 5kHz, 8-bit
  ledcAttachPin(HtConfig::LCD_BL, HtConfig::BACKLIGHT_LEDC_CHANNEL);
  setBrightness(HtConfig::DEFAULT_BRIGHTNESS);

  Serial.println("Display initialized");
}

void Display::setBrightness(uint8_t brightness) {
  ledcWrite(HtConfig::BACKLIGHT_LEDC_CHANNEL, brightness);
}

void Display::drawBootScreen(float calibProgress, const char* version) {
  sprite.fillSprite(COLOR_BG);

  // Title
  sprite.setTextColor(COLOR_TEXT);
  sprite.setTextSize(2);
  sprite.drawString("HEADTRACKER", CX, 70);

  // Version
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_DIM);
  sprite.drawString(version, CX, 95);

  // Status text
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_TEXT);
  if (calibProgress >= 1.0f) {
    sprite.drawString("Ready!", CX, 130);
  } else {
    sprite.drawString("Calibrating...", CX, 130);
  }

  // Progress bar
  int barX = 50;
  int barY = 150;
  int barW = 140;
  int barH = 10;

  sprite.drawRect(barX, barY, barW, barH, COLOR_DIM);
  int fillW = (int)(barW * calibProgress);
  if (fillW > barW) fillW = barW;
  if (fillW > 0) {
    sprite.fillRect(barX + 1, barY + 1, fillW - 2, barH - 2, COLOR_STABLE);
  }

  // Instruction
  sprite.setTextColor(COLOR_DIM);
  sprite.drawString("Keep still", CX, 175);

  drawCircularMask();
  sprite.pushSprite(0, 0);
}

void Display::drawTrackingScreen(float levelXDeg, float levelYDeg,
                                  float calGx, float calGy, float calGz,
                                  bool gyroStable, float voltage) {
  sprite.fillSprite(COLOR_BG);

  // Crosshair
  int trackRadius = 90;  // Tracking area radius
  drawCrosshair(CX, CY, trackRadius);

  // Tracking dot
  drawTrackingDot(levelXDeg, levelYDeg, trackRadius);

  // Power arc at top
  drawPowerArc(voltage);

  // Debug values (bottom area)
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_DIM);
  char line1[48];
  snprintf(line1, sizeof(line1), "Lx:%5.1f  Ly:%5.1f", levelXDeg, levelYDeg);
  sprite.drawString(line1, CX, 196);

  char line2[48];
  snprintf(line2, sizeof(line2), "G:%5.1f %5.1f %5.1f", calGx, calGy, calGz);
  sprite.drawString(line2, CX, 210);

  // Status dot and label
  drawStatusDot(gyroStable, DisplayMode::NORMAL);

  drawCircularMask();
  sprite.pushSprite(0, 0);
}

void Display::drawGainScreen(float panGain, float tiltGain, bool panActive) {
  sprite.fillSprite(COLOR_BG);

  // Header
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_DIM);
  sprite.drawString("-- GAIN --", CX, 30);

  // Pan gain
  sprite.setTextSize(2);
  uint16_t panColor = panActive ? COLOR_TEXT : COLOR_DIM;
  sprite.setTextColor(panColor);
  sprite.drawString("PAN", CX, 80);

  char valStr[16];
  snprintf(valStr, sizeof(valStr), "< %.1f >", panGain);
  sprite.setTextColor(COLOR_ACCENT);
  sprite.drawString(valStr, CX, 105);

  // Tilt gain
  uint16_t tiltColor = !panActive ? COLOR_TEXT : COLOR_DIM;
  sprite.setTextColor(tiltColor);
  sprite.drawString("TILT", CX, 145);

  snprintf(valStr, sizeof(valStr), "^ %.1f v", tiltGain);
  sprite.setTextColor(COLOR_ACCENT);
  sprite.drawString(valStr, CX, 170);

  // Instructions
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_DIM);
  sprite.drawString("Swipe L/R: Pan", CX, 200);
  sprite.drawString("Swipe U/D: Tilt", CX, 213);
  sprite.drawString("Hold to save", CX, 228);

  drawCircularMask();
  sprite.pushSprite(0, 0);
}

void Display::drawWifiScreen(const char* ssid, const char* ip) {
  sprite.fillSprite(COLOR_BG);

  // Header
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_DIM);
  sprite.drawString("-- WiFi --", CX, 30);

  // SSID
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_TEXT);
  sprite.drawString("SSID:", CX, 70);

  sprite.setTextSize(2);
  sprite.setTextColor(COLOR_WIFI);
  sprite.drawString(ssid, CX, 95);

  // IP
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_TEXT);
  sprite.drawString("Go to:", CX, 130);

  sprite.setTextSize(2);
  sprite.setTextColor(COLOR_WIFI);
  sprite.drawString(ip, CX, 155);

  // Instructions
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_DIM);
  sprite.drawString("Hold to exit", CX, 210);

  drawCircularMask();
  sprite.pushSprite(0, 0);
}

void Display::setMode(DisplayMode mode) {
  currentMode = mode;
}

DisplayMode Display::getMode() const {
  return currentMode;
}

// ========== Private Helpers ==========

void Display::drawCrosshair(int cx, int cy, int radius) {
  // Thin crosshair lines
  sprite.drawLine(cx - radius, cy, cx + radius, cy, COLOR_CROSSHAIR);
  sprite.drawLine(cx, cy - radius, cx, cy + radius, COLOR_CROSSHAIR);

  // Small center mark
  sprite.drawCircle(cx, cy, 3, COLOR_CROSSHAIR);
}

void Display::drawTrackingDot(float panDeg, float tiltDeg, int radius) {
  // Small deadband to reduce jitter while holding still
  if (fabs(panDeg) < HtConfig::LEVEL_UI_DEADBAND_DEG) panDeg = 0.0f;
  if (fabs(tiltDeg) < HtConfig::LEVEL_UI_DEADBAND_DEG) tiltDeg = 0.0f;

  // Edge-lock behavior to avoid circling when level approaches singular/extreme poses
  float magDeg = sqrtf(panDeg * panDeg + tiltDeg * tiltDeg);
  if (magDeg > HtConfig::LEVEL_UI_MAX_DEG) {
    float norm = max(1e-6f, magDeg);
    panDeg = panDeg / norm * HtConfig::LEVEL_UI_MAX_DEG;
    tiltDeg = tiltDeg / norm * HtConfig::LEVEL_UI_MAX_DEG;
    magDeg = HtConfig::LEVEL_UI_MAX_DEG;
  }
  if (!levelEdgeLock && magDeg >= HtConfig::LEVEL_UI_EDGE_LOCK_ENTER_DEG) {
    levelEdgeLock = true;
    float norm = max(1e-6f, magDeg);
    float clampMag = min(magDeg, 90.0f);
    levelLockXDeg = panDeg / norm * clampMag;
    levelLockYDeg = tiltDeg / norm * clampMag;
  } else if (levelEdgeLock && magDeg <= HtConfig::LEVEL_UI_EDGE_LOCK_EXIT_DEG) {
    levelEdgeLock = false;
  }

  // Always clear lock near center after re-level to avoid stale edge direction
  if (magDeg < 5.0f) {
    levelEdgeLock = false;
  }

  if (levelEdgeLock) {
    panDeg = levelLockXDeg;
    tiltDeg = levelLockYDeg;
  }

  // Map level angle to pixel position
  // +/-90 degrees maps to full radius
  float maxDeg = 90.0f;
  int dotX = CX + (int)(panDeg / maxDeg * radius);
  int dotY = CY + (int)(tiltDeg / maxDeg * radius);

  // Clamp to circular display area
  float dx = dotX - CX;
  float dy = dotY - CY;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist > radius) {
    dotX = CX + (int)(dx / dist * radius);
    dotY = CY + (int)(dy / dist * radius);
  }

  // Draw tracking dot with glow effect
  sprite.fillCircle(dotX, dotY, 6, COLOR_DOT);
  sprite.drawCircle(dotX, dotY, 8, COLOR_DOT);
}

void Display::drawStatusDot(bool gyroStable, DisplayMode mode) {
  uint16_t color;
  const char* label;

  if (mode == DisplayMode::WIFI) {
    color = COLOR_WIFI;
    label = "WIFI";
  } else if (gyroStable) {
    color = COLOR_STABLE;
    label = "TRACKING";
  } else {
    color = COLOR_UNSTABLE;
    label = "CALIBRATING";
  }

  // Status dot at bottom
  sprite.fillCircle(70, 218, 4, color);
  sprite.setTextSize(1);
  sprite.setTextColor(color);
  sprite.setTextDatum(ML_DATUM);
  sprite.drawString(label, 80, 218);
  sprite.setTextDatum(MC_DATUM);  // Reset
}

void Display::drawPowerArc(float voltage) {
  // Draw a small arc at the top showing power status
  // Map voltage (3.0V - 5.5V) to arc coverage
  sprite.setTextSize(1);
  sprite.setTextColor(COLOR_DIM);

  char vStr[10];
  snprintf(vStr, sizeof(vStr), "%.1fV", voltage);
  sprite.drawString(vStr, CX, 15);
}

void Display::drawCircularMask() {
  // Mask corners to create circular display appearance
  // Draw black arcs in the corners outside the circular region
  for (int y = 0; y < HtConfig::LCD_HEIGHT; y++) {
    for (int x = 0; x < HtConfig::LCD_WIDTH; x++) {
      float dx = x - CX;
      float dy = y - CY;
      if (dx * dx + dy * dy > RADIUS * RADIUS) {
        sprite.drawPixel(x, y, COLOR_BG);
      }
    }
  }
}
