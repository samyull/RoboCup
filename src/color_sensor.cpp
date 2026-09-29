#include "color_sensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// Same settings as the 102_Color example.
static Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// --- Classification thresholds ---
// TODO: calibrate on the arena. Put the robot on each surface and read the
// RGBC values from telemetry, then adjust these.
static const uint16_t BLACK_MAX_CLEAR = 300;   // clear below this = black
static const uint16_t WHITE_MIN_CLEAR = 3000;  // neutral and clear above this = white
static const float    COLOR_MIN_SHARE = 0.40f; // channel's share of R+G+B to count as that colour
static const float    COLOR_MARGIN    = 1.15f; // and must beat the other two channels by this factor

static const uint32_t POLL_MS = 50;  // one integration period

static bool sensor_ready = false;
static bool has_reading = false;
static uint16_t raw_r, raw_g, raw_b, raw_c;
static SurfaceColor current = COLOR_UNKNOWN;
static uint32_t last_read_ms = 0;

static SurfaceColor classify(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < BLACK_MAX_CLEAR) return COLOR_BLACK;

  const float sum = (float)r + g + b;
  if (sum <= 0.0f) return COLOR_BLACK;
  const float rs = r / sum, gs = g / sum, bs = b / sum;

  if (rs >= COLOR_MIN_SHARE && rs > gs * COLOR_MARGIN && rs > bs * COLOR_MARGIN) return COLOR_RED;
  if (gs >= COLOR_MIN_SHARE && gs > rs * COLOR_MARGIN && gs > bs * COLOR_MARGIN) return COLOR_GREEN;
  if (bs >= COLOR_MIN_SHARE && bs > rs * COLOR_MARGIN && bs > gs * COLOR_MARGIN) return COLOR_BLUE;

  return c >= WHITE_MIN_CLEAR ? COLOR_WHITE : COLOR_GREY;
}

bool color_sensor_init() {
  sensor_ready = tcs.begin(TCS34725_ADDRESS, &Wire1);
  if (!sensor_ready) {
    Serial.println("ERROR: TCS34725 colour sensor not found - check Con63 wiring");
    return false;
  }
  tcs.setInterrupt(false);  // LED on and left on: readings are continuous
  last_read_ms = millis();
  Serial.println("Colour sensor initialised");
  return true;
}

void color_sensor_update() {
  if (!sensor_ready || millis() - last_read_ms < POLL_MS) return;

  // Not getRawData(): it delay()s a full integration period after reading.
  if (!(tcs.read8(TCS34725_STATUS) & TCS34725_STATUS_AVALID)) return;
  raw_c = tcs.read16(TCS34725_CDATAL);
  raw_r = tcs.read16(TCS34725_RDATAL);
  raw_g = tcs.read16(TCS34725_GDATAL);
  raw_b = tcs.read16(TCS34725_BDATAL);
  last_read_ms = millis();
  has_reading = true;

  const SurfaceColor next = classify(raw_r, raw_g, raw_b, raw_c);
  if (next != current) {
    current = next;
    Serial.print("Surface colour: "); Serial.println(color_name(current));
  }
}

SurfaceColor color_sensor_color() {
  return current;
}

const char *color_name(SurfaceColor color) {
  switch (color) {
    case COLOR_UNKNOWN: return "UNKNOWN";
    case COLOR_BLACK:   return "BLACK";
    case COLOR_WHITE:   return "WHITE";
    case COLOR_GREY:    return "GREY";
    case COLOR_RED:     return "RED";
    case COLOR_GREEN:   return "GREEN";
    case COLOR_BLUE:    return "BLUE";
  }
  return "?";
}

bool color_sensor_raw(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (!has_reading) return false;
  r = raw_r; g = raw_g; b = raw_b; c = raw_c;
  return true;
}
