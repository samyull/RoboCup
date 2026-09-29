#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <stdint.h>

// TCS34725 colour sensor (based on the 102_Color example), facing the floor.
// Plugged into Con63, which is on the Teensy's second I2C bus (Wire1,
// shared with the BNO055 at 0x28; this sensor is fixed at 0x29).

enum SurfaceColor : uint8_t {
  COLOR_UNKNOWN,  // no sensor / no reading yet
  COLOR_BLACK,
  COLOR_WHITE,
  COLOR_GREY,     // neutral, between black and white
  COLOR_RED,
  COLOR_GREEN,
  COLOR_BLUE
};

// Returns false if the sensor isn't found (the robot keeps running without it).
bool color_sensor_init();

// Non-blocking: call every loop. Picks up a new reading whenever the
// sensor has finished one (every ~50 ms) and reclassifies the colour.
void color_sensor_update();

// Latest classified surface colour.
SurfaceColor color_sensor_color();
const char *color_name(SurfaceColor color);

// Latest raw channel counts. Returns false if there is no reading yet.
// Use these (printed in telemetry) to calibrate the thresholds in color_sensor.cpp.
bool color_sensor_raw(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c);

#endif
