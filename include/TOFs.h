#ifndef TOFS_H
#define TOFS_H

#include <stdint.h>

// Sensor order matches the physical connector/XSHUT assignment.
enum TofIndex : uint8_t {
  TOF_LST = 0, // left side top, XSHUT6
  TOF_LSB = 1, // left side bottom, XSHUT7
  TOF_LFT = 2, // left front top, XSHUT1
  TOF_LFB = 3, // left front bottom, XSHUT5
  TOF_RFB = 4, // right front bottom, XSHUT3
  TOF_RFT = 5, // right front top, XSHUT4
  TOF_RSB = 6, // right side bottom, XSHUT0
  TOF_RST = 7, // right side top, XSHUT2
  TOF_COUNT = 8
};
#define TOF_TOTAL_COUNT TOF_COUNT
#define TOF_MAX_RANGE_MM 1000

static const uint32_t TOF_MAX_AGE_MS = 150;
static const uint32_t TOF_PAIR_MAX_SKEW_MS = 50;

struct TofReading {
  uint16_t range_mm = 0;
  uint32_t received_ms = 0;
  uint32_t sequence = 0;
  bool valid = false;
  bool new_this_loop = false;
};

// Valid measurement received within the allowed age; never substitutes clear space.
bool tof_reading_usable(const TofReading &reading);

// Initializes the SX1509 expander, resets all sensors via XSHUT, brings
// each sensor up one at a time and assigns it a unique I2C address.
// Returns false if any sensor fails to initialize.
bool tof_init();

// Poll ready sensors only. Caller retains this array between calls.
// Layout follows TofIndex.
void tof_read(TofReading ranges[TOF_TOTAL_COUNT]);

// Per-sensor timeout check.
bool tof_timeout_occurred(uint8_t sensor_index);

#endif