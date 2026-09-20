#ifndef TOFS_H
#define TOFS_H

#include <stdint.h>

// All four sensors are VL53L1X.
// Index order: [0]=Bottom R, [1]=Bottom L, [2]=Top R, [3]=Top L
#define TOF_COUNT 4
#define TOF_TOTAL_COUNT TOF_COUNT
#define TOF_MAX_RANGE_MM 1000

// Initializes the SX1509 expander, resets all sensors via XSHUT, brings
// each sensor up one at a time and assigns it a unique I2C address.
// Returns false if any sensor fails to initialize.
bool tof_init();

// Reads the latest continuous-mode range (mm) from all sensors into the
// provided array. Layout: [0]=Bottom R, [1]=Bottom L, [2]=Top R, [3]=Top L.
void tof_read(uint16_t ranges[TOF_TOTAL_COUNT]);

// Per-sensor timeout check.
bool tof_timeout_occurred(uint8_t sensor_index);

#endif