//************************************
//         TOFs.cpp
//************************************

// Four VL53L1X sensors on one I2C bus, each with its XSHUT routed via the
// SX1509 I/O expander. See ST's AN4846 for the multi-sensor address scheme.

#include "TOFs.h"
#include <Wire.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3E;

// Order: Bottom R, Bottom L, Top R, Top L
const uint8_t xshutPins[TOF_COUNT] = {3, 5, 4, 6};
const uint8_t tofAddress[TOF_COUNT] = {0x30, 0x32, 0x31, 0x33};

// Bottom sensors use the minimum ROI (4x4 SPADs, ~15 deg FOV).
// Top sensors keep the full 16x16 ROI (~27 deg FOV).
const uint8_t roiSize[TOF_COUNT] = {4, 4, 16, 16};

SX1509 io;
VL53L1X sensors[TOF_COUNT];
static bool timedOut[TOF_COUNT];

bool tof_init() {
  Wire.begin();

  if (!io.begin(SX1509_ADDRESS)) {
    Serial.println("SX1509 NOT FOUND");
    return false;
  }

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
  }

  bool all_ok = true;

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    io.digitalWrite(xshutPins[i], HIGH);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      all_ok = false;
      continue;
    }

    sensors[i].setAddress(tofAddress[i]);

    sensors[i].setDistanceMode(VL53L1X::Long);
    sensors[i].setMeasurementTimingBudget(50000); // 50 ms
    sensors[i].setROISize(roiSize[i], roiSize[i]);

    sensors[i].startContinuous(50); // period must be >= timing budget
  }

  return all_ok;
}

void tof_read(uint16_t ranges[TOF_TOTAL_COUNT]) {
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    uint16_t r = sensors[i].read();
    timedOut[i] = sensors[i].timeoutOccurred();
    if (timedOut[i] || r == 0 || r > TOF_MAX_RANGE_MM) r = TOF_MAX_RANGE_MM;
    ranges[i] = r;
  }
}

bool tof_timeout_occurred(uint8_t sensor_index) {
  return sensor_index < TOF_COUNT ? timedOut[sensor_index] : false;
}