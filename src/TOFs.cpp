//************************************
//         TOFs.cpp
//************************************

// Eight VL53L1X sensors on one I2C bus, each with its XSHUT routed via the
// SX1509 I/O expander. See ST's AN4846 for the multi-sensor address scheme.

#include "TOFs.h"
#include <Wire.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3E;

// Order: LST, LSB, LFT, LFB, RFB, RFT, RSB, RST.
const uint8_t xshutPins[TOF_COUNT] = {6, 7, 1, 5, 3, 4, 0, 2};
const uint8_t tofAddress[TOF_COUNT] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
// Only front top sensors use the full ROI; all others use 4x4.
const uint8_t roiSize[TOF_COUNT] = {4, 4, 16, 4, 4, 16, 4, 4};

SX1509 io;
VL53L1X sensors[TOF_COUNT];
static bool timedOut[TOF_COUNT];
static bool initialized[TOF_COUNT];

bool tof_init() {
  Wire.begin();
  for (uint8_t i = 0; i < TOF_COUNT; ++i) {
    initialized[i] = false;
    timedOut[i] = false;
  }

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
      io.digitalWrite(xshutPins[i], LOW); // isolate failed device at default address
      continue;
    }

    sensors[i].setAddress(tofAddress[i]);

    sensors[i].setDistanceMode(VL53L1X::Long);
    sensors[i].setMeasurementTimingBudget(50000); // 50 ms
    sensors[i].setROISize(roiSize[i], roiSize[i]);

    sensors[i].startContinuous(50); // period must be >= timing budget
    initialized[i] = sensors[i].last_status == 0;
    if (!initialized[i]) {
      all_ok = false;
      io.digitalWrite(xshutPins[i], LOW);
    }
  }

  return all_ok;
}

bool tof_reading_usable(const TofReading &reading) {
  return reading.valid && (uint32_t)(millis() - reading.received_ms) < TOF_MAX_AGE_MS;
}

void tof_read(TofReading ranges[TOF_TOTAL_COUNT]) {
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    TofReading &sample = ranges[i];
    sample.new_this_loop = false;
    timedOut[i] = false;
    if (!initialized[i]) {
      sample.valid = false;
      continue;
    }
    const bool ready = sensors[i].dataReady();
    if (sensors[i].last_status != 0) {
      sample.valid = false;
      continue;
    }
    if (!ready) continue; // retain a recent result while the next is measured

    const uint16_t distance = sensors[i].read(false);
    timedOut[i] = sensors[i].timeoutOccurred();
    sample.range_mm = distance;
    sample.received_ms = millis();
    ++sample.sequence;
    sample.new_this_loop = true;
    sample.valid = !timedOut[i] && sensors[i].last_status == 0 && distance > 0 &&
                   sensors[i].ranging_data.range_status == VL53L1X::RangeValid;
  }
}

bool tof_timeout_occurred(uint8_t sensor_index) {
  return sensor_index < TOF_COUNT ? timedOut[sensor_index] : false;
}
