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
static const char *const tofNames[TOF_COUNT] = {"LST", "LSB", "LFT", "LFB", "RFB", "RFT", "RSB", "RST"};

const char *tof_name(uint8_t index) {
  return index < TOF_COUNT ? tofNames[index] : "?";
}

static void print_sensor_label(uint8_t i) {
  Serial.print("ToF "); Serial.print(tofNames[i]);
  Serial.print(" (XSHUT"); Serial.print(xshutPins[i]); Serial.print(")");
}

// A sensor that produces no measurement for this long is reset and reinitialised.
// Covers brown-outs, which silently return a sensor to the default I2C address.
static const uint32_t TOF_REINIT_AFTER_MS = 500;

SX1509 io;
VL53L1X sensors[TOF_COUNT];
static bool initialized[TOF_COUNT];
static uint32_t last_data_ms[TOF_COUNT];  // last measurement, or last (re)init attempt

// Call with this sensor's XSHUT just raised and every other sensor either at
// its own address or held in reset, so it is the only device at the default
// address. Leaves it held in reset (XSHUT low) on failure.
static bool configure_sensor(uint8_t i) {
  sensors[i] = VL53L1X();  // fresh driver state, pointing at the default address
  sensors[i].setTimeout(500);
  if (!sensors[i].init()) {
    io.digitalWrite(xshutPins[i], LOW); // isolate failed device at default address
    return false;
  }

  sensors[i].setAddress(tofAddress[i]);

  sensors[i].setDistanceMode(VL53L1X::Long);
  sensors[i].setMeasurementTimingBudget(50000); // 50 ms
  sensors[i].setROISize(roiSize[i], roiSize[i]);

  sensors[i].startContinuous(50); // period must be >= timing budget
  if (sensors[i].last_status != 0) {
    io.digitalWrite(xshutPins[i], LOW);
    return false;
  }
  return true;
}

bool tof_init() {
  Wire.begin();
  for (uint8_t i = 0; i < TOF_COUNT; ++i) {
    initialized[i] = false;
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

    initialized[i] = configure_sensor(i);
    last_data_ms[i] = millis();
    print_sensor_label(i);
    if (!initialized[i]) {
      Serial.println(" FAILED - will keep retrying");
      all_ok = false;
    } else {
      Serial.println(" OK");
    }
  }

  return all_ok;
}

// Reset every stale sensor together, then bring them up one at a time, so two
// sensors that both fell back to the default address never answer at once.
static void reinit_sensors(const bool stale[TOF_COUNT]) {
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    if (!stale[i]) continue;
    if (initialized[i]) {
      print_sensor_label(i);
      Serial.println(" no data for 500 ms - reinitialising");
    }
    io.digitalWrite(xshutPins[i], LOW);
  }
  delay(1);

  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    if (!stale[i]) continue;
    const bool was_running = initialized[i];
    io.digitalWrite(xshutPins[i], HIGH);
    delay(2);  // boot time is 1.2 ms max; init() also waits for boot
    initialized[i] = configure_sensor(i);
    last_data_ms[i] = millis();  // next retry, if needed, is another 500 ms away
    // Report recoveries and first failures only, not every retry of a dead sensor.
    if (initialized[i] || was_running) {
      print_sensor_label(i);
      Serial.println(initialized[i] ? " reinit OK" : " reinit FAILED - will keep retrying");
    }
  }
}

static bool reading_fresh(const TofReading &reading) {
  return (uint32_t)(millis() - reading.received_ms) < TOF_MAX_AGE_MS;
}

bool tof_reading_usable(const TofReading &reading) {
  return reading.valid && reading_fresh(reading);
}

bool tof_reading_no_target(const TofReading &reading) {
  return reading.no_target && reading_fresh(reading);
}

bool tof_reading_live(const TofReading &reading) {
  return tof_reading_usable(reading) || tof_reading_no_target(reading);
}

void tof_reading_clear(TofReading &reading) {
  reading.valid = false;
  reading.no_target = false;
}

// Range statuses meaning the measurement worked but found nothing within range.
static bool status_is_no_target(uint8_t status) {
  switch (status) {
    case VL53L1X::SignalFail:       // return too weak - nothing close enough to see
    case VL53L1X::SigmaFail:        // too noisy to give a distance - typically a far, weak return
    case VL53L1X::OutOfBoundsFail:  // nothing detected in range
    case VL53L1X::WrapTargetFail:   // target beyond the range limit (phase wrapped)
      return true;
    default:
      return false;
  }
}

static void read_sensor(uint8_t i, TofReading &sample) {
  sample.new_this_loop = false;
  if (!initialized[i]) {
    tof_reading_clear(sample);
    return;
  }
  const bool ready = sensors[i].dataReady();
  if (sensors[i].last_status != 0) {
    tof_reading_clear(sample);
    return;
  }
  if (!ready) return; // retain a recent result while the next is measured

  const uint16_t distance = sensors[i].read(false);
  const bool timed_out = sensors[i].timeoutOccurred();
  const bool measured = !timed_out && sensors[i].last_status == 0;
  const uint8_t status = sensors[i].ranging_data.range_status;
  // Any completed measurement counts as data, even "nothing in range".
  if (measured) last_data_ms[i] = millis();
  sample.range_mm = distance;
  sample.received_ms = millis();
  ++sample.sequence;
  sample.new_this_loop = true;
  sample.valid = measured && distance > 0 && status == VL53L1X::RangeValid;
  sample.no_target = measured && status_is_no_target(status);
}

void tof_read(TofReading ranges[TOF_TOTAL_COUNT]) {
  bool stale[TOF_COUNT] = {};
  bool any_stale = false;
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    read_sensor(i, ranges[i]);
    if ((uint32_t)(millis() - last_data_ms[i]) >= TOF_REINIT_AFTER_MS) {
      stale[i] = any_stale = true;
      tof_reading_clear(ranges[i]);
    }
  }
  if (any_stale) reinit_sensors(stale);
}
