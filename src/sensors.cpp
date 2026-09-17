//************************************
//         sensors.cpp       
//************************************

 // This file contains functions used to read and average
 // the sensors.


#include "sensors.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Bitcraze_PMW3901.h>


// ---- BNO055 (heading) ----
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ---- PMW3901 (xy) ----
#define FLOW_CS_PIN 10   // TODO: set to whatever Teensy pin your PMW3901 CS is wired to
Bitcraze_PMW3901 flow(FLOW_CS_PIN);

// TODO: calibrate this. Converts raw sensor counts -> mm of real-world displacement.
// The PMW3901's count-to-distance ratio depends on the sensor's fixed mounting
// height above the ground (further away = each count represents more real mm).
// Easiest way to calibrate: push the robot a known distance (e.g. 500mm) in a
// straight line, print the raw counts, then FLOW_SCALE = distance_mm / total_counts
#define FLOW_SCALE 1.0f

void sensors_init() {
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected");
  } else {
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 initialised");
  }

  // if (!flow.begin()) {
  //   Serial.println("ERROR: PMW3901 not detected");
  // } else {
  //   Serial.println("PMW3901 initialised");
  // }
}

float getHeading() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;   // heading in degrees, 0-360
}

void read_opticalFlow(float &dy, float &dx) {
  int16_t deltaX = 0, deltaY = 0;
  // flow.readMotionCount(&deltaX, &deltaY);

  dx = deltaX * FLOW_SCALE;
  dy = deltaY * FLOW_SCALE;
}