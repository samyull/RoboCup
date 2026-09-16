#include <Arduino.h>
#include "motors.h"
#include "pose.h"
#include "navigation.h"

// TODO: change to whatever pin the pmw3901 cs is actually connected to
static const uint8_t FLOW_CHIP_SELECT = 10;

static Target test_target = {1.0f, 0.5f};
static bool arrived = false;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial port time to connect on Teensy

  motors_init();
  pose_init(FLOW_CHIP_SELECT);

  Serial.println("Setup complete - driving to test target at (1.0, 0.5)");
}

void loop() {
  Pose pose = pose_update();
  if (!arrived && has_arrived(pose, test_target)) {
    arrived = true;
    stop_motors();
    Serial.println("Arrived at target!");
  }

  if (!arrived) {
    int left_pct, right_pct;
    navigate_to_target(pose, test_target, left_pct, right_pct);
    set_motors(left_pct, right_pct);

    // Lightweight telemetry output for debugging
    Serial.print("x="); Serial.print(pose.x, 3);
    Serial.print(", y="); Serial.print(pose.y, 3);
    Serial.print(", theta (deg)="); Serial.print(degrees(pose.theta), 1);
    Serial.print(", L="); Serial.print(left_pct);
    Serial.print(", R="); Serial.println(right_pct);
  }

  delay(20); // 50Hz loop
}