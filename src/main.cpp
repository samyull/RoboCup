#include <Arduino.h>
#include "motors.h"
#include "pose.h"
#include "navigation.h"
#include "grid_map.h"

// TODO: change to whatever pin the PMW3901's CS line is actually wired to
static const uint8_t FLOW_CHIP_SELECT = 10;

// --- DEBUG MODE ---
// Set to true first: this bypasses sensors/navigation entirely and just
// pulses the motors directly, so you can confirm the motor driver boards,
// wiring, and pin numbers are actually correct before trusting anything
// upstream of them. Once you see the wheels actually turn, set to false.
static const bool MOTOR_TEST_MODE = false;

static Target test_target = {0.0f, 1.0f};
static bool arrived = false;

void run_motor_test() {
  Serial.println("--- MOTOR TEST: forward ---");
  set_motors(90, 90);
  delay(1500);

  Serial.println("--- MOTOR TEST: stop ---");
  set_motors(0, 0);
  delay(1000);

  Serial.println("--- MOTOR TEST: backward ---");
  set_motors(-90, -90);
  delay(1500);

  Serial.println("--- MOTOR TEST: stop ---");
  set_motors(0, 0);
  delay(1000);

  Serial.println("--- MOTOR TEST: spin (left back, right fwd) ---");
  set_motors(-90, 90);
  delay(1500);

  Serial.println("--- MOTOR TEST: stop ---");
  set_motors(0, 0);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  delay(2000); // give Serial time to come up on Teensy AND time for you to open the monitor
  Serial.println("=== BOOT ===");

  motors_init();
  Serial.println("motors_init() done");

  map_init();

  if (!MOTOR_TEST_MODE) {
    pose_init(FLOW_CHIP_SELECT);
    Serial.print("pose_init() done - imu_ready=");
    Serial.print(pose_imu_ready());
    Serial.print(" flow_ready=");
    Serial.println(pose_flow_ready());
  }

  Serial.println("=== SETUP COMPLETE ===");
}

void loop() {
  if (MOTOR_TEST_MODE) {
    run_motor_test();
    return; // repeats the test sequence forever
  }

  Pose pose = pose_update();
  map_update(pose);

  // Debug print is slow (extra I2C reads) - throttle it to 5 Hz
  static uint32_t last_debug_ms = 0;
  if (millis() - last_debug_ms >= 200) {
    last_debug_ms = millis();
    pose_print_debug();
  }

  if (!arrived && has_arrived(pose, test_target)) {
    arrived = true;
    stop_motors();
    Serial.println("Arrived at target");
  }

  if (!arrived) {
    int left_pct, right_pct;
    navigate_to_target(pose, test_target, left_pct, right_pct);

    // float err = heading_error_to(pose, test_target);
    // float dist = distance_to(pose, test_target);
    // Serial.print("[nav] err_deg="); Serial.print(degrees(err), 1);
    // Serial.print(" dist="); Serial.print(dist, 3);
    // Serial.print(" L="); Serial.print(left_pct);
    // Serial.print(" R="); Serial.println(right_pct);

    set_motors(left_pct, right_pct);
  }

  delay(20); // ~50Hz control loop
}