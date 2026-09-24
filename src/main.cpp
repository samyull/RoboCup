#include <Arduino.h>
#include "motors.h"
#include "pose.h"
#include "navigation.h"
#include "grid_map.h"
#include "TOFs.h"
#include "tof_nav.h"

static const uint8_t FLOW_CHIP_SELECT = 10;

static Target test_target = {1.0f, 0.0f};
static bool arrived = false;

static int left_pct, right_pct;
static uint16_t ranges[TOF_TOTAL_COUNT];

static int log_count = 1;

void setup() {
  Serial.begin(115200);
  delay(2000); // give Serial time to come up on Teensy AND time for you to open the monitor
  Serial.println("=== BOOT ===");

  motors_init();
  Serial.println("Motors initialized");

  map_init();
  Serial.println("Map initialised");

  pose_init(FLOW_CHIP_SELECT);
  Serial.print("pose_init() done - imu_ready=");
  Serial.print(pose_imu_ready());
  Serial.print(" flow_ready=");
  Serial.println(pose_flow_ready());

  if (!tof_init()) {
    Serial.println("WARNING: one or more TOF sensors failed to initialize");
  } else {
    Serial.println("All TOF sensors initialized");
  }

  Serial.println("=== SETUP COMPLETE ===");
}

void print_debugging_info(uint16_t ranges[TOF_TOTAL_COUNT]) {
  // Prints the current serial log number (how many times serial has printed)
  Serial.print("LOG "); Serial.print(log_count);
  log_count++;
  
  Serial.print("BOT R (L0): "); Serial.print(ranges[0]);
  if (tof_timeout_occurred(0)) Serial.print(" TIMEOUT");
  Serial.print(" | BOT L (L0): "); Serial.print(ranges[1]);
  if (tof_timeout_occurred(1)) Serial.print(" TIMEOUT");
  Serial.print(" | TOP R (L1): "); Serial.print(ranges[2]);
  if (tof_timeout_occurred(2)) Serial.print(" TIMEOUT");
  Serial.print(" | TOP L (L1): "); Serial.print(ranges[3]);
  if (tof_timeout_occurred(3)) Serial.print(" TIMEOUT");
  Serial.println();


}

void loop() {
  Pose pose = pose_update();
  if (!pose_imu_ready() || !pose_flow_ready()) {
    stop_motors();
    static uint32_t last_warn = 0;
    if (millis() - last_warn > 1000) {
      last_warn = millis();
      Serial.println("SENSOR NOT READY - motors held off");
    }
    delay(20);
    return;
  }
  map_update(pose);
  Serial.println("CP1: after map_update");
  Serial.print("P,"); Serial.print(pose.x, 3); Serial.print(",");
  Serial.print(pose.y, 3); Serial.print(","); Serial.println(pose.theta, 4);
  Serial.println("CP2: after pose print");

  // Debug print is slow (extra I2C reads) - throttle it to 5 Hz
  static uint32_t last_debug_ms = 0;
  if (millis() - last_debug_ms >= 200) {
    last_debug_ms = millis();
    pose_print_debug();
  }

  tof_read(ranges);
  Serial.println("CP3: after tof_read");

  tof_classify_readings(ranges, pose);
  Serial.println("CP4: after tof_classify_readings");

  print_debugging_info(ranges);


  if (!arrived && has_arrived(pose, test_target)) {
    arrived = true;
    stop_motors();
    Serial.println("Arrived at target");
  }

  if (!arrived) {
    navigate_to_target(pose, test_target, left_pct, right_pct);

    // float err = heading_error_to(pose, test_target);
    // float dist = distance_to(pose, test_target);
    // Serial.print("[nav] err_deg="); Serial.print(degrees(err), 1);
    // Serial.print(" dist="); Serial.print(dist, 3);
    // Serial.print(" L="); Serial.print(left_pct);
    // Serial.print(" R="); Serial.println(right_pct);

    // set_motors(left_pct, right_pct);
    stop_motors();
  }

  // if (tof_nav_update(ranges, left_pct, right_pct) != TOF_NAV_CLEAR) {
  //   set_motors(left_pct, right_pct);
  // } else {
  //   set_motors(3000, 3000);   // default driving, replace with navigate_to_target() later
  // }


  delay(20); // ~50Hz control loop
}