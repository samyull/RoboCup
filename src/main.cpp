#include <Arduino.h>
#include "motors.h"
#include "pose.h"
#include "navigation.h"
#include "grid_map.h"
#include "TOFs.h"
#include "tof_nav.h"
#include "state_machine.h"
#include "induction.h"
#include "ir_proximity.h"
#include "arms.h"
#include "electromagnets.h"
#include "weight_collection.h"

static const uint8_t FLOW_CHIP_SELECT = 10;
static uint16_t ranges[TOF_TOTAL_COUNT];

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== BOOT ===");

  motors_init();
  map_init();

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

  induction_init();
  ir_proximity_init();
  arms_init();
  electromagnets_init();
  weight_collection_init();   // parks crane at idle, arms at rest

  state_machine_init();

  Serial.println("=== SETUP COMPLETE ===");
}

void loop() {
  // --- Localisation ---
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

  Serial.print("P,"); Serial.print(pose.x, 3); Serial.print(",");
  Serial.print(pose.y, 3); Serial.print(","); Serial.println(pose.theta, 4);

  static uint32_t last_debug_ms = 0;
  if (millis() - last_debug_ms >= 200) {
    last_debug_ms = millis();
    pose_print_debug();
  }

  // --- Perception ---
  tof_read(ranges);
  bool weight_seen_this_frame = tof_classify_readings(ranges, pose);

  // --- Weight pickup interrupt - runs every loop regardless of top-level state ---
  weight_collection_update();

  int left_pct = 0, right_pct = 0;

  if (weight_collection_busy()) {
    // Catchment sensors have priority: hold still while waiting for
    // induction, picking up, or moving an arm. Top-level state machine is
    // deliberately not called at all here, so whatever state it was in is
    // untouched and resumes automatically once this clears.
    stop_motors();
  } else {
    state_machine_update(pose, ranges, weight_seen_this_frame, left_pct, right_pct);
    set_motors(left_pct, right_pct);
  }

  delay(20); // ~50Hz control loop
}