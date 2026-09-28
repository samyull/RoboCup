#include <Arduino.h>
#include "motors.h"
#include "pose.h"
#include "grid_map.h"
#include "TOFs.h"
#include "tof_nav.h"
#include "state_machine.h"
#include "induction.h"
#include "ir_proximity.h"
#include "arms.h"
#include "electromagnets.h"
#include "weight_collection.h"
#include "arena_config.h"

static const uint8_t FLOW_CHIP_SELECT = 10;
static TofReading ranges[TOF_TOTAL_COUNT];
// Matches the current minimum effective motor power; tune on the robot.
static const int FUNNEL_FEED_SPEED_PCT = 60;

static bool testing = true;

// Print normal-loop averages/maxima once a second; never from the fast collection path.
enum LoopStage { LS_COLLECTION, LS_DECAY, LS_POSE, LS_DEBUG, LS_TOF, LS_MAP,
                 LS_DRIVE, LS_SERIAL, LS_DELAY, LS_COUNT };
static const char *stage_names[LS_COUNT] = {
  "collection", "decay", "pose", "debug", "tof", "map", "drive", "serial", "delay"
};
static uint64_t stage_total_us[LS_COUNT] = {};
static uint32_t stage_max_us[LS_COUNT] = {};
static uint32_t timing_loops = 0, timing_last_report_ms = 0;
static uint64_t timing_total_us = 0;
static uint32_t timing_max_us = 0, timing_previous_report_us = 0;
static void time_stage(LoopStage stage, uint32_t &mark) {
  const uint32_t now = micros(), elapsed = now - mark;
  stage_total_us[stage] += elapsed;
  if (elapsed > stage_max_us[stage]) stage_max_us[stage] = elapsed;
  mark = now;
}
static void report_loop_timing(uint32_t loop_start_us) {
  const uint32_t duration = micros() - loop_start_us;
  ++timing_loops;
  timing_total_us += duration;
  if (duration > timing_max_us) timing_max_us = duration;
  if (millis() - timing_last_report_ms < 1000) return;
  const uint32_t report_start = micros();
  Serial.print("Loop timing: n="); Serial.print(timing_loops);
  Serial.print(" avg_ms="); Serial.print((double)timing_total_us / timing_loops / 1000.0, 2);
  Serial.print(" max_ms="); Serial.print(timing_max_us / 1000.0, 2);
  Serial.print(" stages_avg/max_ms:");
  for (int i = 0; i < LS_COUNT; ++i) {
    Serial.print(" "); Serial.print(stage_names[i]); Serial.print("=");
    Serial.print((double)stage_total_us[i] / timing_loops / 1000.0, 2);
    Serial.print("/"); Serial.print(stage_max_us[i] / 1000.0, 2);
    stage_total_us[i] = 0; stage_max_us[i] = 0;
  }
  Serial.print(" previous_report_ms="); Serial.println(timing_previous_report_us / 1000.0, 2);
  timing_previous_report_us = micros() - report_start;
  timing_last_report_ms = millis();
  timing_loops = 0; timing_total_us = 0; timing_max_us = 0;
}
static bool collection_motion_active() {
  const WeightState state = weight_collection_state();
  return state == WC_PICKUP || state == WC_CLEARING || state == WC_RELEASING;
}

// Standard ARM Cortex-M system reset, via the Application Interrupt and
// Reset Control Register. Architectural, not Teensy-specific - resets the
// chip and re-runs setup()/loop() from scratch. Does NOT reflash anything;
// this only restarts whatever program is already on the chip.
// BENCH TESTING ONLY - never trigger this during an actual timed round,
// it wipes all state (pose, map, weight count) back to zero.
#define CPU_RESTART_ADDR ((volatile uint32_t *)0xE000ED0Cu)
#define CPU_RESTART_VAL  0x5FA0004u
static inline void cpu_restart() {
  *CPU_RESTART_ADDR = CPU_RESTART_VAL;
  while (1) {}   // never reached - reset happens immediately
}

static void check_serial_commands() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'R') {
      Serial.println("Restarting...");
      delay(50);   // let the message actually get out before resetting
      cpu_restart();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== BOOT ===");

  motors_init();
  map_init();

  pose_init(FLOW_CHIP_SELECT);
  pose_set_position(ROBOT_START_X_M, ROBOT_START_Y_M);
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
  if(testing) {
    check_serial_commands();
  }

  static bool collection_was_active = false;
  static bool recovery_active = false;
  static bool discard_tof_on_resume = false;
  static uint32_t recovery_start_ms = 0;
  // These commands must also reset when collection bypasses normal navigation.
  static int left_pct = 0, right_pct = 0;
  const uint32_t loop_start_us = micros();
  uint32_t timing_mark = loop_start_us;

  if (collection_was_active && !collection_motion_active()) {
    pose_resume_after_collection();
    for (auto &reading : ranges) reading.valid = false;
    discard_tof_on_resume = true;
    recovery_active = true;
    recovery_start_ms = millis();
    collection_was_active = false;
    Serial.println("Collection complete - refreshing pose and ToFs before driving");
  }
  // Poll catchment sensors before any slower I2C, mapping or debug work.
  weight_collection_update();
  if (collection_motion_active() || collection_was_active) {
    stop_motors();
    left_pct = right_pct = 0;
    if (!collection_was_active) {
      Serial.println("M,0,0");
      Serial.print("S,"); Serial.print((int)state_machine_current_state()); Serial.println(",1");
    }
    collection_was_active = true;
    // Same cadence as crane_test(); no pose/ToF/map diagnostics in this path.
    delay(10);
    return;
  }
  time_stage(LS_COLLECTION, timing_mark);

  // --- Localisation ---
  map_decay();
  time_stage(LS_DECAY, timing_mark);
  Pose pose = pose_update();
  time_stage(LS_POSE, timing_mark);
  if (!pose_imu_ready() || !pose_flow_ready()) {
    stop_motors();
    static uint32_t last_warn = 0;
    if (millis() - last_warn > 1000) {
      last_warn = millis();
      Serial.println("SENSOR NOT READY - motors held off");
    }
    time_stage(LS_DEBUG, timing_mark);
    delay(20);
    time_stage(LS_DELAY, timing_mark);
    report_loop_timing(loop_start_us);
    return;
  }
  // Stop promptly on timeout, but keep collection and sensor updates running.
  if (pose_heading_stale()) stop_motors();

  Serial.print("P,"); Serial.print(pose.x, 3); Serial.print(",");
  Serial.print(pose.y, 3); Serial.print(","); Serial.println(pose.theta, 4);

  static uint32_t last_debug_ms = 0;
  if (millis() - last_debug_ms >= 200) {
    last_debug_ms = millis();
    pose_print_debug();
  }

  time_stage(LS_DEBUG, timing_mark);
  // --- Perception ---
  tof_read(ranges);
  if (discard_tof_on_resume) {
    // The first ready result may have been buffered during collection.
    for (auto &reading : ranges) reading.valid = false;
    discard_tof_on_resume = false;
  }
  time_stage(LS_TOF, timing_mark);
  if (recovery_active && millis() - recovery_start_ms >= 150 && pose_heading_valid()) {
    bool all_fresh = true;
    for (const auto &reading : ranges) all_fresh &= tof_reading_usable(reading);
    if (all_fresh) recovery_active = false;
  }
  const bool weight_seen_this_frame = !recovery_active && tof_classify_readings(ranges, pose);
  time_stage(LS_MAP, timing_mark);

  // Retain the previous command through a brief heading-read failure.
  int obstacle_left = 0, obstacle_right = 0;
  const bool obstacle_data_missing =
      tof_nav_update(ranges, obstacle_left, obstacle_right) == TOF_NAV_UNKNOWN;

  if (pose_heading_stale()) {
    left_pct = right_pct = 0;
    stop_motors();
    static uint32_t last_heading_warn_ms = 0;
    if (millis() - last_heading_warn_ms >= 1000) {
      last_heading_warn_ms = millis();
      Serial.println("IMU HEADING TIMEOUT - motors held off; retrying");
    }
  } else if (recovery_active || obstacle_data_missing) {
    // Also hold scan/verification turns when the obstacle sensors are unknown.
    left_pct = right_pct = 0;
    stop_motors();
  } else if (weight_collection_feeding()) {
    // Keep the object moving into the funnel without navigation turning away.
    // Upper ToF obstacle detection stops feeding instead of steering sideways.
    int avoid_left = 0, avoid_right = 0;
    left_pct = right_pct = 0;
    if (tof_nav_update(ranges, avoid_left, avoid_right) == TOF_NAV_CLEAR) {
      left_pct = right_pct = FUNNEL_FEED_SPEED_PCT;
    }
    set_motors(left_pct, right_pct);
  } else if (weight_collection_busy()) {
    // Navigation stays paused throughout feeding, pickup, and arm movement.
    left_pct = right_pct = 0;
    stop_motors();
  } else if (pose_heading_valid()) {
    state_machine_update(pose, ranges, weight_seen_this_frame, left_pct, right_pct);
    set_motors(left_pct, right_pct);
  } else {
    // Do not verify targets or alter the map using a failed heading sample.
    // Keep obstacle avoidance active during the short grace period.
    int avoid_left = 0, avoid_right = 0;
    if (tof_nav_update(ranges, avoid_left, avoid_right) != TOF_NAV_CLEAR) {
      left_pct = avoid_left;
      right_pct = avoid_right;
    }
    set_motors(left_pct, right_pct);
  }

  time_stage(LS_DRIVE, timing_mark);

  // Debug: motor output and state, for the visualiser / bench debugging.
  Serial.print("M,"); Serial.print(left_pct); Serial.print(","); Serial.println(right_pct);
  Serial.print("S,"); Serial.print((int)state_machine_current_state());
  Serial.print(","); Serial.println(weight_collection_busy());

  // Debug: raw catchment sensor readings - if IR proximity reads "detected"
  // with nothing in front of it, the pin is very likely floating.
  Serial.print("D,ind="); Serial.print(induction_read());
  Serial.print(",ind_det="); Serial.print(induction_detected());
  Serial.print(",ir_det="); Serial.println(ir_proximity_detected());

  time_stage(LS_SERIAL, timing_mark);
  delay(20); // nominal pause, not the total loop duration
  time_stage(LS_DELAY, timing_mark);
  report_loop_timing(loop_start_us);
}
