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
#include "exploration.h"
#include "wall_anchor.h"
#include "ultrasound.h"
#include "color_sensor.h"
#include "return_to_base.h"
#include "bench_config.h"
#if ENABLE_BENCH_TESTS
#include "flow_test.h"
#endif

static const uint8_t FLOW_CHIP_SELECT = 10;
static TofReading ranges[TOF_TOTAL_COUNT];
// Matches the current minimum effective motor power; tune on the robot.
static const int FUNNEL_FEED_SPEED_PCT = 65;

#if ENABLE_BENCH_TESTS
static bool testing = true;
// Hold the pre-round loop after a bench test so its summary stays visible.
static bool bench_output_paused = false;
#endif

// Correct x/y against the four outer arena walls (see wall_anchor.h).
static const bool USE_WALL_ANCHOR = true;

// --- Round: wait for the start button, run for 120 s, then halt ---
// Button on A7 (zener for protection), internal pull-up. The wiring's polarity
// is not assumed: whatever level the pin idles at while waiting is "released"
// and the opposite level is "pressed" (measured on the robot: idles LOW).
static const uint8_t  START_BUTTON_PIN   = A7;
static const uint32_t START_LEARN_MS     = 300;  // sample the untouched level this long
static const uint32_t START_ARM_MS       = 500;  // must read released this long before a press counts
static const uint32_t START_PRESS_MS     = 150;  // minimum hold for a press
static const uint32_t START_RELEASE_MS   = 50;   // release must be steady this long to start
static const uint32_t ROUND_LENGTH_MS    = 120000;
enum RoundPhase { ROUND_WAITING, ROUND_RUNNING, ROUND_OVER };
static RoundPhase round_phase = ROUND_WAITING;
static uint32_t round_start_ms = 0;
#if ENABLE_BENCH_TESTS
static bool start_requested = false;  // set by the 'G' serial command (testing only)
#endif

static const char *round_phase_name() {
  switch (round_phase) {
    case ROUND_WAITING: return "WAITING";
    case ROUND_RUNNING: return "RUNNING";
    case ROUND_OVER:    return "OVER";
  }
  return "?";
}

// --- Ultrasound escape ---
// Left ultrasound interrupt -> left track reverses; right -> right track reverses;
// both at once -> full reverse. Lasts US_ESCAPE_MS from the latest trigger.
// Overrides navigation, but never weight collection or an IMU timeout stop.
static const uint32_t US_ESCAPE_MS        = 2000;
static const int      US_ESCAPE_SPEED_PCT = 65;   // minimum effective motor power
static uint32_t us_escape_start_ms = 0;
static bool us_escape_running = false;
static int us_escape_left_pct = 0, us_escape_right_pct = 0;

// Attached to both ultrasound interrupts in setup(). Re-reads both sides so a
// second side triggering during an escape upgrades it to a full reverse.
static void on_ultrasound_interrupt() {
  if (round_phase != ROUND_RUNNING) return;
  const bool left = ultrasound_interrupt_left(), right = ultrasound_interrupt_right();
  us_escape_left_pct  = left  ? -US_ESCAPE_SPEED_PCT : 0;
  us_escape_right_pct = right ? -US_ESCAPE_SPEED_PCT : 0;
  us_escape_start_ms = millis();
  us_escape_running = true;
  Serial.print("ULTRASOUND ESCAPE: ");
  Serial.println(left && right ? "full reverse" : left ? "left track reverse" : "right track reverse");
}

static bool us_escape_active() {
  if (us_escape_running && millis() - us_escape_start_ms >= US_ESCAPE_MS) {
    us_escape_running = false;
    Serial.println("Ultrasound escape finished");
  }
  return us_escape_running;
}

// --- Soft start ---
// The flow sensor tracks poorly on the coloured start square, so the round opens
// by driving straight off it: SOFT_START_PCT for SOFT_START_MS, holding the start
// heading with the IMU. Ends early if something is close ahead (normal navigation
// and its obstacle reflex take over). Weight collection still pre-empts it.
static const uint32_t SOFT_START_MS  = 2000;
static const int      SOFT_START_PCT = 80;
static const float    SOFT_START_TURN_KP = 40.0f;  // % per rad of heading error
static const int      SOFT_START_MAX_TURN_PCT = 20;
static bool soft_start_running = false;

static void soft_start_begin() {
  soft_start_running = true;
  Serial.println("SOFT START: driving off the start square");
}

// True (and sets the motor commands) while the soft start is driving.
static bool soft_start_drive(const TofReading r[TOF_TOTAL_COUNT], const Pose &pose,
                             int &left_pct, int &right_pct) {
  if (!soft_start_running) return false;
  int avoid_left, avoid_right;
  const bool blocked = tof_nav_update(r, avoid_left, avoid_right) == TOF_NAV_AVOID;
  if (blocked || millis() - round_start_ms >= SOFT_START_MS) {
    soft_start_running = false;
    Serial.println(blocked ? "SOFT START: ended early - obstacle ahead" : "SOFT START: done");
    return false;
  }
  // Positive error = need to turn left (CCW): speed up the right track.
  const float error = remainderf(radians(ROBOT_START_HEADING_DEG) - pose.theta, 2.0f * PI);
  const int turn = constrain((int)(SOFT_START_TURN_KP * error),
                             -SOFT_START_MAX_TURN_PCT, SOFT_START_MAX_TURN_PCT);
  left_pct = SOFT_START_PCT - turn;
  right_pct = SOFT_START_PCT + turn;
  return true;
}

// A TEL line and a TOF line every loop, whatever path the loop takes. Map cells (Q,)
// and one-off events (state changes, warnings) are still printed as they happen.
static Pose last_pose;

static void print_telemetry(int left_pct, int right_pct) {
  Serial.print("TEL,round=");  Serial.print(round_phase_name());
  Serial.print(",t_s=");
  Serial.print(round_phase == ROUND_WAITING ? 0.0f : (millis() - round_start_ms) / 1000.0f, 1);
  Serial.print(",state="); Serial.print(state_machine_state_name());
  Serial.print(",collect=");  Serial.print(weight_collection_state_name());
  Serial.print(",x=");        Serial.print(last_pose.x, 3);
  Serial.print(",y=");        Serial.print(last_pose.y, 3);
  Serial.print(",th=");       Serial.print(degrees(last_pose.theta), 1);
  Serial.print(",motors=");   Serial.print(left_pct); Serial.print("/"); Serial.print(right_pct);
  Serial.print(",ind=");      Serial.print(induction_read());
  Serial.print(",ind_det=");  Serial.print(induction_detected());
  Serial.print(",ir_det=");   Serial.print(ir_proximity_detected());
  uint8_t sys, gyro, accel, mag;
  pose_get_calibration(sys, gyro, accel, mag);
  Serial.print(",cal="); Serial.print(sys); Serial.print("/"); Serial.print(gyro);
  Serial.print("/"); Serial.print(accel); Serial.print("/"); Serial.println(mag);

  // Distances in mm; "-" = no usable reading (not fitted, failed, timed out or stale).
  Serial.print("TOF");
  for (int i = 0; i < TOF_TOTAL_COUNT; ++i) {
    Serial.print(","); Serial.print(tof_name(i)); Serial.print("=");
    if (tof_reading_usable(ranges[i])) Serial.print(ranges[i].range_mm);
    else Serial.print("-");
  }
  // Ultrasounds on the same line, also in mm.
  for (uint8_t i = 0; i < US_COUNT; ++i) {
    uint16_t mm;
    Serial.print(","); Serial.print(ultrasound_name(i)); Serial.print("=");
    if (ultrasound_read_mm(i, mm)) Serial.print(mm);
    else Serial.print("-");
  }
  // Surface colour last, with raw counts for calibration.
  Serial.print(",COL="); Serial.print(color_name(color_sensor_color()));
  uint16_t r, g, b, c;
  Serial.print(",RGBC=");
  if (color_sensor_raw(r, g, b, c)) {
    Serial.print(r); Serial.print("/"); Serial.print(g); Serial.print("/");
    Serial.print(b); Serial.print("/"); Serial.print(c);
  } else Serial.print("-");
  Serial.println();
}

// Set true to print normal-loop averages/maxima once a second.
enum LoopStage { LS_COLLECTION, LS_DECAY, LS_POSE, LS_DEBUG, LS_TOF, LS_MAP,
                 LS_DRIVE, LS_SERIAL, LS_DELAY, LS_COUNT };
#if ENABLE_BENCH_TESTS
static const bool PRINT_LOOP_TIMING = false;
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
  if (!PRINT_LOOP_TIMING || millis() - timing_last_report_ms < 1000) return;
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
#else
static void time_stage(LoopStage, uint32_t &) {}
static void report_loop_timing(uint32_t) {}
#endif

static bool collection_motion_active() {
  const WeightState state = weight_collection_state();
  return state == WC_PICKUP || state == WC_CLEARING || state == WC_RELEASING;
}

#if ENABLE_BENCH_TESTS
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

// Bench test ('W'): spin on the spot at the minimum motor power, each way, and
// report the turn rate - sizes the scan speed against the mapping turn-rate gate.
static const int      SPIN_TEST_PCT        = 65;    // = MIN_EFFECTIVE_PCT in motors.cpp
static const uint32_t SPIN_TEST_SPINUP_MS  = 500;   // ignored while the motors spin up
static const uint32_t SPIN_TEST_MEASURE_MS = 3000;
static const uint32_t SPIN_TEST_SAMPLE_MS  = 50;
static const uint32_t SPIN_TEST_SETTLE_MS  = 400;   // coast to a stop before the end position

static float wrap_pi(float angle_rad) {
  while (angle_rad > PI)  angle_rad -= 2.0f * PI;
  while (angle_rad < -PI) angle_rad += 2.0f * PI;
  return angle_rad;
}

// dir: +1 = anticlockwise (turn left), -1 = clockwise (turn right).
static void spin_test_direction(int dir) {
  const char *name = dir > 0 ? "CCW" : "CW";
  // A spin on the spot should not move x/y; any change shows flow-offset error.
  const Pose before = pose_update();
  set_motors(-dir * SPIN_TEST_PCT, dir * SPIN_TEST_PCT);
  const uint32_t start_ms = millis();
  bool have_prev = false;
  float prev_theta = 0.0f, total_rad = 0.0f;
  float min_rate = 1e9f, max_rate = -1e9f;
  uint32_t prev_ms = 0, first_ms = 0;
  int samples = 0;

  while (millis() - start_ms < SPIN_TEST_SPINUP_MS + SPIN_TEST_MEASURE_MS) {
    const Pose p = pose_update();
    const uint32_t now = millis();
    if (now - start_ms >= SPIN_TEST_SPINUP_MS && pose_heading_valid()) {
      if (have_prev && now > prev_ms) {
        const float d = wrap_pi(p.theta - prev_theta);
        total_rad += d;
        const float rate = fabsf(d) / ((now - prev_ms) * 1e-3f);
        if (rate < min_rate) min_rate = rate;
        if (rate > max_rate) max_rate = rate;
      } else if (!have_prev) {
        first_ms = now;
      }
      have_prev = true;
      prev_theta = p.theta;
      prev_ms = now;
      ++samples;
      Serial.print("SPIN,"); Serial.print(name); Serial.print(",t_ms=");
      Serial.print(now - start_ms); Serial.print(",theta_deg="); Serial.println(degrees(p.theta), 1);
    }
    delay(SPIN_TEST_SAMPLE_MS);
  }
  stop_motors();
  const uint32_t stop_ms = millis();
  Pose after = before;
  while (millis() - stop_ms < SPIN_TEST_SETTLE_MS) { after = pose_update(); delay(SPIN_TEST_SAMPLE_MS); }

  const float secs = (prev_ms - first_ms) * 1e-3f;
  Serial.print("SPIN_SUMMARY,dir="); Serial.print(name);
  Serial.print(",power_pct="); Serial.print(SPIN_TEST_PCT);
  Serial.print(",samples="); Serial.print(samples);
  Serial.print(",moved_mm="); Serial.print(1000.0f * hypotf(after.x - before.x, after.y - before.y), 0);
  Serial.print(",dx_mm="); Serial.print(1000.0f * (after.x - before.x), 0);
  Serial.print(",dy_mm="); Serial.print(1000.0f * (after.y - before.y), 0);
  Serial.print(",total_turn_deg="); Serial.print(degrees(wrap_pi(after.theta - before.theta)), 1);
  if (samples < 2 || secs <= 0.0f) {
    Serial.println(",result=NO VALID HEADING - check IMU");
    return;
  }
  const float avg = fabsf(total_rad) / secs;
  Serial.print(",avg_rad_s="); Serial.print(avg, 3);
  Serial.print(",avg_deg_s="); Serial.print(degrees(avg), 1);
  Serial.print(",min_rad_s="); Serial.print(min_rate, 3);
  Serial.print(",max_rad_s="); Serial.print(max_rate, 3);
  Serial.print(",turned_deg="); Serial.print(degrees(total_rad), 1);
  Serial.print(",mapping_gate_rad_s="); Serial.print(TOF_MAX_TURN_RATE_RAD_S, 2);
  Serial.println(max_rate <= TOF_MAX_TURN_RATE_RAD_S ? ",maps_while_spinning=yes" : ",maps_while_spinning=NO");
}

static void run_spin_test() {
  if (weight_collection_busy()) {
    Serial.println("SPIN test refused: weight collection is busy");
    return;
  }
  Serial.println("SPIN test starting - robot will turn on the spot both ways");
  spin_test_direction(+1);
  const uint32_t pause_ms = millis();
  while (millis() - pause_ms < 1000) { pose_update(); delay(SPIN_TEST_SAMPLE_MS); }
  spin_test_direction(-1);
  Serial.println("SPIN test done");
}

#endif // ENABLE_BENCH_TESTS

// 'M': resend the whole map (the visualiser asks on connect; cells are otherwise
//      only sent when they change, so anything sent earlier would be missed).
// 'R': restart, bench testing only.
// 'W': spin-rate test (see run_spin_test), bench testing only.
// 'G': start the round as if the button were pressed, bench testing only.
// 'F': 500 mm optical-flow drive test; 'O': full-turn compensation tests.
// 'P': motors-off raw-flow manual push test. F/O/P require ROUND_WAITING.
// Send X to stop an active flow test.
// After a flow test, output stays paused. C resumes pre-round monitoring.
static void check_serial_commands() {
  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == 'M') {
      map_publish_all();
      exploration_publish_all();
    }
#if ENABLE_BENCH_TESTS
    else if ((c == 'F' || c == 'O' || c == 'P') && testing) {
      if (round_phase == ROUND_WAITING) {
        bench_output_paused = false;
        flow_test_run(c);
        bench_output_paused = true;
        Serial.println("SERIAL PAUSED - copy the summary above. C resumes; P/F/O runs another test.");
      }
      else Serial.println("FLOW_TEST,refused=reset and test before starting the round");
    } else if ((c == 'X' || c == 'x') && testing && round_phase == ROUND_WAITING) {
      bench_output_paused = true;
      Serial.println("SERIAL PAUSED - C resumes; P/F/O runs another test.");
    } else if (c == 'C' && testing && round_phase == ROUND_WAITING) {
      bench_output_paused = false;
      Serial.println("Pre-round monitoring resumed");
    } else if (c == 'G' && testing) {
      start_requested = true;
    } else if (c == 'W' && testing) {
      run_spin_test();
    } else if (c == 'R' && testing) {
      Serial.println("Restarting...");
      delay(50);   // let the message actually get out before resetting
      cpu_restart();
    }
#endif
  }
}

// Start button, sampled every few ms. A start needs, in order:
//   ARMING  - released continuously for START_ARM_MS (a flickering pin never
//             gets past this),
//   ARMED   - pressed continuously for START_PRESS_MS,
//   PRESSED - released continuously for START_RELEASE_MS -> start (hands clear).
// Any reading that breaks a stage's requirement drops back a stage.
enum StartStage { START_ARMING, START_ARMED, START_PRESSED };

static const char *start_stage_name(StartStage s) {
  switch (s) {
    case START_ARMING:  return "ARMING";
    case START_ARMED:   return "ARMED";
    case START_PRESSED: return "PRESSED";
  }
  return "?";
}

// Learn the untouched level: the majority reading over START_LEARN_MS.
// (Don't hold the button while the robot boots.)
static int learn_button_idle_level() {
  uint32_t samples = 0, lows = 0;
  const uint32_t start_ms = millis();
  while (millis() - start_ms < START_LEARN_MS) {
    ++samples;
    if (digitalRead(START_BUTTON_PIN) == LOW) ++lows;
    delay(2);
  }
  return 2 * lows > samples ? LOW : HIGH;
}

// Blocks until the start button is pressed and released (or 'G' in testing).
// Sensors stay live and telemetry keeps flowing for pre-round checks, but
// nothing is mapped and nothing moves. Returns how the round was started.
static const char *wait_for_start() {
  Serial.println("=== WAITING FOR START BUTTON ===");
  const int idle_level = learn_button_idle_level();
  Serial.print("START_BUTTON idle level learned: "); Serial.print(idle_level == LOW ? "LOW" : "HIGH");
  Serial.print(" - a press reads "); Serial.println(idle_level == LOW ? "HIGH" : "LOW");
  StartStage stage = START_ARMING;
  uint32_t stage_since_ms = millis();
  uint32_t last_sensor_ms = 0, last_status_ms = 0;
  uint32_t samples = 0, pressed_samples = 0;  // per status period, to expose a noisy pin

  while (true) {
    check_serial_commands();
#if ENABLE_BENCH_TESTS
    if (start_requested) return "serial G";
    if (bench_output_paused) {
      // Keep odometry current without ToF retries, colour events or button
      // diagnostics scrolling the result away. C re-arms the start button.
      stop_motors();
      last_pose = pose_update();
      stage = START_ARMING;
      stage_since_ms = millis();
      samples = pressed_samples = 0;
      delay(20);
      continue;
    }
#endif
    ultrasound_update();
    color_sensor_update();

    const uint32_t now = millis();
    const int level = digitalRead(START_BUTTON_PIN);
    const bool pressed = level != idle_level;
    ++samples;
    if (pressed) ++pressed_samples;

    switch (stage) {
      case START_ARMING:
        if (pressed) stage_since_ms = now;  // restart the released-for-500 ms count
        else if (now - stage_since_ms >= START_ARM_MS) { stage = START_ARMED; stage_since_ms = now; }
        break;
      case START_ARMED:
        if (!pressed) stage_since_ms = now;  // waiting for a continuous press
        else if (now - stage_since_ms >= START_PRESS_MS) { stage = START_PRESSED; stage_since_ms = now; }
        break;
      case START_PRESSED:
        if (pressed) stage_since_ms = now;  // still held (or bounced): keep waiting
        else if (now - stage_since_ms >= START_RELEASE_MS) return "button";
        break;
    }

    // Sensors and telemetry at the normal ~20 ms cadence; the button is sampled faster.
    if (now - last_sensor_ms >= 20) {
      last_sensor_ms = now;
      last_pose = pose_update();
      tof_read(ranges);
      print_telemetry(0, 0);
    }
    if (now - last_status_ms >= 1000) {
      last_status_ms = now;
      // Untouched should read pressed_pct=0; anything else means a noisy pin.
      Serial.print("START_BUTTON,raw="); Serial.print(level == LOW ? 0 : 1);
      Serial.print(",pressed_pct="); Serial.print(samples ? 100 * pressed_samples / samples : 0);
      Serial.print(",stage="); Serial.println(start_stage_name(stage));
      samples = pressed_samples = 0;
    }
    delay(2);
  }
}

// Fresh round from the configured start layout (arena_config.h).
static void start_round(const char *source) {
  pose_reset();
  pose_set_position(ROBOT_START_X_M, ROBOT_START_Y_M);
  pose_set_heading(radians(ROBOT_START_HEADING_DEG));
  last_pose = pose_update();
  map_init();
  exploration_init();
  state_machine_init();
  for (auto &reading : ranges) tof_reading_clear(reading);
  round_start_ms = millis();
  round_phase = ROUND_RUNNING;
  home_square_begin_round();  // flow ignored until the black line
  soft_start_begin();
  Serial.print("=== ROUND START ("); Serial.print(source);
  Serial.print(") === x="); Serial.print(ROBOT_START_X_M, 2);
  Serial.print(" y="); Serial.print(ROBOT_START_Y_M, 2);
  Serial.print(" heading_deg="); Serial.println(ROBOT_START_HEADING_DEG, 0);
}

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
  ultrasound_init();
  ultrasound_attach_interrupt_left(on_ultrasound_interrupt);
  ultrasound_attach_interrupt_right(on_ultrasound_interrupt);
  color_sensor_init();  // after pose_init(): shares Wire1 with the IMU
  arms_init();
  electromagnets_init();
  weight_collection_init();   // parks crane at idle, arms at rest

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  state_machine_init();

  Serial.println("=== SETUP COMPLETE ===");
  start_round(wait_for_start());
}

void loop() {
  check_serial_commands();
  ultrasound_update();
  color_sensor_update();

  if (round_phase == ROUND_RUNNING && millis() - round_start_ms >= ROUND_LENGTH_MS) {
    round_phase = ROUND_OVER;
    stop_motors();
    Serial.println("=== ROUND OVER - motors and arms halted ===");
  }
  if (round_phase == ROUND_OVER) {
    // No drive, no collection updates: the crane and arm servos hold where they
    // are and the magnets stay as they were. Telemetry continues until reset.
    stop_motors();
    print_telemetry(0, 0);
    delay(100);
    return;
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
    for (auto &reading : ranges) tof_reading_clear(reading);
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
    print_telemetry(left_pct, right_pct);
    collection_was_active = true;
    // Fast crane update cadence; no pose/ToF/map diagnostics in this path.
    delay(10);
    return;
  }
  time_stage(LS_COLLECTION, timing_mark);

  // --- Localisation ---
  map_decay();
  time_stage(LS_DECAY, timing_mark);
  Pose pose = pose_update();
  if (home_square_update(pose)) pose = pose_current();  // black-line position fix
  last_pose = pose;
  exploration_mark_footprint(pose);  // ground under the robot has been searched
  time_stage(LS_POSE, timing_mark);
  if (!pose_imu_ready() || !pose_flow_ready()) {
    stop_motors();
    static uint32_t last_warn = 0;
    if (millis() - last_warn > 1000) {
      last_warn = millis();
      Serial.println("SENSOR NOT READY - motors held off");
    }
    print_telemetry(0, 0);
    time_stage(LS_DEBUG, timing_mark);
    delay(20);
    time_stage(LS_DELAY, timing_mark);
    report_loop_timing(loop_start_us);
    return;
  }
  // Stop promptly on timeout, but keep collection and sensor updates running.
  if (pose_heading_stale()) stop_motors();
  time_stage(LS_DEBUG, timing_mark);
  // --- Perception ---
  tof_read(ranges);
  if (discard_tof_on_resume) {
    // The first ready result may have been buffered during collection.
    for (auto &reading : ranges) tof_reading_clear(reading);
    discard_tof_on_resume = false;
  }
  time_stage(LS_TOF, timing_mark);
  if (recovery_active && millis() - recovery_start_ms >= 150 && pose_heading_valid()) {
    // Only the front pairs drive navigation and mapping; a dead or unfitted
    // side sensor must not hold the robot stopped forever.
    static const uint8_t RECOVERY_TOFS[] = {TOF_LFT, TOF_LFB, TOF_RFB, TOF_RFT};
    bool all_fresh = true;
    for (uint8_t i : RECOVERY_TOFS) all_fresh &= tof_reading_live(ranges[i]);
    if (all_fresh) recovery_active = false;
  }
  // Blind approach: front ToFs ignored for mapping, sightings and obstacle holds.
  const bool front_blind = state_machine_front_blind();
  tof_set_front_blind(front_blind);
  const bool weight_seen_this_frame = !recovery_active && tof_classify_readings(ranges, pose);
  if (USE_WALL_ANCHOR && !recovery_active) {
    wall_anchor_update(ranges, pose, front_blind);
    pose = pose_current();  // pick up any correction
    last_pose = pose;
  }
  time_stage(LS_MAP, timing_mark);

  // Retain the previous command through a brief heading-read failure.
  int obstacle_left = 0, obstacle_right = 0;
  const bool obstacle_data_missing = !front_blind &&
      tof_nav_update(ranges, obstacle_left, obstacle_right) == TOF_NAV_UNKNOWN;

  if (pose_heading_stale()) {
    left_pct = right_pct = 0;
    stop_motors();
    static uint32_t last_heading_warn_ms = 0;
    if (millis() - last_heading_warn_ms >= 1000) {
      last_heading_warn_ms = millis();
      Serial.println("IMU HEADING TIMEOUT - motors held off; retrying");
    }
  } else if (!weight_collection_busy() && us_escape_active()) {
    left_pct = us_escape_left_pct;
    right_pct = us_escape_right_pct;
    set_motors(left_pct, right_pct);
  } else if (recovery_active || obstacle_data_missing) {
    // Also hold scan/verification turns when the obstacle sensors are unknown.
    left_pct = right_pct = 0;
    stop_motors();
  } else if (weight_collection_feeding()) {
    // Keep the object moving into the funnel without navigation turning away.
    // Blind: the front ToFs misread this close, so they cannot stop feeding.
    // If induction never fires, the clearing arm sweeps after the timeout.
    left_pct = right_pct = FUNNEL_FEED_SPEED_PCT;
    set_motors(left_pct, right_pct);
  } else if (weight_collection_busy()) {
    // Navigation stays paused throughout feeding, pickup, and arm movement.
    left_pct = right_pct = 0;
    stop_motors();
  } else if (pose_heading_valid() && soft_start_drive(ranges, pose, left_pct, right_pct)) {
    set_motors(left_pct, right_pct);
  } else if (pose_heading_valid()) {
    state_machine_update(pose, ranges, weight_seen_this_frame, left_pct, right_pct);
    set_motors(left_pct, right_pct);
  } else {
    // Do not verify targets or alter the map using a failed heading sample.
    // Keep obstacle avoidance active during the short grace period (not while blind).
    int avoid_left = 0, avoid_right = 0;
    if (!front_blind && tof_nav_update(ranges, avoid_left, avoid_right) != TOF_NAV_CLEAR) {
      left_pct = avoid_left;
      right_pct = avoid_right;
    }
    set_motors(left_pct, right_pct);
  }

  time_stage(LS_DRIVE, timing_mark);

  map_publish_changes();
  exploration_publish_changes();
  print_telemetry(left_pct, right_pct);
  time_stage(LS_SERIAL, timing_mark);
  delay(20); // nominal pause, not the total loop duration
  time_stage(LS_DELAY, timing_mark);
  report_loop_timing(loop_start_us);
}
