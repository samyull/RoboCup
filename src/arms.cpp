#include "arms.h"
#include "electromagnets.h"
#include <Arduino.h>
#include <Servo.h>

Servo release_arm;
Servo clearing_arm;
Servo crane;

// RDS5160 uses a 500-2500us pulse range (0-180 deg), wider than the
// Servo library default of 544-2400us.
static const int CRANE_US_MIN = 500;
static const int CRANE_US_MAX = 2500;

void arms_init() {
  release_arm.attach(RELEASE_ARM_PIN);
  clearing_arm.attach(CLEARING_ARM_PIN);
  crane.attach(CRANE_PIN, CRANE_US_MIN, CRANE_US_MAX);
  Serial.println("Arms initialised");
}

static int clamp_angle(int deg) {
  if (deg < 0) return 0;
  if (deg > 180) return 180;
  return deg;
}

void set_release_arm_angle(int degrees) {
  release_arm.write(clamp_angle(degrees));
}

void set_clearing_arm_angle(int degrees) {
  clearing_arm.write(clamp_angle(degrees));
}

// Crane position is tracked as a float so slow moves can step by fractions
// of a degree each update. -1 means unknown (nothing written since boot).
static float crane_pos = -1.0f;
static float crane_target = 0.0f;
static float crane_speed = 0.0f;   // deg/s
static uint32_t crane_last_ms = 0;

// Temporary command-timing diagnostics. Only print after a move completes,
// so serial output does not disturb the updates being measured.
static const bool CRANE_TIMING_ENABLED = true;
static bool crane_timing_active = false;
static uint32_t crane_timing_last_us = 0;
static uint64_t crane_timing_total_us = 0;
static uint32_t crane_timing_samples = 0;
static uint32_t crane_timing_max_us = 0;
static uint32_t crane_timing_over_20ms = 0;
static float crane_timing_start_angle = 0.0f;
static float crane_timing_max_step_deg = 0.0f;

static void finish_crane_timing() {
  if (!crane_timing_active || crane_timing_samples == 0) return;
  crane_timing_active = false;
  Serial.print("Crane timing: from="); Serial.print(crane_timing_start_angle, 1);
  Serial.print(" to="); Serial.print(crane_target, 1);
  Serial.print(" speed_deg_s="); Serial.print(crane_speed, 1);
  Serial.print(" updates="); Serial.print(crane_timing_samples);
  Serial.print(" avg_ms=");
  Serial.print((double)crane_timing_total_us / crane_timing_samples / 1000.0, 2);
  Serial.print(" max_ms="); Serial.print(crane_timing_max_us / 1000.0, 2);
  Serial.print(" gaps_over_20ms="); Serial.print(crane_timing_over_20ms);
  Serial.print(" max_step_deg="); Serial.println(crane_timing_max_step_deg, 2);
}

static void write_crane(float degrees) {
  crane_pos = degrees;
  // writeMicroseconds gives finer steps than write(), so slow moves are smooth
  int us = CRANE_US_MIN + (int)(degrees * (CRANE_US_MAX - CRANE_US_MIN) / 180.0f);
  crane.writeMicroseconds(us);
}

void set_crane_angle(int degrees) {
  crane_timing_active = false; // direct positioning cancels any measured ramp
  crane_target = clamp_angle(degrees);
  write_crane(crane_target);
}

void move_crane_to(int degrees, float deg_per_sec) {
  crane_target = clamp_angle(degrees);
  crane_speed = deg_per_sec;
  crane_last_ms = millis();
  // Position unknown on first move, so there's nothing to ramp from - jump.
  if (crane_pos < 0.0f) write_crane(crane_target);
  crane_timing_active = CRANE_TIMING_ENABLED && !crane_at_target();
  crane_timing_last_us = micros(); // include command-to-first-update latency
  crane_timing_total_us = 0;
  crane_timing_samples = 0;
  crane_timing_max_us = 0;
  crane_timing_over_20ms = 0;
  crane_timing_start_angle = crane_pos;
  crane_timing_max_step_deg = 0.0f;
}

void crane_update() {
  const uint32_t update_us = micros();
  uint32_t now = millis();
  float step = crane_speed * (now - crane_last_ms) / 1000.0f;
  crane_last_ms = now;

  if (crane_at_target()) return;

  if (crane_timing_active) {
    const uint32_t gap_us = update_us - crane_timing_last_us;
    crane_timing_last_us = update_us;
    crane_timing_total_us += gap_us;
    ++crane_timing_samples;
    if (gap_us > crane_timing_max_us) crane_timing_max_us = gap_us;
    if (gap_us > 20000) ++crane_timing_over_20ms;
  }

  const float previous_position = crane_pos;
  float error = crane_target - crane_pos;
  if (fabsf(error) <= step) {
    write_crane(crane_target);
  } else {
    write_crane(crane_pos + (error > 0 ? step : -step));
  }
  if (crane_timing_active) {
    const float actual_step = fabsf(crane_pos - previous_position);
    if (actual_step > crane_timing_max_step_deg) crane_timing_max_step_deg = actual_step;
    if (crane_at_target()) finish_crane_timing();
  }
}

bool crane_at_target() {
  return crane_pos == crane_target;
}

void move_crane_blocking(int degrees, float deg_per_sec) {
  move_crane_to(degrees, deg_per_sec);
  while (!crane_at_target()) {
    crane_update();
    delay(10);
  }
}

void crane_test() {
  Serial.println("Crane 37 deg");
  move_crane_blocking(37, 20);
  delay(500);

  Serial.println("To Releasing Position");
  move_crane_blocking(42, 15);
  delay(2000);
  Serial.println("Electromagnets OFF");
  electromagnets_off();

  Serial.println("Crane 63 deg");
  move_crane_blocking(63, 100);
  delay(1000);

  Serial.println("To Idle Position");
  move_crane_blocking(50, 100);
  delay(1000);

  Serial.println("To Picking Up Position");
  move_crane_blocking(63, 15);
  delay(1000);
  Serial.println("Electromagnets ON");
  electromagnets_on();
}
