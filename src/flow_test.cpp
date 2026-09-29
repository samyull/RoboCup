#include "bench_config.h"
#if ENABLE_BENCH_TESTS
#include "flow_test.h"
#include "pose.h"
#include "motors.h"
#include "weight_collection.h"
#include <Arduino.h>

static const int POWER_PCT = 70;  // above the 65% minimum so the test drive does not stall
// Target by the robot's own pose, including the roll-on after the motors cut:
// they stop COAST_M early so the robot comes to rest at DRIVE_M. COAST_M is the
// measured final roll-on at 70% power (14-19 mm earlier runs, 25 mm last run) -
// update it from forward_mm - stop_forward_mm if POWER_PCT changes.
static const float DRIVE_M = 0.5f;
static const float COAST_M = 0.022f;
static const uint32_t TIMEOUT_MS = 15000;
static const uint32_t SETTLE_MS = 700;

static float wrap(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

// Consume commands during the test so a queued G cannot start a round later.
// Tracking quality over the samples taken while moving (see FlowDiagnostic).
struct QualityStats {
  uint32_t n = 0;
  uint32_t squal_sum = 0, shutter_sum = 0;
  uint8_t squal_min = 255;
  uint16_t shutter_max = 0;
  void add(const FlowDiagnostic &d) {
    if (!d.quality_read) return;
    ++n;
    squal_sum += d.squal;
    shutter_sum += d.shutter;
    squal_min = min(squal_min, d.squal);
    shutter_max = max(shutter_max, d.shutter);
  }
  void print() const {
    Serial.print(",quality_samples="); Serial.print(n);
    if (!n) return;
    Serial.print(",squal_min="); Serial.print(squal_min);
    Serial.print(",squal_avg="); Serial.print((float)squal_sum / n, 1);
    Serial.print(",shutter_avg="); Serial.print((float)shutter_sum / n, 0);
    Serial.print(",shutter_max="); Serial.print(shutter_max);
  }
};

static void print_quality_now(const FlowDiagnostic &d) {
  Serial.print(",squal="); Serial.print(d.squal);
  Serial.print(",shutter="); Serial.print(d.shutter);
}

static bool stop_requested() {
  bool stop = false;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 'X' || c == 'x') stop = true;
  }
  return stop;
}

static bool run_leg(int direction) {
  // direction 0 = straight; +1 = CCW; -1 = CW.
  const char *name = direction == 0 ? "DRIVE" : direction > 0 ? "CCW" : "CW";
  stop_motors();
  Pose p = pose_update();
  if (!pose_heading_valid()) {
    Serial.println("FLOW_TEST,refused=heading unavailable");
    return false;
  }
  const Pose origin = p;
  Pose previous = p;
  const FlowDiagnostic at_rest = pose_flow_diagnostic();
  QualityStats moving;
  float turned = 0, peak = 0, path = 0, max_heading_error = 0;
  const uint32_t start = millis();
  uint32_t last_print = start, stopped_at = 0;
  bool stopped = false;
  const char *result = "OK";
  float stop_forward = 0;
  Serial.print("FLOW_BEGIN,test="); Serial.print(name);
  Serial.print(",meters_per_count="); Serial.print(FLOW_METERS_PER_COUNT, 9);
  Serial.print(",power_pct="); Serial.print(POWER_PCT);
  Serial.print(",target_mm="); Serial.print(DRIVE_M * 1000, 0);
  Serial.println(",turn_target_deg=360");

  while (true) {
    const bool abort = stop_requested();
    p = pose_update();
    const FlowDiagnostic d = pose_flow_diagnostic();
    if (!stopped) moving.add(d);
    const uint32_t now = millis();
    const float dx = p.x - origin.x, dy = p.y - origin.y;
    const float forward = dx * cosf(origin.theta) + dy * sinf(origin.theta);
    const float lateral = -dx * sinf(origin.theta) + dy * cosf(origin.theta);
    turned += wrap(p.theta - previous.theta);
    path += hypotf(p.x - previous.x, p.y - previous.y);
    peak = max(peak, hypotf(dx, dy));
    max_heading_error = max(max_heading_error, fabsf(wrap(p.theta - origin.theta)));
    previous = p;

    if (abort || !pose_heading_valid()) {
      result = abort ? "ABORTED" : "HEADING_FAILURE";
      stop_motors();
      if (!stopped) { stopped = true; stopped_at = now; stop_forward = forward; }
    }
    if (!stopped) {
      const bool reached = direction == 0 ? forward >= DRIVE_M - COAST_M
                                          : direction * turned >= 2.0f * PI;
      if (reached || now - start >= TIMEOUT_MS) {
        if (!reached) result = "TIMEOUT";
        stop_motors();
        stopped = true;
        stopped_at = now;
        stop_forward = forward;
      } else if (direction) {
        set_motors(-direction * POWER_PCT, direction * POWER_PCT);
      } else {
        // Hold the starting IMU heading. Inner track stays above minimum power.
        const int correction = constrain((int)(40.0f * wrap(origin.theta - p.theta)), -20, 20);
        set_motors(POWER_PCT + max(0, -correction), POWER_PCT + max(0, correction));
      }
    }

    if (now - last_print >= 100) {
      last_print = now;
      Serial.print("FLOW_SAMPLE,test="); Serial.print(name);
      Serial.print(",t_ms="); Serial.print(now - start);
      Serial.print(",forward_mm="); Serial.print(forward * 1000, 1);
      Serial.print(",left_mm="); Serial.print(lateral * 1000, 1);
      Serial.print(",turn_deg="); Serial.print(degrees(turned), 1);
      print_quality_now(d);
      Serial.println();
    }
    if (stopped && now - stopped_at >= SETTLE_MS) {
      Serial.print("FLOW_SUMMARY,test="); Serial.print(name);
      Serial.print(",result="); Serial.print(result);
      Serial.print(",meters_per_count="); Serial.print(FLOW_METERS_PER_COUNT, 9);
      Serial.print(",forward_mm="); Serial.print(forward * 1000, 1);
      Serial.print(",left_mm="); Serial.print(lateral * 1000, 1);
      Serial.print(",stop_forward_mm="); Serial.print(stop_forward * 1000, 1);
      Serial.print(",final_displacement_mm="); Serial.print(hypotf(dx, dy) * 1000, 1);
      Serial.print(",peak_displacement_mm="); Serial.print(peak * 1000, 1);
      Serial.print(",pose_path_mm="); Serial.print(path * 1000, 1);
      Serial.print(",turn_deg="); Serial.print(degrees(turned), 1);
      Serial.print(",max_heading_error_deg="); Serial.print(degrees(max_heading_error), 1);
      Serial.print(",rest_squal="); Serial.print(at_rest.squal);
      Serial.print(",rest_shutter="); Serial.print(at_rest.shutter);
      moving.print();  // while driving (before the stop command)
      Serial.println();
      return result[0] == 'O';
    }
    delay(20);
  }
}

static void run_manual_push() {
  stop_motors();
  pose_set_stationary_filter(false);  // motors are off on purpose: count the push
  const Pose origin = pose_update();
  const uint32_t start = millis();
  uint32_t last_print = start, samples = 0, zero_samples = 0, integrated = 0;
  int32_t sum_x = 0, sum_y = 0;
  uint32_t abs_x = 0, abs_y = 0;
  float sensor_fwd = 0, sensor_left = 0, corr_fwd = 0, corr_left = 0;
  QualityStats moving;  // samples with motion only, so standing still doesn't hide drops
  Serial.print("FLOW_BEGIN,test=PUSH,meters_per_count=");
  Serial.print(FLOW_METERS_PER_COUNT, 9);
  Serial.println(",motors=OFF; push now, stop moving then send X; automatic end at 30 seconds");
  while (true) {
    const bool finish = stop_requested() || millis() - start >= 30000;
    const Pose p = pose_update();
    const FlowDiagnostic d = pose_flow_diagnostic();
    if (d.read) {
      ++samples;
      if (!d.raw_x && !d.raw_y) ++zero_samples;
      else moving.add(d);
      sum_x += d.raw_x; sum_y += d.raw_y;
      abs_x += abs((int)d.raw_x); abs_y += abs((int)d.raw_y);
    }
    if (d.integrated) ++integrated;
    sensor_fwd += d.sensor_forward_m; sensor_left += d.sensor_left_m;
    corr_fwd += d.correction_forward_m; corr_left += d.correction_left_m;
    const uint32_t now = millis();
    if (finish || now - last_print >= 100) {
      last_print = now;
      const float dx = p.x - origin.x, dy = p.y - origin.y;
      Serial.print(finish ? "FLOW_SUMMARY,test=PUSH" : "FLOW_RAW,test=PUSH");
      Serial.print(",t_ms="); Serial.print(now - start);
      Serial.print(",raw_x="); Serial.print(d.raw_x);
      Serial.print(",raw_y="); Serial.print(d.raw_y);
      Serial.print(",sum_x="); Serial.print(sum_x);
      Serial.print(",sum_y="); Serial.print(sum_y);
      Serial.print(",abs_x="); Serial.print(abs_x);
      Serial.print(",abs_y="); Serial.print(abs_y);
      Serial.print(",samples="); Serial.print(samples);
      Serial.print(",zero_samples="); Serial.print(zero_samples);
      Serial.print(",integrated_samples="); Serial.print(integrated);
      Serial.print(",sensor_forward_mm="); Serial.print(sensor_fwd * 1000, 1);
      Serial.print(",sensor_left_mm="); Serial.print(sensor_left * 1000, 1);
      Serial.print(",correction_forward_mm="); Serial.print(corr_fwd * 1000, 1);
      Serial.print(",correction_left_mm="); Serial.print(corr_left * 1000, 1);
      Serial.print(",pose_forward_mm="); Serial.print((dx * cosf(origin.theta) + dy * sinf(origin.theta)) * 1000, 1);
      Serial.print(",pose_left_mm="); Serial.print((-dx * sinf(origin.theta) + dy * cosf(origin.theta)) * 1000, 1);
      Serial.print(",heading_valid="); Serial.print(pose_heading_valid());
      Serial.print(",heading_change_deg="); Serial.print(degrees(wrap(p.theta - origin.theta)), 1);
      if (finish) moving.print();
      else print_quality_now(d);
      Serial.println();
    }
    if (finish) break;
    delay(20);
  }
  pose_set_stationary_filter(true);
}

void flow_test_run(char command) {
  if (weight_collection_busy() || !pose_imu_ready() || !pose_flow_ready()) {
    Serial.println("FLOW_TEST,refused=collection busy or pose sensors unavailable");
    return;
  }
  Serial.println("FLOW_TEST: starts in 2 seconds; X stops. Mapping and wall correction disabled during test.");
  stop_motors();
  const uint32_t start = millis();
  while (millis() - start < 2000) {
    if (stop_requested()) { Serial.println("FLOW_TEST,ABORTED"); return; }
    pose_update();
    delay(20);
  }
  if (command == 'P') run_manual_push();
  else if (command == 'F') run_leg(0);
  else if (run_leg(+1)) run_leg(-1);
  stop_motors();
  Serial.println("FLOW_TEST,DONE");
}
#endif // ENABLE_BENCH_TESTS
