#include "tof_nav.h"
#include <Arduino.h>
#include "grid_map.h"
#include "pose.h"

static const float MAX_SAFE_TURN_RATE_RAD_S = 0.52f;

// Relevant constants
static const float MM_TO_M = 0.001f;
static const float WALL_TOLERANCE_M = 0.12f;
static const float DETECTION_MAX_M = (float)(TOF_MAX_RANGE_MM - 50) * MM_TO_M;

// Use the named hardware indices from TOFs.h.

// TOF offsets (metres, body frame: forward, left)
static const float TOP_LEFT_FWD_M    =  0.1506f;
static const float TOP_LEFT_LEFT_M   =  0.06164f;
static const float BOTTOM_LEFT_FWD_M =  0.174f;
static const float BOTTOM_LEFT_LEFT_M=  0.06164f;

static const float TOP_RIGHT_FWD_M    =  0.1506f;
static const float TOP_RIGHT_LEFT_M   = -0.08975f;
static const float BOTTOM_RIGHT_FWD_M =  0.174f;
static const float BOTTOM_RIGHT_LEFT_M= -0.08975f;

// --- Reflex-only constants ---
// Tighter than DETECTION_MAX_M on purpose - this is a last-resort override,
// not the primary navigation signal. Tune against real hardware.
static const uint16_t IMMINENT_MM = 150;
static const int AVOID_SPEED = 40;   // spin-in-place speed while avoiding

static void project_point(const Pose &pose, float fwd_offset, float left_offset, float distance, float &x_m, float &y_m) {
  float body_fwd = fwd_offset + distance;
  float body_left = left_offset;

  float c = cosf(pose.theta);
  float s = sinf(pose.theta);

  x_m = pose.x + body_fwd * c - body_left * s;
  y_m = pose.y + body_fwd * s + body_left * c;
}

struct PairObservation {
  bool usable = false;
  bool weight = false;
  uint32_t bottom_ms = 0, top_ms = 0;
  float x = 0, y = 0;
};
static PairObservation observations[2];

static bool after_settling(const PairObservation &o, uint32_t after_ms) {
  return o.usable && (int32_t)(o.bottom_ms - after_ms) > 0 &&
         (int32_t)(o.top_ms - after_ms) > 0;
}
bool tof_observation_after(uint32_t after_ms) {
  return after_settling(observations[0], after_ms) || after_settling(observations[1], after_ms);
}
bool tof_confirm_target(float x, float y, uint32_t after_ms, float &mx, float &my) {
  const float match_radius_m = 0.10f; // allow small pose/range and grid quantisation errors
  for (const auto &o : observations) {
    if (!after_settling(o, after_ms) || !o.weight) continue;
    const float dx = o.x - x, dy = o.y - y;
    if (dx * dx + dy * dy <= match_radius_m * match_radius_m) {
      mx = o.x; my = o.y;
      return true;
    }
  }
  return false;
}

static bool classification_complete = false;

bool tof_classification_complete() { return classification_complete; }

static bool consume_pair(const TofReading ranges[TOF_TOTAL_COUNT], uint8_t bottom, uint8_t top) {
  static uint32_t consumed[TOF_TOTAL_COUNT] = {};
  if (ranges[bottom].sequence == consumed[bottom] ||
      ranges[top].sequence == consumed[top]) return false;
  if (!tof_reading_usable(ranges[bottom]) || !tof_reading_usable(ranges[top])) return false;
  const uint32_t now = millis();
  const uint32_t bottom_age = now - ranges[bottom].received_ms;
  const uint32_t top_age = now - ranges[top].received_ms;
  const uint32_t skew = bottom_age > top_age ? bottom_age - top_age : top_age - bottom_age;
  if (skew > TOF_PAIR_MAX_SKEW_MS) return false;
  consumed[bottom] = ranges[bottom].sequence;
  consumed[top] = ranges[top].sequence;
  return true;
}

static bool classify_pair(const TofReading &bottom, const TofReading &top,
                          float bottom_fwd, float top_fwd, float bottom_left, float top_left, const Pose &pose, PairObservation &observation) {
  const float bottom_range = bottom.range_mm * MM_TO_M;
  const float top_range = top.range_mm * MM_TO_M;
  // Both endpoints measured from the same robot reference, not sensor faces.
  const float clearance = top_fwd + top_range - bottom_fwd - bottom_range;
  const bool lower_hit = bottom_range < DETECTION_MAX_M;
  const bool upper_hit = top_range < DETECTION_MAX_M;
  const bool collectable = lower_hit && clearance > WALL_TOLERANCE_M;
  float ox, oy, bx, by, tx, ty;
  int bgx, bgy, tgx, tgy;
  project_point(pose, bottom_fwd, bottom_left, 0, ox, oy);
  project_point(pose, bottom_fwd, bottom_left, min(bottom_range, DETECTION_MAX_M), bx, by);
  map_ray_trace(ox, oy, bx, by, false);
  const bool bottom_inside = world_to_grid(bx, by, bgx, bgy);
  if (bottom_inside) {
    if (collectable) map_observe_weight(bgx, bgy);
    else if (lower_hit) map_reject_weight(bgx, bgy);
    else map_observe_free(bgx, bgy);
  }
  project_point(pose, top_fwd, top_left, 0, ox, oy);
  project_point(pose, top_fwd, top_left, min(top_range, DETECTION_MAX_M), tx, ty);
  map_ray_trace(ox, oy, tx, ty, true);
  if (world_to_grid(tx, ty, tgx, tgy)) {
    if (upper_hit) map_observe_wall(tgx, tgy);
    else map_observe_free(tgx, tgy, true);
  }
  observation.usable = true;
  observation.bottom_ms = bottom.received_ms;
  observation.top_ms = top.received_ms;
  observation.weight = collectable && bottom_inside && map_get_cell(bgx, bgy) == MAP_CELL_WEIGHT;
  if (observation.weight) grid_to_world(bgx, bgy, observation.x, observation.y);
  return observation.weight;
}

bool tof_classify_readings(const TofReading ranges[TOF_TOTAL_COUNT], const Pose &pose) {
  classification_complete = false;
  for (auto &o : observations) o.usable = false;
  const bool right_ready = consume_pair(ranges, TOF_RFB, TOF_RFT);
  const bool left_ready = consume_pair(ranges, TOF_LFB, TOF_LFT);
  if (!pose_heading_valid() || !isfinite(angular_speed_rad_s()) ||
      fabsf(angular_speed_rad_s()) > MAX_SAFE_TURN_RATE_RAD_S) return false;
  classification_complete = right_ready && left_ready;
  bool found = false;
  if (right_ready) found |= classify_pair(ranges[TOF_RFB], ranges[TOF_RFT],
      BOTTOM_RIGHT_FWD_M, TOP_RIGHT_FWD_M, BOTTOM_RIGHT_LEFT_M, TOP_RIGHT_LEFT_M, pose, observations[0]);
  if (left_ready) found |= classify_pair(ranges[TOF_LFB], ranges[TOF_LFT],
      BOTTOM_LEFT_FWD_M, TOP_LEFT_FWD_M, BOTTOM_LEFT_LEFT_M, TOP_LEFT_LEFT_M, pose, observations[1]);
  return found;
}

TofNavState tof_nav_update(const TofReading ranges[TOF_TOTAL_COUNT], int &left_pct, int &right_pct) {
  const bool right_valid = tof_reading_usable(ranges[TOF_RFT]);
  const bool left_valid = tof_reading_usable(ranges[TOF_LFT]);
  left_pct = right_pct = 0;
  // Missing obstacle data is explicitly unknown, not clear. Hold for now;
  // broader degraded-sensor driving behaviour is a separate task.
  if (!right_valid || !left_valid) return TOF_NAV_UNKNOWN;
  bool r = ranges[TOF_RFT].range_mm < IMMINENT_MM;
  bool l = ranges[TOF_LFT].range_mm < IMMINENT_MM;

  if (!r && !l) return TOF_NAV_CLEAR;

  bool turn_left;
  if (r && l) turn_left = ranges[TOF_RFT].range_mm <= ranges[TOF_LFT].range_mm;
  else        turn_left = r;

  if (turn_left) { left_pct = -AVOID_SPEED; right_pct =  AVOID_SPEED; }
  else           { left_pct =  AVOID_SPEED; right_pct = -AVOID_SPEED; }

  return TOF_NAV_AVOID;
}
