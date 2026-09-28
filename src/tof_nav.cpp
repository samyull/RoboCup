#include "tof_nav.h"
#include <Arduino.h>
#include "grid_map.h"
#include "pose.h"
#include "exploration.h"

// A reading is ready some time after its measurement: on average about half a
// loop waiting to be polled plus half the 50 ms timing budget. Its pose is
// looked up for this long before it was read out.
static const uint32_t TOF_MEASUREMENT_LAG_MS = 45;

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

// Side-facing pairs (LSB/LST, RSB/RST). Set false to leave them out of mapping.
static const bool USE_SIDE_TOFS = true;
// Top and bottom share a position; each side faces straight out (90 deg).
static const float SIDE_FWD_M            = 0.1325f;
static const float SIDE_LEFT_LEFT_M      = 0.11175f;
static const float SIDE_RIGHT_LEFT_M     = -0.11175f;
// Beyond this the upward-angled side top ToF can start to see a weight, so a
// top hit would no longer mean "wall" (set a little short of that point).
static const float SIDE_DETECTION_MAX_M  = 0.335f;

// Field of view by ROI size (see roiSize in TOFs.cpp).
static const float FOV_16X16_RAD = radians(27.0f);  // front top sensors
static const float FOV_4X4_RAD   = radians(15.0f);  // everything else

// Free space: hits stay on the centre line (keeps narrow objects narrow), but
// "nothing closer than the reading" holds across the cone. Use a narrowed cone,
// since sensitivity falls off towards its edge, and stop short of any hit.
static const float CONE_FOV_FRACTION = 2.0f / 3.0f;
static const float CONE_HIT_MARGIN_M = 0.10f;

struct PairGeometry {
  float bottom_fwd, bottom_left, top_fwd, top_left;  // sensor positions, body frame (m)
  float facing_rad;         // beam direction, body frame: 0 = forward, +pi/2 = left
  float max_range_m;        // readings at or beyond this are "nothing there"
  // Top beam seeing nothing within range counts as clear above a bottom hit.
  // Needed when max_range_m is too short for the clearance test to pass.
  bool top_miss_is_clear;
  float bottom_fov_rad, top_fov_rad;  // full field of view of each sensor
  // The bottom cone marks ground as explored: only the front bottom sensors
  // search far enough ahead to count (a weight there would have been seen).
  bool marks_seen;
};

static const PairGeometry FRONT_RIGHT = {BOTTOM_RIGHT_FWD_M, BOTTOM_RIGHT_LEFT_M,
    TOP_RIGHT_FWD_M, TOP_RIGHT_LEFT_M, 0.0f, DETECTION_MAX_M, false, FOV_4X4_RAD, FOV_16X16_RAD, true};
static const PairGeometry FRONT_LEFT = {BOTTOM_LEFT_FWD_M, BOTTOM_LEFT_LEFT_M,
    TOP_LEFT_FWD_M, TOP_LEFT_LEFT_M, 0.0f, DETECTION_MAX_M, false, FOV_4X4_RAD, FOV_16X16_RAD, true};
static const PairGeometry SIDE_RIGHT = {SIDE_FWD_M, SIDE_RIGHT_LEFT_M,
    SIDE_FWD_M, SIDE_RIGHT_LEFT_M, -HALF_PI, SIDE_DETECTION_MAX_M, true, FOV_4X4_RAD, FOV_4X4_RAD, false};
static const PairGeometry SIDE_LEFT = {SIDE_FWD_M, SIDE_LEFT_LEFT_M,
    SIDE_FWD_M, SIDE_LEFT_LEFT_M, HALF_PI, SIDE_DETECTION_MAX_M, true, FOV_4X4_RAD, FOV_4X4_RAD, false};

// --- Reflex-only constants ---
// Tighter than DETECTION_MAX_M on purpose - this is a last-resort override,
// not the primary navigation signal. Tune against real hardware.
static const uint16_t IMMINENT_MM = 150;
static const int AVOID_SPEED = 40;   // spin-in-place speed while avoiding

static void project_point(const Pose &pose, float fwd_offset, float left_offset, float facing_rad,
                          float distance, float &x_m, float &y_m) {
  float body_fwd = fwd_offset + distance * cosf(facing_rad);
  float body_left = left_offset + distance * sinf(facing_rad);

  float c = cosf(pose.theta);
  float s = sinf(pose.theta);

  x_m = pose.x + body_fwd * c - body_left * s;
  y_m = pose.y + body_fwd * s + body_left * c;
}

enum ConeUse {
  CONE_FREE_EVIDENCE,  // free-space evidence off the centre line (left to map_ray_trace); weights protected
  CONE_MARK_SEEN,      // mark every cell in the cone as explored, centre line included
};

// Cells inside the narrowed cone, out to reach_m from the sensor.
static void cone_cells(const Pose &pose, float fwd, float left, float facing_rad,
                       float fov_rad, float reach_m, ConeUse use) {
  if (reach_m <= 0.0f) return;
  const float half = 0.5f * fov_rad * CONE_FOV_FRACTION;
  const float spread = tanf(half);
  float ox, oy;
  project_point(pose, fwd, left, facing_rad, 0, ox, oy);
  const float dir = pose.theta + facing_rad;
  const float cd = cosf(dir), sd = sinf(dir);

  // Grid bounding box of the sector: the apex and the two far corners.
  const float px[3] = {ox, ox + reach_m * cosf(dir - half), ox + reach_m * cosf(dir + half)};
  const float py[3] = {oy, oy + reach_m * sinf(dir - half), oy + reach_m * sinf(dir + half)};
  int gx0, gy0, gx1, gy1, gx, gy;
  world_to_grid(min(px[0], min(px[1], px[2])), min(py[0], min(py[1], py[2])), gx0, gy0);
  world_to_grid(max(px[0], max(px[1], px[2])), max(py[0], max(py[1], py[2])), gx1, gy1);
  gx0 = max(gx0, 0); gy0 = max(gy0, 0);
  gx1 = min(gx1, MAP_GRID_W - 1); gy1 = min(gy1, MAP_GRID_H - 1);

  for (gy = gy0; gy <= gy1; ++gy) {
    for (gx = gx0; gx <= gx1; ++gx) {
      float wx, wy;
      grid_to_world(gx, gy, wx, wy);
      const float along = (wx - ox) * cd + (wy - oy) * sd;
      if (along <= 0.0f || along >= reach_m) continue;
      const float across = fabsf(-(wx - ox) * sd + (wy - oy) * cd);
      if (across > along * spread) continue;
      if (use == CONE_MARK_SEEN) {
        exploration_mark_fine_seen(gx, gy);
        continue;
      }
      if (across < 0.5f * MAP_CELL_SIZE_M) continue;  // centre line: map_ray_trace
      map_observe_free(gx, gy, true);
    }
  }
}

bool tof_sensor_geometry(uint8_t index, float &fwd_m, float &left_m, float &facing_rad) {
  switch (index) {
    case TOF_LFT: fwd_m = TOP_LEFT_FWD_M;     left_m = TOP_LEFT_LEFT_M;     facing_rad = 0.0f;     return true;
    case TOF_LFB: fwd_m = BOTTOM_LEFT_FWD_M;  left_m = BOTTOM_LEFT_LEFT_M;  facing_rad = 0.0f;     return true;
    case TOF_RFT: fwd_m = TOP_RIGHT_FWD_M;    left_m = TOP_RIGHT_LEFT_M;    facing_rad = 0.0f;     return true;
    case TOF_RFB: fwd_m = BOTTOM_RIGHT_FWD_M; left_m = BOTTOM_RIGHT_LEFT_M; facing_rad = 0.0f;     return true;
    case TOF_LST: case TOF_LSB:
      fwd_m = SIDE_FWD_M; left_m = SIDE_LEFT_LEFT_M;  facing_rad = HALF_PI;  return true;
    case TOF_RST: case TOF_RSB:
      fwd_m = SIDE_FWD_M; left_m = SIDE_RIGHT_LEFT_M; facing_rad = -HALF_PI; return true;
  }
  return false;
}

Pose tof_pose_for_reading(const TofReading &reading, const Pose &current) {
  Pose p = current;
  pose_at(reading.received_ms - TOF_MEASUREMENT_LAG_MS, p);
  return p;
}

// While true the front pairs are ignored for mapping and sightings (see tof_set_front_blind).
static bool front_blind = false;

void tof_set_front_blind(bool blind) {
  front_blind = blind;
}

bool tof_front_clear_to(const TofReading ranges[TOF_TOTAL_COUNT], float distance_m, float margin_m) {
  // Both top sensors must be working; either one seeing something well short
  // of distance_m (measured from the robot centre) means the path is blocked.
  if (!tof_reading_live(ranges[TOF_LFT]) || !tof_reading_live(ranges[TOF_RFT])) return false;
  const float reach_m = distance_m - margin_m;
  if (tof_reading_usable(ranges[TOF_LFT]) &&
      TOP_LEFT_FWD_M + ranges[TOF_LFT].range_mm * MM_TO_M < reach_m) return false;
  if (tof_reading_usable(ranges[TOF_RFT]) &&
      TOP_RIGHT_FWD_M + ranges[TOF_RFT].range_mm * MM_TO_M < reach_m) return false;
  return true;
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

static bool consume_pair(const TofReading ranges[TOF_TOTAL_COUNT], uint8_t bottom, uint8_t top) {
  static uint32_t consumed[TOF_TOTAL_COUNT] = {};
  if (ranges[bottom].sequence == consumed[bottom] ||
      ranges[top].sequence == consumed[top]) return false;
  if (!tof_reading_live(ranges[bottom]) || !tof_reading_live(ranges[top])) return false;
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
                          const PairGeometry &g, const Pose &current, PairObservation &observation) {
  // Each beam is placed from where the robot was when that sensor measured.
  const Pose bottom_pose = tof_pose_for_reading(bottom, current);
  const Pose top_pose = tof_pose_for_reading(top, current);
  // Nothing within range counts as empty space out to the detection range:
  // no hit, and the beam's cells receive the usual free-space evidence.
  const float bottom_range = bottom.no_target ? g.max_range_m : bottom.range_mm * MM_TO_M;
  const float top_range = top.no_target ? g.max_range_m : top.range_mm * MM_TO_M;
  // Both endpoints measured along the beam from the same robot reference, not sensor faces.
  const float c = cosf(g.facing_rad), s = sinf(g.facing_rad);
  const float clearance = (g.top_fwd * c + g.top_left * s + top_range) -
                          (g.bottom_fwd * c + g.bottom_left * s + bottom_range);
  const bool lower_hit = bottom_range < g.max_range_m;
  const bool upper_hit = top_range < g.max_range_m;
  const bool collectable = lower_hit &&
      (clearance > WALL_TOLERANCE_M || (g.top_miss_is_clear && !upper_hit));
  // Cones first; the centre-line updates below then act on the results.
  cone_cells(bottom_pose, g.bottom_fwd, g.bottom_left, g.facing_rad, g.bottom_fov_rad,
             lower_hit ? bottom_range - CONE_HIT_MARGIN_M : g.max_range_m, CONE_FREE_EVIDENCE);
  cone_cells(top_pose, g.top_fwd, g.top_left, g.facing_rad, g.top_fov_rad,
             upper_hit ? top_range - CONE_HIT_MARGIN_M : g.max_range_m, CONE_FREE_EVIDENCE);
  // Explored: out to what the bottom beam reached (the hit cell included), no further.
  if (g.marks_seen) {
    cone_cells(bottom_pose, g.bottom_fwd, g.bottom_left, g.facing_rad, g.bottom_fov_rad,
               lower_hit ? bottom_range + 0.5f * MAP_CELL_SIZE_M : g.max_range_m, CONE_MARK_SEEN);
  }
  float ox, oy, bx, by, tx, ty;
  int bgx, bgy, tgx, tgy;
  project_point(bottom_pose, g.bottom_fwd, g.bottom_left, g.facing_rad, 0, ox, oy);
  project_point(bottom_pose, g.bottom_fwd, g.bottom_left, g.facing_rad, min(bottom_range, g.max_range_m), bx, by);
  map_ray_trace(ox, oy, bx, by, false);
  const bool bottom_inside = world_to_grid(bx, by, bgx, bgy);
  if (bottom_inside) {
    if (collectable) map_observe_weight(bgx, bgy);
    else if (lower_hit) map_reject_weight(bgx, bgy);
    else map_observe_free(bgx, bgy);
  }
  project_point(top_pose, g.top_fwd, g.top_left, g.facing_rad, 0, ox, oy);
  project_point(top_pose, g.top_fwd, g.top_left, g.facing_rad, min(top_range, g.max_range_m), tx, ty);
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
  for (auto &o : observations) o.usable = false;
  const bool right_ready = !front_blind && consume_pair(ranges, TOF_RFB, TOF_RFT);
  const bool left_ready = !front_blind && consume_pair(ranges, TOF_LFB, TOF_LFT);
  const bool side_right_ready = USE_SIDE_TOFS && consume_pair(ranges, TOF_RSB, TOF_RST);
  const bool side_left_ready = USE_SIDE_TOFS && consume_pair(ranges, TOF_LSB, TOF_LST);
  if (!pose_heading_valid() || !isfinite(angular_speed_rad_s()) ||
      fabsf(angular_speed_rad_s()) > TOF_MAX_TURN_RATE_RAD_S) return false;
  bool found = false;
  if (right_ready) found |= classify_pair(ranges[TOF_RFB], ranges[TOF_RFT], FRONT_RIGHT, pose, observations[0]);
  if (left_ready) found |= classify_pair(ranges[TOF_LFB], ranges[TOF_LFT], FRONT_LEFT, pose, observations[1]);
  // Side pairs only update the map. Sightings and target confirmation stay
  // front-only, since the robot turns to face a weight before confirming it.
  PairObservation side_observation;
  if (side_right_ready) classify_pair(ranges[TOF_RSB], ranges[TOF_RST], SIDE_RIGHT, pose, side_observation);
  if (side_left_ready) classify_pair(ranges[TOF_LSB], ranges[TOF_LST], SIDE_LEFT, pose, side_observation);
  return found;
}

TofNavState tof_nav_update(const TofReading ranges[TOF_TOTAL_COUNT], int &left_pct, int &right_pct) {
  const bool right_valid = tof_reading_live(ranges[TOF_RFT]);
  const bool left_valid = tof_reading_live(ranges[TOF_LFT]);
  left_pct = right_pct = 0;
  // Missing obstacle data (sensor failure) is explicitly unknown, not clear. Hold for now;
  // broader degraded-sensor driving behaviour is a separate task.
  if (!right_valid || !left_valid) return TOF_NAV_UNKNOWN;
  // "Nothing within range" is clear space, never an imminent obstacle.
  bool r = tof_reading_usable(ranges[TOF_RFT]) && ranges[TOF_RFT].range_mm < IMMINENT_MM;
  bool l = tof_reading_usable(ranges[TOF_LFT]) && ranges[TOF_LFT].range_mm < IMMINENT_MM;

  if (!r && !l) return TOF_NAV_CLEAR;

  bool turn_left;
  if (r && l) turn_left = ranges[TOF_RFT].range_mm <= ranges[TOF_LFT].range_mm;
  else        turn_left = r;

  if (turn_left) { left_pct = -AVOID_SPEED; right_pct =  AVOID_SPEED; }
  else           { left_pct =  AVOID_SPEED; right_pct = -AVOID_SPEED; }

  return TOF_NAV_AVOID;
}
