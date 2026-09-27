#include "state_machine.h"
#include "navigation.h"
#include "grid_map.h"
#include "tof_nav.h"
#include "weight_collection.h"
#include <Arduino.h>

// --- Tuning - all TODOs need checking/adjusting on real hardware ---
static const uint32_t WEIGHT_LOST_TIMEOUT_MS = 2500;   // no fresh sighting -> scan (only if map is empty)
static const int WEIGHTS_BEFORE_HOME = 3;

// APPROACH_VERIFY: rotate to face the target and confirm it before driving
static const float ALIGN_TOLERANCE_RAD = radians(5.0f);
static const int   VERIFY_TURN_PCT = 35;
static const uint32_t VERIFY_ALIGN_TIMEOUT_MS = 2000;  // safety - don't spin forever trying to align exactly

// SCANNING: stop-and-sample sweep. Coarse steps + short settle to keep this
// cheap given the 2-minute round - see conversation notes on TOF timing.
static const float    SCAN_STEP_DEG = 45.0f;
static const uint32_t SCAN_SETTLE_MS = 150;            // >= TOF timing budget (50ms) + margin
static const int      SCAN_TURN_PCT = 35;

// Lawnmower search pattern
static const int MAX_WAYPOINTS = 24;
static const float WAYPOINT_MARGIN_M = 0.2f;
static const float WAYPOINT_STRIP_SPACING_M = 0.4f;    // TODO: tune to your TOF/flow effective sensing width

// --- State ---
static StateMachine current_state = NAVIGATION;
static Target locked_target;
static uint32_t last_weight_seen_ms = 0;

static Target waypoints[MAX_WAYPOINTS];
static int waypoint_count = 0;
static int waypoint_index = 0;

static uint32_t verify_start_ms = 0;

static int scan_step = 0;
static int scan_step_count = 0;
static float scan_start_theta = 0.0f;
static bool scan_turning = true;
static uint32_t scan_settle_start_ms = 0;

static float wrap_angle(float angle_rad) {
  while (angle_rad > PI)  angle_rad -= 2.0f * PI;
  while (angle_rad < -PI) angle_rad += 2.0f * PI;
  return angle_rad;
}

// Overrides out_left_pct/out_right_pct if something is dangerously close,
// regardless of what navigation decided.
static void apply_reflex(const uint16_t ranges[TOF_TOTAL_COUNT], int &left_pct, int &right_pct) {
  int rl, rr;
  if (tof_nav_update(ranges, rl, rr) == TOF_NAV_AVOID) {
    left_pct = rl;
    right_pct = rr;
  }
}

// Simple back-and-forth sweep across the arena, used as the default search
// pattern instead of a single fixed corner target.
static void build_waypoints() {
  waypoint_count = 0;
  float x_min = WAYPOINT_MARGIN_M;
  float x_max = MAP_GRID_SIZE_X_M - WAYPOINT_MARGIN_M;
  bool going_right = true;

  for (float y = WAYPOINT_MARGIN_M;
       y < MAP_GRID_SIZE_Y_M - WAYPOINT_MARGIN_M && waypoint_count < MAX_WAYPOINTS;
       y += WAYPOINT_STRIP_SPACING_M) {
    waypoints[waypoint_count].x = going_right ? x_max : x_min;
    waypoints[waypoint_count].y = y;
    waypoint_count++;
    going_right = !going_right;
  }

  if (waypoint_count == 0) {
    // Arena smaller than the margins allow for - fall back to one corner.
    waypoints[0] = {MAP_GRID_SIZE_X_M - WAYPOINT_MARGIN_M, MAP_GRID_SIZE_Y_M - WAYPOINT_MARGIN_M};
    waypoint_count = 1;
  }
}

void state_machine_init() {
  build_waypoints();
  waypoint_index = 0;
  current_state = NAVIGATION;
  last_weight_seen_ms = millis();
}

StateMachine state_machine_current_state() {
  return current_state;
}

static void run_navigation(const Pose &pose, const uint16_t ranges[TOF_TOTAL_COUNT],
                            int &left_pct, int &right_pct) {
  Target found;
  if (find_nearest_weight(pose, found)) {
    locked_target = found;
    verify_start_ms = millis();
    current_state = APPROACH_VERIFY;
    left_pct = right_pct = 0;
    return;
  }

  if (millis() - last_weight_seen_ms >= WEIGHT_LOST_TIMEOUT_MS) {
    scan_step = 0;
    scan_step_count = (int)(360.0f / SCAN_STEP_DEG);
    scan_start_theta = pose.theta;
    scan_turning = true;
    current_state = SCANNING;
    left_pct = right_pct = 0;
    return;
  }

  if (has_arrived(pose, waypoints[waypoint_index])) {
    waypoint_index = (waypoint_index + 1) % waypoint_count;
  }
  navigate_to_target(pose, waypoints[waypoint_index], left_pct, right_pct);
  apply_reflex(ranges, left_pct, right_pct);
}

static void run_approach_verify(const Pose &pose, bool weight_seen_this_frame,
                                 int &left_pct, int &right_pct) {
  float err = heading_error_to(pose, locked_target);
  bool timed_out = millis() - verify_start_ms >= VERIFY_ALIGN_TIMEOUT_MS;

  if (fabsf(err) > ALIGN_TOLERANCE_RAD && !timed_out) {
    int turn = err > 0 ? VERIFY_TURN_PCT : -VERIFY_TURN_PCT;
    left_pct = -turn;
    right_pct = turn;
    return;
  }

  // Aligned (or gave up trying to align exactly) - trust this frame's
  // classification to confirm the weight is actually still there.
  left_pct = right_pct = 0;
  if (weight_seen_this_frame) {
    current_state = APPROACH_WEIGHT;
    return;
  }

  // Stale - someone (probably the other robot) got there first, or it was
  // a false positive. Clear the cell and try the next-nearest, if any.
  int gx, gy;
  if (world_to_grid(locked_target.x, locked_target.y, gx, gy)) {
    map_set_cell(gx, gy, MAP_CELL_FREE);
  }

  Target next;
  if (find_nearest_weight(pose, next)) {
    locked_target = next;
    verify_start_ms = millis();
    // stay in APPROACH_VERIFY, will align to the new target next loop
  } else {
    current_state = NAVIGATION;
  }
}

static void run_approach_weight(const Pose &pose, const uint16_t ranges[TOF_TOTAL_COUNT],
                                 int &left_pct, int &right_pct) {
  // Fallback distance check - the catchment sensors (induction/IR proximity,
  // handled entirely outside this state machine via weight_collection) are
  // the primary way a pickup actually happens. This just stops us treating
  // "still approaching" as the state forever if the sensors miss it.
  if (has_arrived(pose, locked_target, WEIGHT_ARRIVAL_RADIUS_M)) {
    current_state = NAVIGATION;
    left_pct = right_pct = 0;
    return;
  }
  navigate_to_target(pose, locked_target, left_pct, right_pct);
  apply_reflex(ranges, left_pct, right_pct);
}

static void run_scanning(const Pose &pose, int &left_pct, int &right_pct) {
  if (scan_turning) {
    float target_theta = wrap_angle(scan_start_theta + radians(SCAN_STEP_DEG) * scan_step);
    float err = wrap_angle(target_theta - pose.theta);

    if (fabsf(err) <= ALIGN_TOLERANCE_RAD) {
      scan_turning = false;
      scan_settle_start_ms = millis();
      left_pct = right_pct = 0;
    } else {
      int turn = err > 0 ? SCAN_TURN_PCT : -SCAN_TURN_PCT;
      left_pct = -turn;
      right_pct = turn;
    }
    return;
  }

  left_pct = right_pct = 0;   // hold still while the TOFs settle and get read

  if (millis() - scan_settle_start_ms < SCAN_SETTLE_MS) return;

  scan_step++;
  if (scan_step >= scan_step_count) {
    // Full sweep done.
    Target found;
    if (find_nearest_weight(pose, found)) {
      locked_target = found;
      verify_start_ms = millis();
      current_state = APPROACH_VERIFY;
    } else {
      last_weight_seen_ms = millis();   // don't immediately re-trigger another scan
      current_state = NAVIGATION;
    }
  } else {
    scan_turning = true;
  }
}

static void run_return_home(const Pose &pose, const uint16_t ranges[TOF_TOTAL_COUNT],
                             int &left_pct, int &right_pct) {
  Target home = {0.0f, 0.0f};

  if (has_arrived(pose, home)) {
    pose_set_position(0.0f, 0.0f);   // correct drift now we know exactly where we are
    weight_collection_request_release();
    current_state = DROP_OFF;
    left_pct = right_pct = 0;
    return;
  }

  navigate_to_target(pose, home, left_pct, right_pct);
  apply_reflex(ranges, left_pct, right_pct);
}

void state_machine_update(const Pose &pose, const uint16_t ranges[TOF_TOTAL_COUNT],
                           bool weight_seen_this_frame, int &out_left_pct, int &out_right_pct) {
  if (weight_seen_this_frame) {
    last_weight_seen_ms = millis();
  }

  // 3 collected -> head home, overriding whatever else was happening.
  if (current_state != RETURN_HOME && current_state != DROP_OFF &&
      weight_collection_count() >= WEIGHTS_BEFORE_HOME) {
    current_state = RETURN_HOME;
  }

  out_left_pct = 0;
  out_right_pct = 0;

  switch (current_state) {
    case NAVIGATION:
      run_navigation(pose, ranges, out_left_pct, out_right_pct);
      break;
    case APPROACH_VERIFY:
      run_approach_verify(pose, weight_seen_this_frame, out_left_pct, out_right_pct);
      break;
    case APPROACH_WEIGHT:
      run_approach_weight(pose, ranges, out_left_pct, out_right_pct);
      break;
    case SCANNING:
      run_scanning(pose, out_left_pct, out_right_pct);
      break;
    case RETURN_HOME:
      run_return_home(pose, ranges, out_left_pct, out_right_pct);
      break;
    case DROP_OFF:
      // Only reached again once weight_collection's release swing has
      // actually completed (see main.cpp's busy-gating) - colour sensing /
      // real deposit logic goes here later. For now, just resume searching.
      current_state = NAVIGATION;
      last_weight_seen_ms = millis();
      break;
  }
}