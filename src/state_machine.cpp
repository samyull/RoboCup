#include "state_machine.h"
#include "navigation.h"
#include "grid_map.h"
#include "tof_nav.h"
#include "weight_collection.h"
#include <Arduino.h>

// --- Tuning - all TODOs need checking/adjusting on real hardware ---
static const uint32_t WEIGHT_LOST_TIMEOUT_MS = 2500;   // no fresh sighting -> scan (only if map is empty)
static const int WEIGHTS_BEFORE_HOME = 3;

static const int   APPROACH_SPEED_PCT = 75;   // TODO: find the sweet spot on the bench
static const float APPROACH_TURN_KP   = 40.0f; // gentler than general navigation - funnel forgives error
static const int   MAX_APPROACH_TURN_PCT = 30;

// APPROACH_VERIFY: rotate to face the target and confirm it before driving
static const float ALIGN_TOLERANCE_RAD = radians(5.0f);
static const int   VERIFY_TURN_PCT = 35;
static const uint32_t VERIFY_ALIGN_TIMEOUT_MS = 2000;  // safety - don't spin forever trying to align exactly

// SCANNING: stop-and-sample sweep. Coarse steps + short settle to keep this
// cheap given the 2-minute round - see conversation notes on TOF timing.
static const float    SCAN_STEP_DEG = 15.0f;
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
enum VerifyPhase { VERIFY_ALIGN, VERIFY_SETTLE, VERIFY_OBSERVE };
static VerifyPhase verify_phase = VERIFY_ALIGN;
static uint32_t verify_phase_ms = 0;
static uint32_t verify_observe_after_ms = 0;
static const uint32_t VERIFY_SETTLE_MS = 150;
static const uint32_t VERIFY_OBSERVE_TIMEOUT_MS = 1500;
static const float VERIFY_SETTLE_RATE_RAD_S = 0.10f;

static void begin_verification(const Target &target) {
  locked_target = target;
  verify_start_ms = millis();
  verify_phase = VERIFY_ALIGN;
  current_state = APPROACH_VERIFY;
}


static int scan_step = 0;
static int scan_step_count = 0;
static float scan_start_theta = 0.0f;
static bool scan_turning = true;
static uint32_t scan_settle_start_ms = 0;

static void abandon_verification(const Pose &pose) {
  // No reliable confirmation: search again without deleting map evidence.
  scan_step = 0;
  scan_step_count = (int)(360.0f / SCAN_STEP_DEG);
  scan_start_theta = pose.theta;
  scan_turning = true;
  current_state = SCANNING;
}

static float wrap_angle(float angle_rad) {
  while (angle_rad > PI)  angle_rad -= 2.0f * PI;
  while (angle_rad < -PI) angle_rad += 2.0f * PI;
  return angle_rad;
}

// Overrides out_left_pct/out_right_pct if something is dangerously close,
// regardless of what navigation decided.
static void apply_reflex(const TofReading ranges[TOF_TOTAL_COUNT], int &left_pct, int &right_pct) {
  int rl, rr;
  if (tof_nav_update(ranges, rl, rr) != TOF_NAV_CLEAR) {
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

static void run_navigation(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT],
                            int &left_pct, int &right_pct) {
  Target found;
  if (find_nearest_weight(pose, found)) {
    begin_verification(found);
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

static void run_approach_verify(const Pose &pose, int &left_pct, int &right_pct) {
  left_pct = right_pct = 0;
  const uint32_t now = millis();
  const float err = heading_error_to(pose, locked_target);
  if (verify_phase == VERIFY_ALIGN) {
    if (fabsf(err) > ALIGN_TOLERANCE_RAD) {
      if (now - verify_start_ms >= VERIFY_ALIGN_TIMEOUT_MS) {
        abandon_verification(pose);
        return;
      }
      const int turn = err > 0 ? VERIFY_TURN_PCT : -VERIFY_TURN_PCT;
      left_pct = -turn; right_pct = turn;
      return;
    }
    verify_phase = VERIFY_SETTLE;
    verify_phase_ms = now;
    return; // stop before beginning any verification
  }
  if (fabsf(err) > ALIGN_TOLERANCE_RAD) {
    // Overshoot requires realignment, never acceptance of an unrelated sighting.
    verify_phase = VERIFY_ALIGN;
    return;
  }
  const float rate = angular_speed_rad_s();
  if (!pose_heading_valid() || !isfinite(rate) || fabsf(rate) > VERIFY_SETTLE_RATE_RAD_S) {
    if (now - verify_start_ms >= VERIFY_ALIGN_TIMEOUT_MS + VERIFY_OBSERVE_TIMEOUT_MS) {
      abandon_verification(pose);
      return;
    }
    verify_phase = VERIFY_SETTLE;
    verify_phase_ms = now;
    return;
  }
  if (verify_phase == VERIFY_SETTLE) {
    if (now - verify_phase_ms < VERIFY_SETTLE_MS) return;
    verify_phase = VERIFY_OBSERVE;
    verify_phase_ms = now;
    verify_observe_after_ms = now;
    return; // require both pair readings to arrive AFTER settling finished
  }
  float matched_x, matched_y;
  if (tof_confirm_target(locked_target.x, locked_target.y, verify_observe_after_ms,
                         matched_x, matched_y)) {
    locked_target = {matched_x, matched_y};
    current_state = APPROACH_WEIGHT;
    return;
  }
  int gx, gy;
  if (tof_observation_after(verify_observe_after_ms) &&
      (!world_to_grid(locked_target.x, locked_target.y, gx, gy) ||
       map_get_cell(gx, gy) != MAP_CELL_WEIGHT)) {
    // Normal confidence updates already retired the target; don't clear it twice.
    abandon_verification(pose);
    return;
  }
  if (now - verify_phase_ms >= VERIFY_OBSERVE_TIMEOUT_MS) abandon_verification(pose);
}

static void run_approach_weight(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT],
                                 int &left_pct, int &right_pct) {
  int target_gx, target_gy;
  if (!world_to_grid(locked_target.x, locked_target.y, target_gx, target_gy) ||
      map_get_cell(target_gx, target_gy) != MAP_CELL_WEIGHT) {
    // Confidence decayed or new observations made this target uncollectable.
    current_state = NAVIGATION;
    left_pct = right_pct = 0;
    return;
  }
  // Fallback only - real "arrival" is the catchment sensors firing, handled
  // entirely outside this state machine via weight_collection_busy().
  if (has_arrived(pose, locked_target, WEIGHT_ARRIVAL_RADIUS_M)) {
    current_state = NAVIGATION;
    left_pct = right_pct = 0;
    return;
  }

  float err = heading_error_to(pose, locked_target);
  int turn = (int)(APPROACH_TURN_KP * err);
  if (turn > MAX_APPROACH_TURN_PCT) turn = MAX_APPROACH_TURN_PCT;
  if (turn < -MAX_APPROACH_TURN_PCT) turn = -MAX_APPROACH_TURN_PCT;

  left_pct = APPROACH_SPEED_PCT - turn;
  right_pct = APPROACH_SPEED_PCT + turn;

  apply_reflex(ranges, left_pct, right_pct);
}

void run_scanning(const Pose &pose, int &left_pct, int &right_pct) {
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
      begin_verification(found);
    } else {
      last_weight_seen_ms = millis();   // don't immediately re-trigger another scan
      current_state = NAVIGATION;
    }
  } else {
    scan_turning = true;
  }
}

static void run_return_home(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT],
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

void state_machine_update(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT], bool weight_seen_this_frame, int &out_left_pct, int &out_right_pct) {

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
      run_approach_verify(pose, out_left_pct, out_right_pct);
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
