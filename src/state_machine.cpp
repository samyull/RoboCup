#include "state_machine.h"
#include "navigation.h"
#include "grid_map.h"
#include "tof_nav.h"
#include "weight_collection.h"
#include "path_planner.h"
#include "exploration.h"
#include "return_to_base.h"
#include <Arduino.h>

// Frontier exploration + path planning. false = the old waypoint sweep.
static const bool USE_FRONTIER_NAV = true;

// --- Tuning - all TODOs need checking/adjusting on real hardware ---
static const uint32_t WEIGHT_LOST_TIMEOUT_MS = 2500;   // legacy: no fresh sighting -> scan
static const int WEIGHTS_BEFORE_HOME = 3;
static const uint32_t EARLY_HOME_MS = 90000;           // from 90 s, any weight on board -> home
// After the home colour wasn't found with a part load, keep exploring at least
// this long before an early/partial return home is allowed again.
static const uint32_t HOME_RETRY_COOLDOWN_MS = 15000;

static const int   APPROACH_SPEED_PCT = 75;   // TODO: find the sweet spot on the bench
static const float APPROACH_TURN_KP   = 40.0f; // gentler than general navigation - funnel forgives error
static const int   MAX_APPROACH_TURN_PCT = 30;

// APPROACH_VERIFY: rotate to face the target and confirm it before driving
static const float ALIGN_TOLERANCE_RAD = radians(5.0f);
static const int   VERIFY_TURN_PCT = 35;
static const uint32_t VERIFY_ALIGN_TIMEOUT_MS = 2000;  // safety - don't spin forever trying to align exactly

// APPROACH_BLIND: final approach once the target is confirmed and the path is clear.
static const float    BLIND_START_DIST_M      = 1.0f;   // start within this of the target
static const float    BLIND_CORRIDOR_HALF_W_M = 0.155f; // robot half-width incl. safety margin (310 mm corridor)
static const float    BLIND_TOF_MARGIN_M      = 0.15f;  // top ToF reading this far short of target = blocked
static const uint32_t BLIND_TIMEOUT_MS        = 5000;
static const float    BLIND_CLOSE_DIST_M      = 0.20f;  // this close at the timeout: keep creeping in
static const uint32_t BLIND_CLOSE_EXTRA_MS    = 3000;   // hard cap on that extra creep

// Frontier exploration / path following
static const uint32_t REPLAN_MS            = 500;
static const float    FRONTIER_ARRIVE_M    = 0.15f;
static const float    WEIGHT_HANDOVER_M    = 1.0f;    // GO_TO_WEIGHT -> APPROACH_VERIFY when this close + line of sight
static const float    WEIGHT_MIN_PATH_M    = 0.05f;   // floor for confidence / path-length scoring
static const uint32_t WEIGHT_SKIP_MS       = 10000;   // failed/unreachable weight cooldown
static const float    WEIGHT_SKIP_RADIUS_M = 0.15f;
static const uint32_t DROP_OFF_TIMEOUT_MS  = 3000;    // release should start well within this

// Stuck: driving forward but not getting anywhere.
static const uint32_t STUCK_WINDOW_MS  = 3000;
static const float    STUCK_MIN_MOVE_M = 0.05f;
static const uint32_t STUCK_REVERSE_MS = 500;
static const int      STUCK_REVERSE_PCT = 60;

// SCANNING. Frontier mode: one continuous turn (the ToF pose history keeps
// mapping valid while turning). Legacy: stop-and-sample steps.
static const float    SCAN_STEP_DEG = 15.0f;
static const uint32_t SCAN_SETTLE_MS = 150;            // >= TOF timing budget (50ms) + margin
static const int      SCAN_TURN_PCT = 35;

// Legacy lawnmower search pattern
static const int MAX_WAYPOINTS = 24;
static const float WAYPOINT_MARGIN_M = 0.2f;
static const float WAYPOINT_STRIP_SPACING_M = 0.4f;    // TODO: tune to your TOF/flow effective sensing width

// --- State ---
static StateMachine current_state = NAVIGATION;
static Target locked_target;
static uint32_t last_weight_seen_ms = 0;
static uint32_t round_start_ms = 0;

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

static uint32_t blind_start_ms = 0;
static uint32_t blind_start_pickups = 0;
static uint32_t blind_start_clears = 0;
static bool blind_holding_heading = false;  // within BLIND_CLOSE_DIST_M: drive straight
static float blind_hold_theta = 0.0f;

static uint32_t last_plan_ms = 0;
static bool have_path = false;
static uint32_t drop_off_start_ms = 0;
static uint32_t home_retry_after_ms = 0;  // partial-load returns home are held off until this

static bool stuck_tracking = false;
static float stuck_x = 0.0f, stuck_y = 0.0f;
static uint32_t stuck_since_ms = 0;
static uint32_t reverse_until_ms = 0;

struct WeightSkip { float x, y; uint32_t until_ms; };
static WeightSkip weight_skips[4];

static int scan_step = 0;
static int scan_step_count = 0;
static float scan_start_theta = 0.0f;
static bool scan_turning = true;
static uint32_t scan_settle_start_ms = 0;
static float scan_turned_rad = 0.0f;
static float scan_prev_theta = 0.0f;

static float wrap_angle(float angle_rad) {
  while (angle_rad > PI)  angle_rad -= 2.0f * PI;
  while (angle_rad < -PI) angle_rad += 2.0f * PI;
  return angle_rad;
}

static void force_replan() {
  last_plan_ms = millis() - REPLAN_MS;
}

// Back to searching: frontier exploration, or the legacy sweep.
static void resume_search() {
  current_state = USE_FRONTIER_NAV ? EXPLORE : NAVIGATION;
  last_weight_seen_ms = millis();
  have_path = false;
  force_replan();
}

static void begin_verification(const Target &target) {
  locked_target = target;
  verify_start_ms = millis();
  verify_phase = VERIFY_ALIGN;
  current_state = APPROACH_VERIFY;
}

static void skip_weight(const Target &t) {
  WeightSkip *slot = &weight_skips[0];
  for (auto &s : weight_skips)
    if ((int32_t)(s.until_ms - slot->until_ms) < 0) slot = &s;  // oldest / expired
  *slot = {t.x, t.y, millis() + WEIGHT_SKIP_MS};
}

static bool weight_skipped(float x, float y) {
  const uint32_t now = millis();
  for (const auto &s : weight_skips)
    if ((int32_t)(s.until_ms - now) > 0 && hypotf(x - s.x, y - s.y) <= WEIGHT_SKIP_RADIUS_M) return true;
  return false;
}

static void begin_legacy_scan(const Pose &pose) {
  scan_step = 0;
  scan_step_count = (int)(360.0f / SCAN_STEP_DEG);
  scan_start_theta = pose.theta;
  scan_turning = true;
  current_state = SCANNING;
}

static void begin_continuous_scan(const Pose &pose) {
  scan_turned_rad = 0.0f;
  scan_prev_theta = pose.theta;
  current_state = SCANNING;
  Serial.println("No reachable frontier - scanning");
}

static void abandon_verification(const Pose &pose) {
  // No reliable confirmation: search again without deleting map evidence.
  if (USE_FRONTIER_NAV) {
    skip_weight(locked_target);  // don't head straight back to the same spot
    resume_search();
  } else {
    begin_legacy_scan(pose);
  }
}

static void begin_return_home(const char *reason) {
  Serial.print("Heading home: "); Serial.println(reason);
  return_home_begin();
  stuck_tracking = false;
  current_state = RETURN_HOME;
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

// True (and starts a short reverse) when driving forward has not moved the
// robot STUCK_MIN_MOVE_M in STUCK_WINDOW_MS, or path following has been held
// still by a blocked line for STUCK_WINDOW_MS.
static bool check_stuck(const Pose &pose, int left_pct, int right_pct) {
  const uint32_t now = millis();
  if (planner_blocked_ms() >= STUCK_WINDOW_MS) {
    planner_clear_path();  // also resets the blocked timer
    stuck_tracking = false;
    reverse_until_ms = now + STUCK_REVERSE_MS;
    Serial.println("Path blocked - reversing and replanning");
    return true;
  }
  if (left_pct <= 0 || right_pct <= 0) {
    stuck_tracking = false;
    return false;
  }
  if (!stuck_tracking || hypotf(pose.x - stuck_x, pose.y - stuck_y) > STUCK_MIN_MOVE_M) {
    stuck_tracking = true;
    stuck_x = pose.x;
    stuck_y = pose.y;
    stuck_since_ms = now;
    return false;
  }
  if (now - stuck_since_ms < STUCK_WINDOW_MS) return false;
  stuck_tracking = false;
  reverse_until_ms = now + STUCK_REVERSE_MS;
  Serial.println("Stuck - reversing and replanning");
  return true;
}

// Best reachable mapped weight by confidence / path length (call planner_plan first).
static bool choose_weight(Target &out) {
  float best_score = -1.0f;
  for (int gy = 0; gy < MAP_GRID_H; ++gy)
    for (int gx = 0; gx < MAP_GRID_W; ++gx) {
      if (map_get_cell(gx, gy) != MAP_CELL_WEIGHT) continue;
      // Too close to a wall or the border to collect: never a target.
      if (!planner_cell_open(gx, gy)) continue;
      if (planner_cost(gx, gy) == PLAN_UNREACHABLE) continue;
      float x, y;
      grid_to_world(gx, gy, x, y);
      if (weight_skipped(x, y)) continue;
      const float path_m = max(planner_distance_m(gx, gy), WEIGHT_MIN_PATH_M);
      const float score = map_get_confidence(gx, gy) / path_m;
      if (score > best_score) {
        best_score = score;
        out = {x, y};
      }
    }
  return best_score >= 0.0f;
}

static void go_to_weight(const Target &weight) {
  locked_target = weight;
  current_state = GO_TO_WEIGHT;
  stuck_tracking = false;
  have_path = false;
  force_replan();
  Serial.print("Going to weight at "); Serial.print(weight.x, 2);
  Serial.print(","); Serial.println(weight.y, 2);
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
  round_start_ms = millis();
  for (auto &s : weight_skips) s = {0.0f, 0.0f, round_start_ms};
  home_retry_after_ms = round_start_ms;
  stuck_tracking = false;
  reverse_until_ms = round_start_ms;
  planner_clear_path();
  resume_search();
}

StateMachine state_machine_current_state() {
  return current_state;
}

const char *state_machine_state_name() {
  switch (current_state) {
    case NAVIGATION:      return "NAVIGATION";
    case APPROACH_VERIFY: return "APPROACH_VERIFY";
    case APPROACH_WEIGHT: return "APPROACH_WEIGHT";
    case SCANNING:        return "SCANNING";
    case RETURN_HOME:     return "RETURN_HOME";
    case DROP_OFF:        return "DROP_OFF";
    case APPROACH_BLIND:  return "APPROACH_BLIND";
    case EXPLORE:         return "EXPLORE";
    case GO_TO_WEIGHT:    return "GO_TO_WEIGHT";
  }
  return "?";
}

bool state_machine_front_blind() {
  return current_state == APPROACH_BLIND;
}

// --- Frontier exploration ---

static void run_explore(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT],
                        int &left_pct, int &right_pct) {
  left_pct = right_pct = 0;
  const uint32_t now = millis();
  if (now - last_plan_ms >= REPLAN_MS) {
    last_plan_ms = now;
    planner_plan(pose, PLAN_ALLOW_UNSEEN);
    Target weight;
    if (choose_weight(weight)) {
      go_to_weight(weight);
      return;
    }
    FrontierGoal goal;
    if (!exploration_choose_frontier(pose, goal)) {
      begin_continuous_scan(pose);
      return;
    }
    have_path = planner_set_goal(goal.gx, goal.gy);
  }
  if (!have_path) return;
  if (planner_follow(pose, FRONTIER_ARRIVE_M, left_pct, right_pct)) force_replan();  // reached: next goal
  apply_reflex(ranges, left_pct, right_pct);
  if (check_stuck(pose, left_pct, right_pct)) {
    exploration_skip_current_goal();
    force_replan();
  }
}

static void run_go_to_weight(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT],
                             int &left_pct, int &right_pct) {
  left_pct = right_pct = 0;
  int gx, gy;
  if (!world_to_grid(locked_target.x, locked_target.y, gx, gy) || map_get_cell(gx, gy) != MAP_CELL_WEIGHT) {
    resume_search();  // decayed, disproved, or collected by someone else
    return;
  }
  const uint32_t now = millis();
  if (now - last_plan_ms >= REPLAN_MS) {
    last_plan_ms = now;
    planner_plan(pose, PLAN_ALLOW_UNSEEN);
    // A wall mapped since choosing it can put the weight inside the wall margin.
    have_path = planner_cell_open(gx, gy) && planner_set_goal(gx, gy);
    if (!have_path) {
      skip_weight(locked_target);
      resume_search();
      return;
    }
  }
  // Close with a clear straight line: turn to face it and confirm before approaching.
  if (distance_to(pose, locked_target) <= WEIGHT_HANDOVER_M &&
      planner_straight_clear(pose.x, pose.y, locked_target.x, locked_target.y)) {
    planner_clear_path();
    begin_verification(locked_target);
    return;
  }
  if (!have_path) return;
  planner_follow(pose, 0.0f, left_pct, right_pct);
  apply_reflex(ranges, left_pct, right_pct);
  if (check_stuck(pose, left_pct, right_pct)) {
    skip_weight(locked_target);
    resume_search();
  }
}

static void run_scan_continuous(const Pose &pose, int &left_pct, int &right_pct) {
  left_pct = right_pct = 0;
  scan_turned_rad += fabsf(wrap_angle(pose.theta - scan_prev_theta));
  scan_prev_theta = pose.theta;
  const uint32_t now = millis();

  if (now - last_plan_ms >= REPLAN_MS) {
    // A weight spotted mid-scan takes priority.
    last_plan_ms = now;
    planner_plan(pose, PLAN_ALLOW_UNSEEN);
    Target weight;
    if (choose_weight(weight)) {
      go_to_weight(weight);
      return;
    }
  }
  if (scan_turned_rad >= 2.0f * PI) {
    planner_plan(pose, PLAN_ALLOW_UNSEEN);
    FrontierGoal goal;
    if (exploration_choose_frontier(pose, goal)) {
      resume_search();
    } else if (weight_collection_count() > 0 && (int32_t)(now - home_retry_after_ms) >= 0) {
      begin_return_home("nothing left to explore");
    } else {
      exploration_reset();  // everything seen and nothing found: go round again
      resume_search();
    }
    return;
  }
  left_pct = -SCAN_TURN_PCT;
  right_pct = SCAN_TURN_PCT;
}

// --- Legacy waypoint sweep ---

static void run_navigation(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT],
                            int &left_pct, int &right_pct) {
  Target found;
  if (find_nearest_weight(pose, found)) {
    begin_verification(found);
    left_pct = right_pct = 0;
    return;
  }

  if (millis() - last_weight_seen_ms >= WEIGHT_LOST_TIMEOUT_MS) {
    begin_legacy_scan(pose);
    left_pct = right_pct = 0;
    return;
  }

  if (has_arrived(pose, waypoints[waypoint_index])) {
    waypoint_index = (waypoint_index + 1) % waypoint_count;
  }
  navigate_to_target(pose, waypoints[waypoint_index], left_pct, right_pct);
  apply_reflex(ranges, left_pct, right_pct);
}

static void run_legacy_scan(const Pose &pose, int &left_pct, int &right_pct) {
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
      resume_search();   // also stops an immediate re-trigger of another scan
    }
  } else {
    scan_turning = true;
  }
}

// --- Approach ---

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

// Drive forward at approach speed, steering to cancel heading_err.
static void approach_drive(float heading_err, int &left_pct, int &right_pct) {
  int turn = (int)(APPROACH_TURN_KP * heading_err);
  if (turn > MAX_APPROACH_TURN_PCT) turn = MAX_APPROACH_TURN_PCT;
  if (turn < -MAX_APPROACH_TURN_PCT) turn = -MAX_APPROACH_TURN_PCT;

  left_pct = APPROACH_SPEED_PCT - turn;
  right_pct = APPROACH_SPEED_PCT + turn;
}

// Map cells a blind approach must not drive through.
static bool cell_blocks_path(int gx, int gy) {
  const int8_t cell = map_get_cell(gx, gy);
  return cell == MAP_CELL_OBSTACLE || cell == MAP_CELL_BORDER;
}

// Target within BLIND_START_DIST_M, no mapped walls in a robot-width corridor
// up to it, and neither front top ToF seeing anything well short of it.
static bool blind_path_clear(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT]) {
  const float dist = distance_to(pose, locked_target);
  if (dist > BLIND_START_DIST_M) return false;
  if (!tof_front_clear_to(ranges, dist, BLIND_TOF_MARGIN_M)) return false;
  if (dist < 1e-3f) return true;

  const float ux = (locked_target.x - pose.x) / dist;
  const float uy = (locked_target.y - pose.y) / dist;
  const float step = 0.5f * MAP_CELL_SIZE_M;
  // Stop a cell short of the target: the weight itself may sit against a wall.
  for (float d = 0.0f; d < dist - MAP_CELL_SIZE_M; d += step) {
    for (float w = -BLIND_CORRIDOR_HALF_W_M; w <= BLIND_CORRIDOR_HALF_W_M + 1e-3f; w += step) {
      int gx, gy;
      if (world_to_grid(pose.x + ux * d - uy * w, pose.y + uy * d + ux * w, gx, gy) &&
          cell_blocks_path(gx, gy)) return false;
    }
  }
  return true;
}

static void begin_blind(const Pose &pose) {
  blind_start_ms = millis();
  blind_start_pickups = weight_collection_total_pickups();
  blind_start_clears = weight_collection_total_clears();
  blind_holding_heading = false;
  current_state = APPROACH_BLIND;
  Serial.print("Blind approach: target "); Serial.print(distance_to(pose, locked_target), 2);
  Serial.println(" m ahead, front ToFs ignored");
}

static void end_blind_and_reverify(const char *reason) {
  Serial.print("Blind approach ended ("); Serial.print(reason);
  Serial.println(") - looking at the target again");
  begin_verification(locked_target);
}

static void run_approach_weight(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT],
                                 int &left_pct, int &right_pct) {
  int target_gx, target_gy;
  if (!world_to_grid(locked_target.x, locked_target.y, target_gx, target_gy) ||
      map_get_cell(target_gx, target_gy) != MAP_CELL_WEIGHT) {
    // Confidence decayed or new observations made this target uncollectable.
    resume_search();
    left_pct = right_pct = 0;
    return;
  }
  if (blind_path_clear(pose, ranges)) {
    begin_blind(pose);
    approach_drive(heading_error_to(pose, locked_target), left_pct, right_pct);
    return;
  }
  // Fallback only - real "arrival" is the catchment sensors firing, handled
  // entirely outside this state machine via weight_collection_busy().
  if (has_arrived(pose, locked_target, WEIGHT_ARRIVAL_RADIUS_M)) {
    resume_search();
    left_pct = right_pct = 0;
    return;
  }

  approach_drive(heading_error_to(pose, locked_target), left_pct, right_pct);
  apply_reflex(ranges, left_pct, right_pct);
}

// Steers on pose alone: no map check on the target, no reflex, no front ToFs.
// Feeding, pickup and the clearing arm run outside the state machine (it is
// not called while collection is busy); their outcome is checked on return.
static void run_approach_blind(const Pose &pose, int &left_pct, int &right_pct) {
  left_pct = right_pct = 0;

  if (weight_collection_total_pickups() != blind_start_pickups) {
    // Collected: the target cell is now empty, so retire it from the map.
    int gx, gy;
    if (world_to_grid(locked_target.x, locked_target.y, gx, gy)) map_reject_weight(gx, gy);
    Serial.println("Blind approach ended (weight collected)");
    resume_search();
    return;
  }
  if (weight_collection_total_clears() != blind_start_clears) {
    // Fed for the full timeout without induction and swept it away: check for a wall.
    end_blind_and_reverify("clearing arm swept the funnel");
    return;
  }

  const uint32_t elapsed = millis() - blind_start_ms;
  const float dist = distance_to(pose, locked_target);
  if (elapsed >= BLIND_TIMEOUT_MS &&
      (dist > BLIND_CLOSE_DIST_M || elapsed >= BLIND_TIMEOUT_MS + BLIND_CLOSE_EXTRA_MS)) {
    end_blind_and_reverify("timeout");
    return;
  }

  // Close in, steering at the target point gets unstable (and flips once
  // passed), so hold the heading we had on reaching the close zone.
  if (dist <= BLIND_CLOSE_DIST_M && !blind_holding_heading) {
    blind_holding_heading = true;
    blind_hold_theta = pose.theta;
  }
  const float err = blind_holding_heading ? wrap_angle(blind_hold_theta - pose.theta)
                                          : heading_error_to(pose, locked_target);
  approach_drive(err, left_pct, right_pct);
}

// --- Home ---

static void run_return_home(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT],
                             int &left_pct, int &right_pct) {
  const HomeStatus status = return_home_update(pose, left_pct, right_pct);
  if (status == HOME_ARRIVED) {
    weight_collection_request_release();
    drop_off_start_ms = millis();
    current_state = DROP_OFF;
    left_pct = right_pct = 0;
    return;
  }
  if (status == HOME_COLOR_NOT_FOUND) {
    left_pct = right_pct = 0;
    if (weight_collection_count() >= WEIGHTS_BEFORE_HOME) {
      // Full: nothing else to do, so drive round home until the base is found.
      return_home_search();
    } else {
      // Part load: more to gain from collecting than from hunting for the base.
      Serial.println("Home colour not found - back to exploring");
      home_retry_after_ms = millis() + HOME_RETRY_COOLDOWN_MS;
      resume_search();
    }
    return;
  }
  apply_reflex(ranges, left_pct, right_pct);
  check_stuck(pose, left_pct, right_pct);  // reverse, then the 2 Hz replan finds a way round
}

void state_machine_update(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT], bool weight_seen_this_frame, int &out_left_pct, int &out_right_pct) {
  const uint32_t now = millis();
  if (weight_seen_this_frame) {
    last_weight_seen_ms = now;
  }

  // Head home when full, or from 90 s in with anything on board.
  if (current_state != RETURN_HOME && current_state != DROP_OFF) {
    const int on_board = weight_collection_count();
    if (on_board >= WEIGHTS_BEFORE_HOME) begin_return_home("full");
    else if (on_board > 0 && now - round_start_ms >= EARLY_HOME_MS &&
             (int32_t)(now - home_retry_after_ms) >= 0) begin_return_home("90 s with a weight on board");
  }

  out_left_pct = 0;
  out_right_pct = 0;

  // Backing off after getting stuck (path-following states only).
  if ((int32_t)(reverse_until_ms - now) > 0 &&
      (current_state == EXPLORE || current_state == GO_TO_WEIGHT || current_state == RETURN_HOME)) {
    out_left_pct = out_right_pct = -STUCK_REVERSE_PCT;
    return;
  }

  switch (current_state) {
    case EXPLORE:
      run_explore(pose, ranges, out_left_pct, out_right_pct);
      break;
    case GO_TO_WEIGHT:
      run_go_to_weight(pose, ranges, out_left_pct, out_right_pct);
      break;
    case NAVIGATION:
      run_navigation(pose, ranges, out_left_pct, out_right_pct);
      break;
    case APPROACH_VERIFY:
      run_approach_verify(pose, out_left_pct, out_right_pct);
      break;
    case APPROACH_WEIGHT:
      run_approach_weight(pose, ranges, out_left_pct, out_right_pct);
      break;
    case APPROACH_BLIND:
      run_approach_blind(pose, out_left_pct, out_right_pct);
      break;
    case SCANNING:
      if (USE_FRONTIER_NAV) run_scan_continuous(pose, out_left_pct, out_right_pct);
      else                  run_legacy_scan(pose, out_left_pct, out_right_pct);
      break;
    case RETURN_HOME:
      run_return_home(pose, ranges, out_left_pct, out_right_pct);
      break;
    case DROP_OFF:
      // Only reached again once the release swing has finished (the state machine
      // is not called while collection is busy) - or if it never managed to start.
      if (weight_collection_count() == 0 || now - drop_off_start_ms >= DROP_OFF_TIMEOUT_MS) {
        resume_search();
      }
      break;
  }
}
