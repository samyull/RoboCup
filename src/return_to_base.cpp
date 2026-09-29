//************************************
//         return_to_base.cpp
//************************************

 // This file contains functions used to return to and
 // detect bases

#include "return_to_base.h"
#include "path_planner.h"
#include "grid_map.h"
#include "arena_config.h"
#include "Arduino.h"

static const float    HOME_ARRIVE_M   = 0.05f;
static const uint32_t REPLAN_MS       = 500;
static const uint32_t BLOCKED_WAIT_MS = 3000;  // give a blocking robot time to move
// At the home point but the colour sensor doesn't see the home colour: give up after this.
static const uint32_t HOME_COLOR_TIMEOUT_MS = 3000;
// Searching around home (when full): circle of points this far from home,
// kept this far inside the arena walls.
static const float    SEARCH_RADIUS_M      = 0.12f;
static const float    SEARCH_WALL_MARGIN_M = 0.16f;  // robot half-width + a little
static const int      SEARCH_POINTS        = 8;
static const float    SEARCH_POINT_REACHED_M = 0.04f;

enum HomeRoute {
  ROUTE_SEEN_ONLY,  // normal: only ground already seen
  ROUTE_WAITING,    // seen-only route blocked: holding, retrying
  ROUTE_ANY,        // last resort: unexplored ground allowed
};
static HomeRoute route = ROUTE_SEEN_ONLY;
static bool have_path = false;
static uint32_t last_plan_ms = 0;
static uint32_t blocked_since_ms = 0;
static bool waiting_for_color = false;
static uint32_t waiting_since_ms = 0;
static bool searching = false;
static int search_index = 0;

Target return_home_target() {
  return {ROBOT_START_X_M, ROBOT_START_Y_M};
}

void return_home_begin() {
  route = ROUTE_SEEN_ONLY;
  have_path = false;
  waiting_for_color = false;
  searching = false;
  last_plan_ms = millis() - REPLAN_MS;  // plan on the next update
  planner_clear_path();
  Serial.println("Returning home");
}

void return_home_search() {
  searching = true;
  search_index = 0;
  planner_clear_path();
  Serial.println("Home colour not found - searching around home");
}

// Search point i: on a circle round home, pulled back inside the arena walls.
static Target search_point(int i) {
  const Target home = return_home_target();
  const float a = i * 2.0f * PI / SEARCH_POINTS;
  Target t = {home.x + SEARCH_RADIUS_M * cosf(a), home.y + SEARCH_RADIUS_M * sinf(a)};
  t.x = constrain(t.x, SEARCH_WALL_MARGIN_M, MAP_GRID_SIZE_X_M - SEARCH_WALL_MARGIN_M);
  t.y = constrain(t.y, SEARCH_WALL_MARGIN_M, MAP_GRID_SIZE_Y_M - SEARCH_WALL_MARGIN_M);
  return t;
}

HomeStatus return_home_update(const Pose &pose, int &left_pct, int &right_pct) {
  left_pct = right_pct = 0;
  const Target home = return_home_target();

  if (searching) {
    // Driving slowly round home until the colour sensor sees the base.
    if (color_sensor_color() == ROBOT_HOME_COLOR) {
      Serial.println("Home colour found");
      return HOME_ARRIVED;
    }
    if (distance_to(pose, search_point(search_index)) <= SEARCH_POINT_REACHED_M)
      search_index = (search_index + 1) % SEARCH_POINTS;
    navigate_to_target(pose, search_point(search_index), left_pct, right_pct);
    return HOME_TRAVELLING;
  }

  // Both position and base colour must agree before unloading. Hold still
  // inside the home radius while waiting for the colour sensor to confirm,
  // but not forever (sensor missing, misclassified, or pose drifted off the base).
  if (distance_to(pose, home) <= HOME_ARRIVE_M) {
    if (color_sensor_color() == ROBOT_HOME_COLOR) return HOME_ARRIVED;
    if (!waiting_for_color) {
      waiting_for_color = true;
      waiting_since_ms = millis();
    } else if (millis() - waiting_since_ms >= HOME_COLOR_TIMEOUT_MS) {
      waiting_for_color = false;
      Serial.print("At home but colour reads "); Serial.println(color_name(color_sensor_color()));
      return HOME_COLOR_NOT_FOUND;
    }
    return HOME_TRAVELLING;
  }
  waiting_for_color = false;

  const uint32_t now = millis();
  if (now - last_plan_ms >= REPLAN_MS) {
    last_plan_ms = now;
    int gx, gy;
    world_to_grid(home.x, home.y, gx, gy);
    planner_plan(pose, route == ROUTE_ANY ? PLAN_ALLOW_UNSEEN : PLAN_SEEN_ONLY);
    have_path = planner_set_goal(gx, gy);
    if (have_path) {
      if (route == ROUTE_WAITING) {
        Serial.println("Home route clear again");
        route = ROUTE_SEEN_ONLY;
      }
    } else if (route == ROUTE_SEEN_ONLY) {
      Serial.println("Home route over seen ground blocked (another robot?) - waiting");
      route = ROUTE_WAITING;
      blocked_since_ms = now;
    } else if (route == ROUTE_WAITING && now - blocked_since_ms >= BLOCKED_WAIT_MS) {
      Serial.println("Home route still blocked - allowing unexplored ground");
      route = ROUTE_ANY;
      last_plan_ms = now - REPLAN_MS;  // replan straight away with the wider search
    }
  }
  if (!have_path) return HOME_TRAVELLING;  // hold still while waiting
  if (planner_follow(pose, HOME_ARRIVE_M, left_pct, right_pct)) {
    // The planner targets a rounded grid cell. Finish at the exact configured
    // home position; reaching the path endpoint alone cannot confirm arrival.
    navigate_to_target(pose, home, left_pct, right_pct);
  }
  return HOME_TRAVELLING;
}

// Detect what base (if any) the robot is above
void detect_base(/* Parameters */){
  Serial.println("Base detected \n");
}

// Unload weights in home base
void unload_weights(/* Parameters */){
  Serial.println("Unloading weights \n");
}
