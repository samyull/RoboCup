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

static const float    HOME_ARRIVE_M   = 0.15f;
static const uint32_t REPLAN_MS       = 500;
static const uint32_t BLOCKED_WAIT_MS = 3000;  // give a blocking robot time to move

enum HomeRoute {
  ROUTE_SEEN_ONLY,  // normal: only ground already seen
  ROUTE_WAITING,    // seen-only route blocked: holding, retrying
  ROUTE_ANY,        // last resort: unexplored ground allowed
};
static HomeRoute route = ROUTE_SEEN_ONLY;
static bool have_path = false;
static uint32_t last_plan_ms = 0;
static uint32_t blocked_since_ms = 0;

Target return_home_target() {
  return {ROBOT_START_X_M, ROBOT_START_Y_M};
}

void return_home_begin() {
  route = ROUTE_SEEN_ONLY;
  have_path = false;
  last_plan_ms = millis() - REPLAN_MS;  // plan on the next update
  planner_clear_path();
  Serial.println("Returning home");
}

bool return_home_update(const Pose &pose, int &left_pct, int &right_pct) {
  left_pct = right_pct = 0;
  const Target home = return_home_target();
  if (distance_to(pose, home) <= HOME_ARRIVE_M) return true;

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
  if (!have_path) return false;  // hold still while waiting
  return planner_follow(pose, HOME_ARRIVE_M, left_pct, right_pct);
}

// Detect what base (if any) the robot is above
void detect_base(/* Parameters */){
  Serial.println("Base detected \n");
}

// Unload weights in home base
void unload_weights(/* Parameters */){
  Serial.println("Unloading weights \n");
}
