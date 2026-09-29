#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <stdint.h>
#include "pose.h"

// Grid path planning over the 5 cm map. One plan = a distance field from the
// robot to every reachable cell (8-connected, straight step 10, diagonal 14,
// i.e. 10 per 5 cm), so path lengths to every weight and frontier come from a
// single search. The arena border plus a robot half-width is impassable;
// confirmed walls and the robot footprint margin around them are impassable.

#define PLAN_UNREACHABLE 0xFFFF

enum PlanMode {
  PLAN_ALLOW_UNSEEN,  // exploring: unseen ground is assumed free
  PLAN_SEEN_ONLY,     // going home: only ground the robot has already seen
};

void planner_plan(const Pose &pose, PlanMode mode);

uint16_t planner_cost(int gx, int gy);    // PLAN_UNREACHABLE if not reachable
float planner_distance_m(int gx, int gy); // path length in metres (large if unreachable)
// Reachable viewpoint with an extra cell of clearance beyond the drive footprint.
bool planner_goal_clear(int gx, int gy);

// Extract and smooth the path to (gx, gy) from the last plan. False if unreachable.
// Also prints the path ("PATH,x,y,...") and goal ("T,x,y") for the visualiser.
bool planner_set_goal(int gx, int gy);
void planner_clear_path();

// Steer along the current path. Returns true once within arrive_m of its goal.
bool planner_follow(const Pose &pose, float arrive_m, int &left_pct, int &right_pct);

// Straight line between two points crosses no impassable cell (last plan's classes).
bool planner_straight_clear(float x0, float y0, float x1, float y1);

#endif
