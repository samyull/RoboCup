#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <stdint.h>
#include "pose.h"
#include "grid_map.h"

// What has been seen, for frontier exploration. Two layers:
//  - fine: one flag per 5 cm map cell, set when a front bottom ToF cone (the
//    only sensors that can spot a weight) or the robot's own footprint covered it;
//  - coarse: 20 cm cells (4x4 map cells) over the arena interior, "explored"
//    once enough of their fine cells have been seen. Explored is one-and-done.
#define EXPLORE_FINE_PER_CELL 4
#define EXPLORE_W ((MAP_GRID_W - 2 + EXPLORE_FINE_PER_CELL - 1) / EXPLORE_FINE_PER_CELL)
#define EXPLORE_H ((MAP_GRID_H - 2 + EXPLORE_FINE_PER_CELL - 1) / EXPLORE_FINE_PER_CELL)

void exploration_init();

void exploration_mark_fine_seen(int gx, int gy);
void exploration_mark_footprint(const Pose &pose);
bool exploration_fine_seen(int gx, int gy);
bool exploration_explored(int cx, int cy);

// Serial: "V,cx,cy" once per newly explored coarse cell; "V_RESET" on a reset.
void exploration_publish_changes();   // call every loop (bounded output)
void exploration_publish_all();       // full resend (the 'M' command)

// Whole arena explored: forget it and start a second pass.
void exploration_reset();

struct FrontierGoal {
  int gx, gy;      // reachable map cell to drive to
  float x, y;      // its world position (m)
  float score;
};

// Best frontier using the planner's current distance field (call planner_plan
// first). Keeps the previous goal unless another scores clearly better.
bool exploration_choose_frontier(const Pose &pose, FrontierGoal &goal);

// The current goal could not be reached (stuck/unreachable): skip it for a while.
void exploration_skip_current_goal();

#endif
