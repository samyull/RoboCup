#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "pose.h"

struct Target {
    float x;
    float y;
};

extern float ARRIVAL_RADIUS_M;          // corner/waypoint arrival tolerance
extern float WEIGHT_ARRIVAL_RADIUS_M;   // fallback "close enough" tolerance when approaching a weight

// radius_m defaults to ARRIVAL_RADIUS_M; pass WEIGHT_ARRIVAL_RADIUS_M explicitly
// when checking arrival at a weight target.
bool has_arrived(const Pose &pose, const Target &target, float radius_m = ARRIVAL_RADIUS_M);

float heading_error_to(const Pose &pose, const Target &target);
float distance_to(const Pose &pose, const Target &target);

void navigate_to_target(const Pose &pose, const Target &target, int &out_left_pct, int &out_right_pct);

// Scans the occupancy grid for the MAP_CELL_WEIGHT cell nearest the robot.
// Returns false (target untouched) if none is currently known.
bool find_nearest_weight(const Pose &pose, Target &target);

#endif // NAVIGATION_H