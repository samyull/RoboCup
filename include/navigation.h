#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "pose.h"

struct Target {
    float x;
    float y;
};

extern float ARRIVAL_RADIUS_M;

bool has_arrived(const Pose &pose, const Target &target);

float heading_error_to(const Pose &pose, const Target &target);

float distance_to(const Pose &pose, const Target &target);

void navigate_to_target(const Pose &pose, const Target &target, int &out_left_pct, int &out_right_pct);

#endif // NAVIGATION_H