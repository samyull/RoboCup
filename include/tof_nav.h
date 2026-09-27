#ifndef TOF_NAV_H
#define TOF_NAV_H

#include <stdint.h>
#include "TOFs.h"
#include "pose.h"

enum TofNavState {
  TOF_NAV_CLEAR,   // nothing dangerously close, caller drives as normal
  TOF_NAV_AVOID    // something close ahead, left/right set to steer away
};

/**
 * Perception: classify each side's bottom/top TOF pair (wall / weight /
 * nothing) and write the result into the occupancy grid. Call once per loop
 * after pose_update()/map_update(). Returns true if a weight was seen on
 * either side THIS call (a fresh sighting, not "is one known on the map").
 */
bool tof_classify_readings(const uint16_t ranges[TOF_TOTAL_COUNT], const Pose &pose);

/**
 * Reflex: fast, independent-of-classification collision check using only
 * the top sensors at a tight threshold. Meant to override whatever the
 * navigation logic is doing the instant something is dangerously close -
 * call this AFTER deciding what navigation wants to do, and let it
 * overwrite left_pct/right_pct if it returns TOF_NAV_AVOID.
 */
TofNavState tof_nav_update(const uint16_t ranges[TOF_TOTAL_COUNT], int &left_pct, int &right_pct);

#endif