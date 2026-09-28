#ifndef TOF_NAV_H
#define TOF_NAV_H

#include <stdint.h>
#include "TOFs.h"
#include "pose.h"

enum TofNavState {
  TOF_NAV_CLEAR,   // nothing dangerously close, caller drives as normal
  TOF_NAV_UNKNOWN, // missing/stale upper reading, outputs zero until fallback is implemented
  TOF_NAV_AVOID    // something close ahead, left/right set to steer away
};

/**
 * Perception: classify each side's bottom/top TOF pair (wall / weight /
 * nothing) and write the result into the occupancy grid. Call once per loop
 * after pose_update(). Returns true if a weight was seen on
 * either side THIS call (a fresh sighting, not "is one known on the map").
 */
// True only when both sides supplied usable new observations this call.
bool tof_classification_complete();
// Fresh, classified weight near the selected target, with both readings newer
// than the end of settling. Updates matched coordinates for grid-cell jitter.
bool tof_confirm_target(float x, float y, uint32_t after_ms, float &matched_x, float &matched_y);
bool tof_observation_after(uint32_t after_ms);

bool tof_classify_readings(const TofReading ranges[TOF_TOTAL_COUNT], const Pose &pose);

/**
 * Reflex: fast, independent-of-classification collision check using only
 * the top sensors at a tight threshold. Meant to override whatever the
 * navigation logic is doing the instant something is dangerously close -
 * call this AFTER deciding what navigation wants to do, and let it
 * overwrite left_pct/right_pct if it returns anything other than TOF_NAV_CLEAR.
 */
TofNavState tof_nav_update(const TofReading ranges[TOF_TOTAL_COUNT], int &left_pct, int &right_pct);

#endif
