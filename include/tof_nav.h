#ifndef TOF_NAV_H
#define TOF_NAV_H

#include <stdint.h>
#include "TOFs.h"
#include "pose.h"

// Readings are not mapped while turning faster than this. Each reading is placed
// using the pose at the moment it was measured (pose_at), so this only guards
// against the residual error of that estimate.
static const float TOF_MAX_TURN_RATE_RAD_S = 1.0f;

enum TofNavState {
  TOF_NAV_CLEAR,   // nothing dangerously close, caller drives as normal
  TOF_NAV_UNKNOWN, // missing/stale upper reading, outputs zero until fallback is implemented
  TOF_NAV_AVOID    // something close ahead, left/right set to steer away
};

// Fresh, classified weight near the selected target, with both readings newer
// than the end of settling. Updates matched coordinates for grid-cell jitter.
bool tof_confirm_target(float x, float y, uint32_t after_ms, float &matched_x, float &matched_y);
bool tof_observation_after(uint32_t after_ms);

/**
 * Perception: classify each side's bottom/top TOF pair (wall / weight /
 * nothing) and write the result into the occupancy grid. Call once per loop
 * after pose_update(). Returns true if a weight was seen on
 * either side THIS call (a fresh sighting, not "is one known on the map").
 */
// Mounting of a sensor (TofIndex): position in the body frame (m, forward/left)
// and beam direction (rad, 0 = forward, +pi/2 = left). False for a bad index.
bool tof_sensor_geometry(uint8_t index, float &fwd_m, float &left_m, float &facing_rad);

// Robot pose when this reading was measured (current pose if history can't say,
// e.g. just after a pose reset when the robot has not moved).
Pose tof_pose_for_reading(const TofReading &reading, const Pose &current);

// Blind approach: while set, the front pairs neither update the map nor count
// as sightings (they misread very close to a weight). Side pairs still map.
void tof_set_front_blind(bool blind);

// True when both front top sensors are working and neither sees anything closer
// than (distance_m - margin_m) from the robot centre.
bool tof_front_clear_to(const TofReading ranges[TOF_TOTAL_COUNT], float distance_m, float margin_m);

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
