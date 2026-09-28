#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "pose.h"
#include "TOFs.h"

enum StateMachine : int8_t {
  NAVIGATION      = 0,  // legacy waypoint sweep (only when USE_FRONTIER_NAV is false)
  APPROACH_VERIFY = 1,  // just picked a mapped weight - align + confirm it's still real
  APPROACH_WEIGHT = 2,  // confirmed, driving toward it
  SCANNING        = 3,  // 360 on the spot: no reachable frontier (legacy: no weight seen lately)
  RETURN_HOME     = 4,  // full, or 90 s in with a weight on board: path home over seen ground
  DROP_OFF        = 5,  // at home: release, then carry on exploring
  APPROACH_BLIND  = 6,  // final approach with the front ToFs ignored (they misread up close)
  EXPLORE         = 7,  // frontier exploration (default behaviour)
  GO_TO_WEIGHT    = 8,  // path to a mapped weight until close with a clear line of sight
};

// Call at the start of every round: also starts the round clock used for the 90 s rule.
void state_machine_init();

/**
 * Call every loop, AFTER pose_update()/tof_classify_readings(),
 * and ONLY when weight_collection_busy() is false - the catchment interrupt
 * (induction + IR proximity -> pickup) takes full priority over navigation,
 * and while it's busy this function should simply not be called at all, so
 * whatever state we were in is untouched and resumes automatically once
 * weight_collection is done.
 *
 * weight_seen_this_frame should be tof_classify_readings()'s return value
 * from this same loop.
 */
void state_machine_update(const Pose &pose, const TofReading ranges[TOF_TOTAL_COUNT],
                           bool weight_seen_this_frame, int &out_left_pct, int &out_right_pct);

StateMachine state_machine_current_state();
const char *state_machine_state_name();

// True during the blind final approach: the front ToFs must not map, count as
// sightings, trigger the obstacle reflex, or hold the robot for missing data.
bool state_machine_front_blind();

#endif
