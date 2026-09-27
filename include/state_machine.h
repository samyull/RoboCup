#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "pose.h"
#include "TOFs.h"

enum StateMachine : int8_t {
  NAVIGATION      = 0,  // lawnmower search sweep (default behaviour)
  APPROACH_VERIFY = 1,  // just picked a mapped weight - align + confirm it's still real
  APPROACH_WEIGHT = 2,  // confirmed, driving toward it
  SCANNING        = 3,  // stop-and-sample 360 sweep, no weight seen in a while
  RETURN_HOME     = 4,  // 3 weights collected, heading back to (0,0)
  DROP_OFF        = 5,  // vestigial for now - colour sort / deposit, added later
};

void state_machine_init();

/**
 * Call every loop, AFTER pose_update()/map_update()/tof_classify_readings(),
 * and ONLY when weight_collection_busy() is false - the catchment interrupt
 * (induction + IR proximity -> pickup) takes full priority over navigation,
 * and while it's busy this function should simply not be called at all, so
 * whatever state we were in is untouched and resumes automatically once
 * weight_collection is done.
 *
 * weight_seen_this_frame should be tof_classify_readings()'s return value
 * from this same loop.
 */
void state_machine_update(const Pose &pose, const uint16_t ranges[TOF_TOTAL_COUNT],
                           bool weight_seen_this_frame, int &out_left_pct, int &out_right_pct);

StateMachine state_machine_current_state();

#endif