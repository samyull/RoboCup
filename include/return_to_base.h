//************************************
//         return_to_base.h
//************************************

#ifndef RETURN_TO_BASE_H_
#define RETURN_TO_BASE_H_

#include "pose.h"
#include "navigation.h"

// Home = the start position of the configured layout (arena_config.h).
Target return_home_target();

// Start heading home (resets the planning state).
void return_home_begin();

// Plan and drive home over ground already seen. If that route is blocked
// (most likely another robot), hold and retry for a few seconds, then allow
// unexplored ground as a last resort. Returns true only within 5 cm of home
// with the configured home colour detected. Holds there awaiting colour confirmation.
bool return_home_update(const Pose &pose, int &left_pct, int &right_pct);

// Detect what base (if any) the robot is above - placeholder
void detect_base();

// Unload weights in home base - placeholder
void unload_weights();

#endif /* RETURN_TO_BASE_H_ */
