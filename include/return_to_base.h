//************************************
//         return_to_base.h
//************************************

#ifndef RETURN_TO_BASE_H_
#define RETURN_TO_BASE_H_

#include "pose.h"
#include "navigation.h"

// Home = the start position of the configured layout (arena_config.h).
Target return_home_target();

enum HomeStatus {
  HOME_TRAVELLING,       // still on the way (or holding for the colour)
  HOME_ARRIVED,          // at home with the home colour seen: unload
  HOME_COLOR_NOT_FOUND,  // at the home point, but no home colour within the timeout
};

// Start heading home (resets the planning state).
void return_home_begin();

// Switch to driving slowly round the home point until the colour sensor sees
// the home colour (used when full and the colour wasn't found at the point).
void return_home_search();

// Plan and drive home over ground already seen. If that route is blocked
// (most likely another robot), hold and retry for a few seconds, then allow
// unexplored ground as a last resort. Arrived only within 5 cm of home with the
// configured home colour detected; if the colour isn't seen within 3 s there,
// returns HOME_COLOR_NOT_FOUND so the caller can decide what to do.
HomeStatus return_home_update(const Pose &pose, int &left_pct, int &right_pct);

// Detect what base (if any) the robot is above - placeholder
void detect_base();

// Unload weights in home base - placeholder
void unload_weights();

#endif /* RETURN_TO_BASE_H_ */
