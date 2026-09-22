#ifndef TOF_NAV_H
#define TOF_NAV_H

#include <stdint.h>
#include "TOFs.h"
#include "pose.h"

enum TofNavState {
  TOF_NAV_CLEAR,   // nothing detected, caller drives as normal
  TOF_NAV_WEIGHT,  // approaching a weight, left/right set
  TOF_NAV_AVOID    // avoiding an obstacle, left/right set
};

// Call once per loop with the capped ranges from tof_read().

void tof_classify_readings(uint16_t ranges[TOF_TOTAL_COUNT], Pose pose);
#endif