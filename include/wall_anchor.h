#ifndef WALL_ANCHOR_H
#define WALL_ANCHOR_H

#include "TOFs.h"
#include "pose.h"

// Position correction against the four OUTER arena walls only (internal walls
// move between rounds and can be shoved). A top ToF reading that hits an outer
// wall close to square-on, and agrees with the predicted distance to within
// 5 cm, nudges x or y part of the way towards the measured value. Heading is
// never changed. Call once per loop after tof_read(), with a fresh pose.
// front_blind: skip the front sensors (blind approach - they misread up close).
void wall_anchor_update(const TofReading ranges[TOF_TOTAL_COUNT], const Pose &current, bool front_blind);

#endif
