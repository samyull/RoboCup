#ifndef TOF_NAV_H
#define TOF_NAV_H

#include <stdint.h>
#include "TOFs.h"
#include "pose.h"


void tof_classify_readings(const uint16_t ranges[TOF_TOTAL_COUNT], const Pose pose);
#endif