#ifndef POSE_H
#define POSE_H

#include <Arduino.h>

/**
 * Robot pose in a fixed global frame.
 * x, y are in metres, theta is heading in radians, wrapped to [-pi, pi],
 * measured counter-clockwise from wherever the robot was pointing at
 * pose_reset() (that starting heading is defined as theta = 0, and the
 * robot's forward direction at that moment is the world +x axis).
 *
 * Body frame convention used inside pose.cpp: x = forward, y = left.
 */
struct Pose {
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
};

/**
 * Initialise the BNO055 IMU and the PMW3901 optical flow sensor.
 * Call once from setup(), after Serial.begin(). If a sensor fails to
 * initialise, a message is printed and pose_imu_ready()/pose_flow_ready()
 * return false (the code does NOT halt) - check those before trusting
 * the pose.
 *
 * @param flow_chip_select  the CS pin the PMW3901 is wired to
 */
void pose_init(uint8_t flow_chip_select);

/**
 * Read both sensors and integrate the pose estimate forward by one step.
 * Call this every loop iteration. The flow sensor reports counts
 * accumulated since the last read, i.e. a displacement, so no loop-time
 * (dt) is needed - but the counts are stored in an int16, so don't let
 * the gap between calls get very long.
 *
 * @return the updated Pose (also available via pose_get())
 */
Pose pose_update();

/**
 * @return the most recently computed pose without taking a new reading.
 */
Pose pose_get();

/**
 * Reset x, y, theta back to zero, re-zero the heading to the current
 * IMU heading, and discard any flow counts accumulated so far.
 */
void pose_reset();

/**
 * @return true if the BNO055 was detected and initialised successfully.
 *         If false, pose.theta will just stay at 0 - heading is not being
 *         updated at all.
 */
bool pose_imu_ready();

/**
 * @return true if the PMW3901 was detected and initialised successfully.
 *         If false, pose.x/pose.y will never move - position is not being
 *         updated at all.
 */
bool pose_flow_ready();

/**
 * Scale factor converting raw PMW3901 motion counts to metres of travel.
 * Each count is a fixed angle of view, so distance per count is
 * proportional to height above the floor:
 *
 *     metres_per_count ~= height_m * 0.0021
 *
 * At 80 mm that is ~0.000168 m/count (0.168 mm/count). This is an
 * estimate - CALIBRATE by pushing the robot a known distance, summing the
 * raw counts, and setting FLOW_METERS_PER_COUNT = distance / counts.
 */
extern float FLOW_METERS_PER_COUNT;

/**
 * Print raw sensor readings (BNO055 heading, PMW3901 raw counts) plus the
 * current integrated pose to Serial. Useful for confirming the sensors are
 * actually producing data before trusting the integrated x/y/theta.
 */
void pose_print_debug();

#endif // POSE_H