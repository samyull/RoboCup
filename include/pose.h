#ifndef POSE_H
#define POSE_H

#include <Arduino.h>

/**
 * Robot pose in a fixed global frame.
 * x, y are in metres, theta is heading in radians, wrapped to [-pi, pi],
 * measured counter-clockwise from wherever the robot was pointing at
 * pose_init() (i.e. that starting heading is defined as theta = 0).
 */
struct Pose {
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
};

/**
 * Initialise the BNO055 IMU and the PMW3901 optical flow sensor.
 * Call once from setup(), after Serial.begin(). Halts with a Serial
 * message if either sensor fails to initialise, since dead reckoning
 * is meaningless without both.
 *
 * @param flow_chip_select  the CS pin the PMW3901 is wired to
 */
void pose_init(uint8_t flow_chip_select);

/**
 * Read both sensors and integrate the pose estimate forward by one step.
 * Call this every loop iteration, as close to a fixed rate as practical -
 * the optical flow scale factor below assumes a roughly constant loop time.
 *
 * @return the updated Pose (also available via pose_get())
 */
Pose pose_update();

/**
 * @return the most recently computed pose without taking a new reading.
 */
Pose pose_get();

/**
 * Reset x, y, theta back to zero. Handy for re-anchoring when you know
 * the robot is at a known reference point.
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
 * The PMW3901 reports counts proportional to angular motion of the ground
 * pattern under the sensor, so the real-world distance per count depends
 * on height above the surface. This constant assumes a fixed, known
 * mounting height - MEASURE AND CALIBRATE THIS for your actual rover
 * rather than trusting the placeholder value.
 *
 * Rough starting point for the PMW3901 at ~80mm height: ~0.035 mm/count,
 * i.e. 0.000035 m/count. Recalibrate by driving a known distance and
 * comparing to the accumulated count.
 */
extern float FLOW_METERS_PER_COUNT;

/**
 * Print raw sensor readings (BNO055 heading, PMW3901 raw counts) plus the
 * current integrated pose to Serial. Useful for confirming the sensors are
 * actually producing data before trusting the integrated x/y/theta.
 */
void pose_print_debug();

#endif // POSE_H