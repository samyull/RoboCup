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
 * @return the updated Pose
 */
Pose pose_update();

// The current estimate without taking a new reading (e.g. after a correction).
Pose pose_current();

/**
 * Pose at time t_ms (millis), interpolated from the last ~1 s of pose_update()
 * results. A time after the newest sample returns the newest. Returns false
 * (out untouched) if there is no history or t_ms is older than it covers.
 * History is cleared by pose_reset(), pose_set_position() and pose_set_heading().
 */
bool pose_at(uint32_t t_ms, Pose &out);

// Reacquire heading and discard flow accumulated while collection held driving stopped.
void pose_resume_after_collection();

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

// Latest heading transaction succeeded and the sample is less than 200 ms old.
bool pose_heading_valid();

// No heading received yet, or at least 200 ms since the last valid sample.
bool pose_heading_stale();

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

// BNO055 calibration levels, 0 (uncalibrated) to 3 (fully calibrated). All 0 if the IMU is not ready.
void pose_get_calibration(uint8_t &sys, uint8_t &gyro, uint8_t &accel, uint8_t &mag);

void pose_set_position(float x_m, float y_m);

// Shift x/y by a small correction (wall re-anchoring). Pose history is shifted
// too, so readings keep being placed consistently.
void pose_nudge_position(float dx_m, float dy_m);

// Make the current IMU heading read as theta_rad (e.g. the start layout's
// heading); later readings follow on from it.
void pose_set_heading(float theta_rad);

float angular_speed_rad_s();

#endif // POSE_H
