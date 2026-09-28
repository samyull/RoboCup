#include "wall_anchor.h"
#include "tof_nav.h"
#include "grid_map.h"
#include <Arduino.h>

static const float MAX_RANGE_M     = 1.5f;   // longer readings are too noisy to anchor on
static const float MAX_ERROR_M     = 0.05f;  // must agree with the predicted wall distance
static const float MAX_INCIDENCE   = radians(30.0f);  // beam within this of square-on to the wall
static const float GAIN            = 0.25f;  // fraction of the error corrected per reading

// Top sensors only: they see walls and never weights.
static const uint8_t ANCHOR_TOFS[] = {TOF_LFT, TOF_RFT, TOF_LST, TOF_RST};

static bool is_front(uint8_t index) {
  return index == TOF_LFT || index == TOF_RFT;
}

// Distance along a beam from (sx, sy) at angle dir to the arena boundary, and
// which axis that wall is normal to. False if the sensor is outside the arena.
static bool predict_wall(float sx, float sy, float dir, float &range, bool &x_wall) {
  if (sx <= 0.0f || sx >= MAP_GRID_SIZE_X_M || sy <= 0.0f || sy >= MAP_GRID_SIZE_Y_M) return false;
  const float c = cosf(dir), s = sinf(dir);
  float tx = 1e9f, ty = 1e9f;
  if (c > 1e-3f) tx = (MAP_GRID_SIZE_X_M - sx) / c;
  else if (c < -1e-3f) tx = -sx / c;
  if (s > 1e-3f) ty = (MAP_GRID_SIZE_Y_M - sy) / s;
  else if (s < -1e-3f) ty = -sy / s;
  x_wall = tx <= ty;
  range = x_wall ? tx : ty;
  return true;
}

void wall_anchor_update(const TofReading ranges[TOF_TOTAL_COUNT], const Pose &current, bool front_blind) {
  if (!pose_heading_valid() || !isfinite(angular_speed_rad_s()) ||
      fabsf(angular_speed_rad_s()) > TOF_MAX_TURN_RATE_RAD_S) return;

  for (uint8_t index : ANCHOR_TOFS) {
    const TofReading &reading = ranges[index];
    if (!reading.new_this_loop || !tof_reading_usable(reading)) continue;
    if (front_blind && is_front(index)) continue;
    const float measured = reading.range_mm * 0.001f;
    if (measured > MAX_RANGE_M) continue;

    float fwd, left, facing;
    if (!tof_sensor_geometry(index, fwd, left, facing)) continue;
    const Pose p = tof_pose_for_reading(reading, current);
    const float c = cosf(p.theta), s = sinf(p.theta);
    const float sx = p.x + fwd * c - left * s;
    const float sy = p.y + fwd * s + left * c;
    const float dir = p.theta + facing;

    float predicted;
    bool x_wall;
    if (!predict_wall(sx, sy, dir, predicted, x_wall)) continue;
    // Square-on only: at glancing angles small heading errors swamp the range.
    const float normal_component = x_wall ? fabsf(cosf(dir)) : fabsf(sinf(dir));
    if (normal_component < cosf(MAX_INCIDENCE)) continue;
    const float error = measured - predicted;
    if (fabsf(error) > MAX_ERROR_M) continue;  // something else (robot, obstacle) in the way

    // Reading longer than predicted -> actually further from that wall, and v.v.
    if (x_wall) pose_nudge_position(-GAIN * error * cosf(dir), 0.0f);
    else        pose_nudge_position(0.0f, -GAIN * error * sinf(dir));
  }
}
