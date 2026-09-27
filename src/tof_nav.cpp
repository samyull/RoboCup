#include "tof_nav.h"
#include <Arduino.h>
#include "grid_map.h"

// Relevant constants
static const float MM_TO_M = 0.001f;
static const float WALL_TOLERANCE_M = 0.015f;
static const float DETECTION_MAX_M = (float)(TOF_MAX_RANGE_MM - 50) * MM_TO_M;

// Index order from TOFs.h
static const uint8_t BOTTOM_R = 0, BOTTOM_L = 1, TOP_R = 2, TOP_L = 3;

// TOF offsets (metres, body frame: forward, left)
static const float TOP_LEFT_FWD_M    =  0.1506f;
static const float TOP_LEFT_LEFT_M   =  0.06164f;
static const float BOTTOM_LEFT_FWD_M =  0.174f;
static const float BOTTOM_LEFT_LEFT_M=  0.06164f;

static const float TOP_RIGHT_FWD_M    =  0.1506f;
static const float TOP_RIGHT_LEFT_M   = -0.08975f;
static const float BOTTOM_RIGHT_FWD_M =  0.174f;
static const float BOTTOM_RIGHT_LEFT_M= -0.08975f;

// --- Reflex-only constants ---
// Tighter than DETECTION_MAX_M on purpose - this is a last-resort override,
// not the primary navigation signal. Tune against real hardware.
static const uint16_t IMMINENT_MM = 150;
static const int AVOID_SPEED = 40;   // spin-in-place speed while avoiding

static void project_point(const Pose &pose, float fwd_offset, float left_offset, float distance, float &x_m, float &y_m) {
  float body_fwd = fwd_offset + distance;
  float body_left = left_offset;

  float c = cosf(pose.theta);
  float s = sinf(pose.theta);

  x_m = pose.x + body_fwd * c - body_left * s;
  y_m = pose.y + body_fwd * s + body_left * c;
}

bool tof_classify_readings(const uint16_t ranges[TOF_TOTAL_COUNT], const Pose &pose) {
  float topRight = (float)ranges[TOP_R] * MM_TO_M;
  float topLeft = (float)ranges[TOP_L] * MM_TO_M;
  float bottomRight = (float)ranges[BOTTOM_R] * MM_TO_M;
  float bottomLeft = (float)ranges[BOTTOM_L] * MM_TO_M;

  bool weightRight = fabs(topRight - bottomRight) > WALL_TOLERANCE_M && bottomRight < DETECTION_MAX_M;
  bool weightLeft = fabs(topLeft - bottomLeft) > WALL_TOLERANCE_M && bottomLeft < DETECTION_MAX_M;

  bool wallRight = topRight < DETECTION_MAX_M;
  bool wallLeft = topLeft < DETECTION_MAX_M;

  float origin_x, origin_y;
  float reading_x_m, reading_y_m;
  int reading_gx, reading_gy;

  bool weightFound = false;

  // Right side
  if (weightRight) {
    project_point(pose, BOTTOM_RIGHT_FWD_M, BOTTOM_RIGHT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, BOTTOM_RIGHT_FWD_M, BOTTOM_RIGHT_LEFT_M, bottomRight, reading_x_m, reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);
    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_WEIGHT);
    weightFound = true;
  }
  if (wallRight) {
    float wallDist = !weightRight ? (bottomRight + topRight) * 0.5f : topRight;
    project_point(pose, TOP_RIGHT_FWD_M, TOP_RIGHT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, TOP_RIGHT_FWD_M, TOP_RIGHT_LEFT_M, wallDist, reading_x_m, reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);
    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_OBSTACLE);
  }
  if (!wallRight && !weightRight) {
    project_point(pose, TOP_RIGHT_FWD_M, TOP_RIGHT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, TOP_RIGHT_FWD_M, TOP_RIGHT_LEFT_M, DETECTION_MAX_M, reading_x_m, reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);
    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_FREE);
  }

  // Left side
  if (weightLeft) {
    project_point(pose, BOTTOM_LEFT_FWD_M, BOTTOM_LEFT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, BOTTOM_LEFT_FWD_M, BOTTOM_LEFT_LEFT_M, bottomLeft, reading_x_m, reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);
    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_WEIGHT);
    weightFound = true;
  }
  if (wallLeft) {
    float wallDist = !weightLeft ? (bottomLeft + topLeft) * 0.5f : topLeft;
    project_point(pose, TOP_LEFT_FWD_M, TOP_LEFT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, TOP_LEFT_FWD_M, TOP_LEFT_LEFT_M, wallDist, reading_x_m, reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);
    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_OBSTACLE);
  }
  if (!wallLeft && !weightLeft) {
    project_point(pose, TOP_LEFT_FWD_M, TOP_LEFT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, TOP_LEFT_FWD_M, TOP_LEFT_LEFT_M, DETECTION_MAX_M, reading_x_m, reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);
    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_FREE);
  }

  return weightFound;
}

TofNavState tof_nav_update(const uint16_t ranges[TOF_TOTAL_COUNT], int &left_pct, int &right_pct) {
  bool r = ranges[TOP_R] < IMMINENT_MM;
  bool l = ranges[TOP_L] < IMMINENT_MM;

  if (!r && !l) return TOF_NAV_CLEAR;

  bool turn_left;
  if (r && l) turn_left = ranges[TOP_R] <= ranges[TOP_L];
  else        turn_left = r;

  if (turn_left) { left_pct = -AVOID_SPEED; right_pct =  AVOID_SPEED; }
  else           { left_pct =  AVOID_SPEED; right_pct = -AVOID_SPEED; }

  return TOF_NAV_AVOID;
}