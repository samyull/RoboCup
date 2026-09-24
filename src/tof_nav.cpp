#include "tof_nav.h"
#include <Arduino.h>
#include "grid_map.h"

// Relevant constants
static const float MM_TO_M = 0.001f;
static const float WALL_TOLERANCE_M = 0.05f; 
static const float DETECTION_MAX_M = (float)(TOF_MAX_RANGE_MM - 50) * MM_TO_M;

// Index order from TOFs.h
static const uint8_t BOTTOM_R = 0, BOTTOM_L = 1, TOP_R = 2, TOP_L = 3;

// TOF offsets
static const float TOP_LEFT_FWD_M    =  0.1506f;
static const float TOP_LEFT_LEFT_M   =  0.06164f;
static const float BOTTOM_LEFT_FWD_M =  0.174f;
static const float BOTTOM_LEFT_LEFT_M=  0.06164f;

static const float TOP_RIGHT_FWD_M    =  0.1506f;
static const float TOP_RIGHT_LEFT_M   = -0.08975f;
static const float BOTTOM_RIGHT_FWD_M =  0.174f;
static const float BOTTOM_RIGHT_LEFT_M= -0.08975f;

static void project_point(const Pose &pose, float fwd_offset, float left_offset, float distance, float &x_m, float &y_m) {
  // Get the position of detected point in terms of body reference frame
  float body_fwd = fwd_offset + distance;
  float body_left = left_offset;

  // Cosine and sine
  float c = cosf(pose.theta);
  float s = sinf(pose.theta);

  // Convert to world frame
  x_m = pose.x + body_fwd * c - body_left * s;
  y_m = pose.y + body_fwd * s + body_left * c;
}

void tof_classify_readings(const uint16_t ranges[TOF_TOTAL_COUNT], const Pose &pose) {
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

  // Right side
  if (weightRight) {
    // Weight detected right side, write it to the grid @ current pos + heading * bottomRight
    project_point(pose, BOTTOM_RIGHT_FWD_M, BOTTOM_RIGHT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, BOTTOM_RIGHT_FWD_M, BOTTOM_RIGHT_LEFT_M, bottomRight, reading_x_m, reading_y_m);

    Serial.print("ray: "); Serial.print(origin_x); Serial.print(",");
    Serial.print(origin_y); Serial.print(" -> "); Serial.print(reading_x_m);
    Serial.print(","); Serial.println(reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);

    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_WEIGHT);
  }
  if(wallRight) {
    // Wall detected right side, write it to the grid @ current pos + heading * avg(bottomRight, topRight)
    float wallDist = !weightRight ? (bottomRight + topRight) * 0.5f : topRight;

    project_point(pose, TOP_RIGHT_FWD_M, TOP_RIGHT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, TOP_RIGHT_FWD_M, TOP_RIGHT_LEFT_M, wallDist, reading_x_m, reading_y_m);

    Serial.print("ray: "); Serial.print(origin_x); Serial.print(",");
    Serial.print(origin_y); Serial.print(" -> "); Serial.print(reading_x_m);
    Serial.print(","); Serial.println(reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);

    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_OBSTACLE);

  }
  if(!wallRight && !weightRight) {
    // Nothing detected right side, write free space to the grid in a line from current pos to heading * DETECTION_MAX_M
    
    project_point(pose, TOP_RIGHT_FWD_M, TOP_RIGHT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, TOP_RIGHT_FWD_M, TOP_RIGHT_LEFT_M, DETECTION_MAX_M, reading_x_m, reading_y_m);

    Serial.print("ray: "); Serial.print(origin_x); Serial.print(",");
    Serial.print(origin_y); Serial.print(" -> "); Serial.print(reading_x_m);
    Serial.print(","); Serial.println(reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);

    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_FREE);
  }

  // Left side
  if(weightLeft) {
    // Weight detected left side, write it to the grid @ current pos + heading * bottomLeft
    project_point(pose, BOTTOM_LEFT_FWD_M, BOTTOM_LEFT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, BOTTOM_LEFT_FWD_M, BOTTOM_LEFT_LEFT_M, bottomLeft, reading_x_m, reading_y_m);

    Serial.print("ray: "); Serial.print(origin_x); Serial.print(",");
    Serial.print(origin_y); Serial.print(" -> "); Serial.print(reading_x_m);
    Serial.print(","); Serial.println(reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);

    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_WEIGHT);
  }
  if(wallLeft) {
    // Wall detected right side, write it to the grid @ current pos + heading * avg(bottomRight, topRight)
    float wallDist = !weightLeft ? (bottomLeft + topLeft) * 0.5f : topLeft;

    project_point(pose, TOP_LEFT_FWD_M, TOP_LEFT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, TOP_LEFT_FWD_M, TOP_LEFT_LEFT_M, wallDist, reading_x_m, reading_y_m);

    Serial.print("ray: "); Serial.print(origin_x); Serial.print(",");
    Serial.print(origin_y); Serial.print(" -> "); Serial.print(reading_x_m);
    Serial.print(","); Serial.println(reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);

    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_OBSTACLE);
  }
  if(!wallLeft && !weightLeft) {
    // Nothing detected right side, write free space to the grid in a line from current pos to heading * DETECTION_MAX_M
    project_point(pose, TOP_LEFT_FWD_M, TOP_LEFT_LEFT_M, 0.0f, origin_x, origin_y);
    project_point(pose, TOP_LEFT_FWD_M, TOP_LEFT_LEFT_M, DETECTION_MAX_M, reading_x_m, reading_y_m);

    Serial.print("ray: "); Serial.print(origin_x); Serial.print(",");
    Serial.print(origin_y); Serial.print(" -> "); Serial.print(reading_x_m);
    Serial.print(","); Serial.println(reading_y_m);
    map_ray_trace(origin_x, origin_y, reading_x_m, reading_y_m);

    world_to_grid(reading_x_m, reading_y_m, reading_gx, reading_gy);
    map_set_cell(reading_gx, reading_gy, MAP_CELL_FREE);
  }
}