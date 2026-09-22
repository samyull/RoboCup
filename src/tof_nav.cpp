#include "tof_nav.h"
#include <Arduino.h>

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

// // Thresholds
// static const float DETECTION_M = 0.3;
// static const uint16_t WEIGHT_MM   = TOF_MAX_RANGE_MM; // bottom < this = weight
// static const uint16_t OBSTACLE_MM = 200;              // top < this = not a weight

// Speeds (percent)
static const int APPROACH_SPEED = 40;
static const int AVOID_SPEED    = 40;   // spin-in-place speed

// PD tuning: starting values, tune on the robot
static const float KP        = 35.0f;
static const float KD        = 1.5f;
static const float MAX_TURN  = 40.0f;
static const float BOTH_SEEN_SCALE_MM = 200.0f; // L-R difference that gives full error
static const float ONE_SIDE_ERROR     = 0.5f;   // error when only one sensor sees it
static const uint32_t HOLD_MS = 400;  // keep driving straight after the weight drops out of view

static float    prev_error  = 0.0f;
static bool     prev_valid  = false;
static uint32_t prev_ms     = 0;
static uint32_t last_seen_ms = 0;
static bool     ever_seen   = false;

void tof_classify_readings(const uint16_t ranges[TOF_TOTAL_COUNT], Pose pose) {
  float topRight = (float)ranges[TOP_R] * MM_TO_M;
  float topLeft = (float)ranges[TOP_L] * MM_TO_M;
  float bottomRight = (float)ranges[BOTTOM_R] * MM_TO_M;
  float bottomLeft = (float)ranges[BOTTOM_L] * MM_TO_M;

  bool weightRight = fabs(topRight - bottomRight) > WALL_TOLERANCE_M && bottomRight < DETECTION_MAX_M;
  bool weightLeft = fabs(topLeft - bottomLeft) > WALL_TOLERANCE_M && bottomLeft < DETECTION_MAX_M;

  bool wallRight = topRight < DETECTION_MAX_M;
  bool wallLeft = topLeft < DETECTION_MAX_M;


  // Right side
  if(weightRight) {
    // Weight detected right side, write it to the grid @ current pos + heading * bottomRight
  }
  if(wallRight) {
    // Wall detected right side, write it to the grid @ current pos + heading * avg(bottomRight, topRight)
    float wallDist = !weightRight ? (bottomRight + topRight) * 0.5f : topRight;
  }
  if(!wallRight && !weightRight) {
    // Nothing detected right side, write free space to the grid in a line from current pos to heading * DETECTION_MAX_M
  }

  // Left side
  if(weightLeft) {
    // Weight detected right side, write it to the grid @ current pos + heading * bottomRight
  }
  if(wallLeft) {
    // Wall detected right side, write it to the grid @ current pos + heading * avg(bottomRight, topRight)
    float wallDist = !weightLeft ? (bottomLeft + topLeft) * 0.5f : topRight;
  }
  if(!wallLeft && !weightLeft) {
    // Nothing detected right side, write free space to the grid in a line from current pos to heading * DETECTION_MAX_M
  }
}