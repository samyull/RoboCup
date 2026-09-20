#include "tof_nav.h"
#include <Arduino.h>

// Index order from TOFs.h
static const uint8_t BOTTOM_R = 0, BOTTOM_L = 1, TOP_R = 2, TOP_L = 3;

// Thresholds
static const uint16_t WEIGHT_MM   = TOF_MAX_RANGE_MM; // bottom < this = weight
static const uint16_t OBSTACLE_MM = 200;              // top < this = not a weight

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

TofNavState tof_nav_update(const uint16_t ranges[TOF_TOTAL_COUNT],
                           int &left_pct, int &right_pct) {
  uint32_t now = millis();

  // 1) Obstacle avoidance has priority: a top sensor reading < 200 is not a weight.
  bool tr = ranges[TOP_R] < OBSTACLE_MM;
  bool tl = ranges[TOP_L] < OBSTACLE_MM;
  if (tr || tl) {
    bool turn_left;
    if (tr && tl) turn_left = ranges[TOP_R] <= ranges[TOP_L]; // turn away from the closer one
    else          turn_left = tr;                              // obstacle on right -> turn left

    if (turn_left) { left_pct = -AVOID_SPEED; right_pct =  AVOID_SPEED; }
    else           { left_pct =  AVOID_SPEED; right_pct = -AVOID_SPEED; }

    prev_valid = false;
    ever_seen  = false;
    return TOF_NAV_AVOID;
  }

  // 2) Weight tracking with the bottom sensors.
  bool r_seen = ranges[BOTTOM_R] < WEIGHT_MM;
  bool l_seen = ranges[BOTTOM_L] < WEIGHT_MM;

  if (r_seen || l_seen) {
    // error > 0 means turn right, error < 0 means turn left
    float error;
    if (r_seen && l_seen) {
      // Both see it: equalise the distances to keep it centred.
      error = ((float)ranges[BOTTOM_L] - (float)ranges[BOTTOM_R]) / BOTH_SEEN_SCALE_MM;
      if (error >  1.0f) error =  1.0f;
      if (error < -1.0f) error = -1.0f;
    } else {
      // Only one sees it: turn gently towards that side.
      error = r_seen ? ONE_SIDE_ERROR : -ONE_SIDE_ERROR;
    }

    float derivative = 0.0f;
    if (prev_valid && now > prev_ms) {
      derivative = (error - prev_error) / ((now - prev_ms) / 1000.0f);
    }
    prev_error = error;
    prev_ms    = now;
    prev_valid = true;

    float turn = KP * error + KD * derivative;
    if (turn >  MAX_TURN) turn =  MAX_TURN;
    if (turn < -MAX_TURN) turn = -MAX_TURN;

    left_pct  = APPROACH_SPEED + (int)turn;
    right_pct = APPROACH_SPEED - (int)turn;

    last_seen_ms = now;
    ever_seen    = true;
    return TOF_NAV_WEIGHT;
  }

  // 3) Weight just dropped out of view (likely between the two narrow beams):
  //    keep driving straight for a short time.
  if (ever_seen && (now - last_seen_ms) < HOLD_MS) {
    left_pct = right_pct = APPROACH_SPEED;
    prev_valid = false;
    return TOF_NAV_WEIGHT;
  }

  prev_valid = false;
  ever_seen  = false;
  return TOF_NAV_CLEAR;
}