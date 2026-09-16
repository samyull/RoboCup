#include "navigation.h"
#include <Arduino.h>

float ARRIVAL_RADIUS_M = 0.05f; // 5cm

static const int MAX_FORWARD_PCT = 70;
static const int MAX_TURN_PCT = 60;
static const float TURN_KP = 90.0f; // proportional gain for turning, in % per radian of heading error
static const float FORWARD_CUTOFF_RAD = radians(60.0f);
static const float SLOWDOWN_RADIUS_M = 0.30f;
static const int MIN_APPROACH_PCT = 20;

static float wrap_angle(float angle_rad) {
    while (angle_rad > PI) angle_rad -= 2.0f * PI;
    while (angle_rad < -PI) angle_rad += 2.0f * PI;
    return angle_rad;
}

static int clamp_pct(int value) {
    if (value > 100) return 100;
    if (value < -100) return -100;
    return value;
}

float distance_to(const Pose &pose, const Target &target) {
    float dx = target.x - pose.x;
    float dy = target.y - pose.y;
    return sqrtf(dx * dx + dy * dy);
}

float heading_error_to(const Pose &pose, const Target &target) {
    float dx = target.x - pose.x;
    float dy = target.y - pose.y;
    float heading_to_target = atan2f(dy, dx);
    return wrap_angle(heading_to_target - pose.theta);
}

bool has_arrived(const Pose &pose, const Target &target) {
    return distance_to(pose, target) <= ARRIVAL_RADIUS_M;
}

void navigate_to_target(const Pose &pose, const Target &target, int &out_left_pct, int &out_right_pct) {
    float err = heading_error_to(pose, target);
    float dist = distance_to(pose, target);

    int turn_pct = clamp_pct((int)(TURN_KP * err));
    if (turn_pct > MAX_TURN_PCT) turn_pct = MAX_TURN_PCT;
    if (turn_pct < -MAX_TURN_PCT) turn_pct = -MAX_TURN_PCT;

    int forward_pct = 0;
    if (fabsf(err) < FORWARD_CUTOFF_RAD) {
        float heading_scale = cosf(err);
        forward_pct = (int)(MAX_FORWARD_PCT * heading_scale);

        if (dist < SLOWDOWN_RADIUS_M) {
            float approach_scale = dist / SLOWDOWN_RADIUS_M;
            forward_pct = (int)(forward_pct * approach_scale);
            if (forward_pct > 0 && forward_pct < MIN_APPROACH_PCT) {
                forward_pct = MIN_APPROACH_PCT;
            }
        }
    }
}