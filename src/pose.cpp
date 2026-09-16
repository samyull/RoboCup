#include "pose.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Bitcraze_PMW3901.h>

// CALIBRATE THIS GUY
float FLOW_METERS_PER_COUNT = 0.0000035f;

static Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
static Bitcraze_PMW3901 *flow_sensor = nullptr;

static Pose current_pose;
static float theta_offset = 0.0f; // heading at pose_init/reset, subtracted out
static bool imu_ready = false;
static bool flow_ready = false;

// Wrap an angle in radians to [-pi, pi]
static float wrap_angle(float angle_rad) {
    while (angle_rad > PI) angle_rad -= 2.0f * PI;
    while (angle_rad < -PI) angle_rad += 2.0f * PI;
    return angle_rad;
}

void pose_init(uint8_t flow_chip_select) {
    Wire.begin();

    if (!bno.begin()) {
        Serial.println("ERROR: BNO055 not detected - check wiring/address!");
    } else {
        bno.setExtCrystalUse(true);
        imu_ready = true;
        Serial.println("BNO055 initialised");
    }

    flow_sensor = new Bitcraze_PMW3901(flow_chip_select);
    if (!flow_sensor->begin()) {
        Serial.println("ERROR: PMW3901 not detected - check wiring/CS pin!");
    } else {
        flow_ready = true;
        Serial.println("PMW3901 initialised");
    }

    pose_reset();
}

void pose_reset() {
    current_pose = Pose();

    if (imu_ready) {
        sensors_event_t orientation;
        bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
        theta_offset = wrap_angle(-radians(orientation.orientation.x)); // store the current heading as offset
    }
}

Pose pose_update() {
    if (imu_ready) {
        sensors_event_t orientation;
        bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
        float heading_ccw = -radians(orientation.orientation.x);
        current_pose.theta = wrap_angle(heading_ccw + theta_offset);
    }

    if (flow_ready) {
        int16_t delta_x_counts = 0;
        int16_t delta_y_counts = 0;
        flow_sensor->readMotionCount(&delta_x_counts, &delta_y_counts);

        float dx_body = delta_x_counts * FLOW_METERS_PER_COUNT;
        float dy_body = delta_y_counts * FLOW_METERS_PER_COUNT;

        float c =cosf(current_pose.theta);
        float s = sinf(current_pose.theta);

        current_pose.x += dx_body * c - dy_body * s;
        current_pose.y += dx_body * s + dy_body * c;
    }

    return current_pose;
}

Pose pose_get() {
    return current_pose;
}