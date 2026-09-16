#ifndef POSE_H
#define POSE_H

#include <Arduino.h>

struct Pose {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f; // in degrees
};

/*
* Initialises the BNO055 IMU and the PMW3901 optical flow sensor. 
* Call once from setup(), after Serial.begin(). Halts with a Serial
* message if either sensor fails to initialise, since dead reckoning
* is impossible without both of them.
*
* @param flow_chip_select The Teensy pin number connected to the PMW3901's CS pin.
*/
void pose_init(uint8_t flow_chip_select);

/*
* Read both sensors and integrate the pose estimate forward by one step.
* Call this every loop() iteration, as close to a fixed rate as practical - 
* the optical flow scale factor below assumes a roughly constant loop time.
*
* @return the updated Pose (also availale via pose_get())
*/
Pose pose_update();

/*
* @return the most recent pose value without taking a new reading from the sensors.
*/
Pose pose_get();

/*
* Reset x, y, theta back to zero. Useful for re-anchoring when the robot is at a known reference point.
*/
void pose_reset();

/*
* Scale factor converting raw PMW3901 motion counts to metres travelled.
* The PMW3901 reports counts proportional to angular motion of the ground
* pattern under the sensor, so the real-world distance per count depends
* on the height of the sensor above the ground.
*/
extern float FLOW_METERS_PER_COUNT;

#endif // POSE_H