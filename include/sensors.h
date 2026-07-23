//************************************
//         sensors.h     
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

#include <Arduino.h>

void sensors_init();

// Returns current heading in degrees
float getHeading();

// Reads optical flow displacement since last call.
void read_opticalFlow(float &dy, float &dx);

#endif /* SENSORS_H_ */
