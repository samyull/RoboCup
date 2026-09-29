//************************************
//         motors.h    
//************************************

#ifndef MOTORS_H_
#define MOTORS_H_

#include <stdint.h>

// SET THIS TO REAL VALUES
#define LEFT_MOTOR_ADDRESS 1     //Pin corresponding to the left dc motor
#define RIGHT_MOTOR_ADDRESS 0    //Pin corresponding to the right dc motor

#define MOTOR_US_MIN 1050 // Max speed reverse
#define MOTOR_US_MAX 1950 // Max speed forward
#define MOTOR_US_STOP 1500 // Stop speed

void motors_init();
void set_motors(int left_speed, int right_speed);
void stop_motors();

// Milliseconds since both motors were commanded to 0 (at least 1), or 0 while
// either is commanded to move. Lets the pose ignore flow while stopped.
uint32_t motors_stopped_ms();

#endif /* MOTORS_H_*/
