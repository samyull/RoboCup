//************************************
//         motors.h    
//************************************

#ifndef MOTORS_H_
#define MOTORS_H_

// SET THIS TO REAL VALUES
#define LEFT_MOTOR_ADDRESS 1     //Pin corresponding to the left dc motor
#define RIGHT_MOTOR_ADDRESS 0    //Pin corresponding to the right dc motor

#define MOTOR_US_MIN 1050 // Max speed reverse
#define MOTOR_US_MAX 1950 // Max speed forward
#define MOTOR_US_STOP 1500 // Stop speed

void motors_init();
void set_motors(int left_speed, int right_speed);
void stop_motors();

#endif /* MOTORS_H_*/
