#include "motors.h"
#include "Arduino.h"
#include "Servo.h"

Servo right_motor;
Servo left_motor;
void motors_init() {
  left_motor.attach(LEFT_MOTOR_ADDRESS);
  right_motor.attach(RIGHT_MOTOR_ADDRESS);
  stop_motors();
  Serial.println("Motors have been initialised \n");
}

static int clamp_speed(int speed_pct) {
  if (speed_pct > 100) return 100;
  if (speed_pct < -100) return -100;
  return speed_pct;
}

static int speed_to_us(int speed_pct) {
  speed_pct = clamp_speed(speed_pct);
  return MOTOR_US_STOP + (speed_pct * (MOTOR_US_MAX - MOTOR_US_STOP) / 100);
}

void set_motors(int left_speed, int right_speed) {
  int left_us = speed_to_us(left_speed);
  int right_us = speed_to_us(right_speed);

  left_motor.writeMicroseconds(left_us);
  right_motor.writeMicroseconds(right_us);
}

void stop_motors() {
  set_motors(0, 0);
}



/* Check whether the speed value to be written is within the maximum
 *  and minimum speed caps. Act accordingly.
 *
 */ 
// void check_speed_limits(/*parameters*/) {
//   Serial.println("Check the motor speed limit \n");
// }


/* In this section, the motor speeds should be updated/written.
 *It is also a good idea to check whether value to write is valid.
 *It is also a good idea to do so atomically!
 */
void set_motor(/*parameters*/) {

  Serial.println("Change the motor speed \n");
  left_motor.writeMicroseconds(1950);
  right_motor.writeMicroseconds(1950);
  // check_speed_limits();

}



