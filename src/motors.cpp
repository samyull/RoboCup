#include "motors.h"
#include "Arduino.h"
#include "Servo.h"

Servo right_motor;
Servo left_motor;
void motors_init() {
  left_motor.attach(LEFT_MOTOR_ADDRESS);
  right_motor.attach(RIGHT_MOTOR_ADDRESS);
  Serial.println("Motors have been initialised \n");
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



