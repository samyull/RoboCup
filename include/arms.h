#ifndef ARMS_H
#define ARMS_H

#include <stdint.h>

// HZ12K positional servos
#define RELEASE_ARM_PIN  A1   // Con69
#define CLEARING_ARM_PIN A11  // Con73

// RDS5160 60kg positional servo (needs 6-8.4V supply)
#define CRANE_PIN        A13  // Con75

void arms_init();

// Positional servos, 0-180 degrees.
void set_release_arm_angle(int degrees);
void set_clearing_arm_angle(int degrees);

// Slow crane moves at deg_per_sec. Non-blocking: call move_crane_to() once,
// then crane_update() every loop until crane_at_target().
void move_crane_to(int degrees, float deg_per_sec);
void crane_update();
bool crane_at_target();

#endif