#ifndef IR_PROXIMITY_H
#define IR_PROXIMITY_H

#include <stdint.h>

// E18-D80NK -> Digital Level Shift channel A -> Con54 pin 5 (D30Z) on the Teensy.
#define IR_PROXIMITY_PIN 30

void ir_proximity_init();

// Returns true if an object is currently detected.
bool ir_proximity_detected();

#endif