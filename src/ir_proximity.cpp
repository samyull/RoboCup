#include "ir_proximity.h"
#include <Arduino.h>

void ir_proximity_init() {
  // Going through a powered level shifter, not directly into the sensor's
  // open-collector output, so the shifter should drive the line both ways.
  // Plain INPUT to start; switch to INPUT_PULLDOWN only if raw readings
  // float when idle (same symptom as the induction sensor's early testing).
  pinMode(IR_PROXIMITY_PIN, INPUT);
}

bool ir_proximity_detected() {
  // Confirmed active-LOW: 0 = in range, 1 = out of range.
  return digitalRead(IR_PROXIMITY_PIN) == LOW;
}