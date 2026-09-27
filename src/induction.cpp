#include "induction.h"
#include <Arduino.h>

void induction_init() {
  pinMode(INDUCTION_PIN, INPUT);
}

int induction_read() {
  return analogRead(INDUCTION_PIN);
}

bool induction_detected() {
  return induction_read() < INDUCTION_THRESHOLD;
}
