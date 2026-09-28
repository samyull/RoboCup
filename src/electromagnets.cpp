#include "electromagnets.h"
#include <Arduino.h>

static bool magnets_on = false;

void electromagnets_init() {
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  electromagnets_off();   // make sure they're off on boot
  Serial.println("Electromagnets initialised");
}

void electromagnets_on() {
  digitalWrite(ELECTROMAGNET_PIN, HIGH);
  magnets_on = true;
}

void electromagnets_off() {
  digitalWrite(ELECTROMAGNET_PIN, LOW);
  magnets_on = false;
}

bool electromagnets_are_on() {
  return magnets_on;
}