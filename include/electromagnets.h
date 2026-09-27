#ifndef ELECTROMAGNETS_H
#define ELECTROMAGNETS_H

#define ELECTROMAGNET_PIN A12  // Con74, pin 26

void electromagnets_init();
void electromagnets_on();
void electromagnets_off();
bool electromagnets_are_on();

// On for 2s, off for 2s (blocking, for testing).
void electromagnets_test();

#endif