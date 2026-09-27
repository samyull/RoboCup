#ifndef INDUCTION_H
#define INDUCTION_H

#define INDUCTION_PIN 20
#define INDUCTION_THRESHOLD 500

void induction_init();
bool induction_detected();
int  induction_read();

#endif