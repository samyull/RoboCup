#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include <stdint.h>

// Two HC-SR04 style ultrasound sensors on the ultrasound board, wired to the
// Teensy through Con55. Pins follow the 110_UltrasoundDigital example.
// TODO: confirm these against Con55 on the robot.
#define US_LEFT_TRIG_PIN  3   // 'A', Con3 on the ultrasound board
#define US_LEFT_ECHO_PIN  2
#define US_RIGHT_TRIG_PIN 5   // 'B', Con4 on the ultrasound board
#define US_RIGHT_ECHO_PIN 4

enum UltrasoundIndex : uint8_t {
  US_LEFT  = 0,
  US_RIGHT = 1,
  US_COUNT = 2
};

void ultrasound_init();

// Non-blocking (unlike pulseIn()): echoes are timed by pin interrupts. Call every
// loop; it pings the sensors one at a time so they don't hear each other's echoes.
void ultrasound_update();

// Latest distance in mm. Returns false if there is no recent echo
// (nothing in range, sensor unplugged, or the reading has gone stale).
bool ultrasound_read_mm(uint8_t index, uint16_t &mm);

const char *ultrasound_name(uint8_t index);

// ---------------------------------------------------------------------------
// Close-object interrupts
// ---------------------------------------------------------------------------
// A side "interrupts" once it gets ULTRASOUND_TRIGGER_COUNT valid readings in a
// row below ULTRASOUND_INTERRUPT_MM. No echo / out of range / zero readings are
// ignored (they neither count towards triggering nor clear it). It clears after
// ULTRASOUND_CLEAR_COUNT valid readings in a row at or above the threshold, or
// if the sensor stops giving valid readings.
#define ULTRASOUND_INTERRUPT_MM  200
#define ULTRASOUND_TRIGGER_COUNT 3
#define ULTRASOUND_CLEAR_COUNT   3

// Option 1 - poll: true while that side is interrupting.
bool ultrasound_interrupt_left();
bool ultrasound_interrupt_right();

// Option 2 - callback: attach a function that is called ONCE each time that
// side starts interrupting. It runs from ultrasound_update() in the main loop
// (not a hardware ISR), so it is safe to call motor/serial code from it.
// Pass nullptr to detach.
typedef void (*UltrasoundInterruptHandler)();
void ultrasound_attach_interrupt_left(UltrasoundInterruptHandler handler);
void ultrasound_attach_interrupt_right(UltrasoundInterruptHandler handler);

#endif
