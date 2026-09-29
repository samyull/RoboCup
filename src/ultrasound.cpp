#include "ultrasound.h"
#include <Arduino.h>

// --- Tuning ---
// Time allowed per ping before moving to the other sensor. The HC-SR04 holds
// echo high for ~38 ms when nothing comes back, so this also lets old echoes die out.
static const uint32_t PING_PERIOD_MS  = 40;
static const uint32_t MAX_ECHO_US     = 23300;  // ~4 m; longer = nothing in range
static const uint32_t STALE_MS        = 250;    // drop readings older than this

static const uint8_t trigPins[US_COUNT] = {US_LEFT_TRIG_PIN, US_RIGHT_TRIG_PIN};
static const uint8_t echoPins[US_COUNT] = {US_LEFT_ECHO_PIN, US_RIGHT_ECHO_PIN};
static const char *const usNames[US_COUNT] = {"USL", "USR"};

// Written by the echo interrupts.
static volatile uint32_t rise_us[US_COUNT];
static volatile uint32_t pulse_us[US_COUNT];
static volatile bool rising_seen[US_COUNT];
static volatile bool echo_done[US_COUNT];

static uint16_t last_mm[US_COUNT];
static uint32_t last_ok_ms[US_COUNT];
static bool has_reading[US_COUNT];

// Close-object interrupt state (see ultrasound.h).
static uint8_t below_count[US_COUNT];
static uint8_t above_count[US_COUNT];
static bool interrupting[US_COUNT];
static UltrasoundInterruptHandler handlers[US_COUNT] = {nullptr, nullptr};

static uint8_t active = 0;          // sensor currently pinging
static uint32_t ping_start_ms = 0;

static void handle_echo(uint8_t i) {
  const uint32_t now = micros();
  if (digitalReadFast(echoPins[i])) {
    rise_us[i] = now;
    rising_seen[i] = true;
  } else if (rising_seen[i]) {
    pulse_us[i] = now - rise_us[i];
    rising_seen[i] = false;
    echo_done[i] = true;
  }
}

static void echo_isr_left()  { handle_echo(US_LEFT); }
static void echo_isr_right() { handle_echo(US_RIGHT); }

static void trigger(uint8_t i) {
  noInterrupts();
  rising_seen[i] = false;
  echo_done[i] = false;
  interrupts();

  // 10 us HIGH pulse, with a short LOW beforehand for a clean edge.
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);
  ping_start_ms = millis();
}

static void clear_interrupt(uint8_t i) {
  below_count[i] = above_count[i] = 0;
  interrupting[i] = false;
}

// Called only with valid readings; invalid ones never reach here.
static void update_interrupt(uint8_t i, uint16_t mm) {
  if (mm < ULTRASOUND_INTERRUPT_MM) {
    above_count[i] = 0;
    if (below_count[i] < ULTRASOUND_TRIGGER_COUNT) below_count[i]++;
    if (!interrupting[i] && below_count[i] >= ULTRASOUND_TRIGGER_COUNT) {
      interrupting[i] = true;
      Serial.print(usNames[i]); Serial.println(" INTERRUPT");
      if (handlers[i]) handlers[i]();
    }
  } else {
    below_count[i] = 0;
    if (above_count[i] < ULTRASOUND_CLEAR_COUNT) above_count[i]++;
    if (interrupting[i] && above_count[i] >= ULTRASOUND_CLEAR_COUNT) {
      interrupting[i] = false;
      Serial.print(usNames[i]); Serial.println(" interrupt cleared");
    }
  }
}

// Store the result of the ping that just finished (or mark it as no echo).
static void collect(uint8_t i) {
  noInterrupts();
  const bool done = echo_done[i];
  const uint32_t us = pulse_us[i];
  interrupts();

  // Speed of sound ~343 m/s, halved for the round trip: mm = us / 5.83
  const uint16_t mm = (uint16_t)(us * 10 / 58);
  if (done && mm > 0 && us <= MAX_ECHO_US) {
    last_mm[i] = mm;
    last_ok_ms[i] = millis();
    has_reading[i] = true;
    update_interrupt(i, mm);
  } else {
    has_reading[i] = false;  // invalid: ignored by the interrupt logic
  }
}

void ultrasound_init() {
  for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW);
    pinMode(echoPins[i], INPUT);
    has_reading[i] = false;
    clear_interrupt(i);
  }
  attachInterrupt(digitalPinToInterrupt(US_LEFT_ECHO_PIN), echo_isr_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_RIGHT_ECHO_PIN), echo_isr_right, CHANGE);

  active = 0;
  trigger(active);
  Serial.println("Ultrasound initialised");
}

void ultrasound_update() {
  // A sensor that has stopped giving valid readings can't keep interrupting.
  for (uint8_t i = 0; i < US_COUNT; i++) {
    if ((below_count[i] || interrupting[i]) && millis() - last_ok_ms[i] > STALE_MS) {
      if (interrupting[i]) { Serial.print(usNames[i]); Serial.println(" interrupt cleared (no readings)"); }
      clear_interrupt(i);
    }
  }

  if (millis() - ping_start_ms < PING_PERIOD_MS) return;
  collect(active);
  active = (active + 1) % US_COUNT;
  trigger(active);
}

bool ultrasound_read_mm(uint8_t index, uint16_t &mm) {
  if (index >= US_COUNT || !has_reading[index]) return false;
  if (millis() - last_ok_ms[index] > STALE_MS) return false;
  mm = last_mm[index];
  return true;
}

const char *ultrasound_name(uint8_t index) {
  return index < US_COUNT ? usNames[index] : "?";
}

bool ultrasound_interrupt_left()  { return interrupting[US_LEFT]; }
bool ultrasound_interrupt_right() { return interrupting[US_RIGHT]; }

void ultrasound_attach_interrupt_left(UltrasoundInterruptHandler handler)  { handlers[US_LEFT] = handler; }
void ultrasound_attach_interrupt_right(UltrasoundInterruptHandler handler) { handlers[US_RIGHT] = handler; }
