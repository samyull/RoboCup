#include "weight_collection.h"
#include "arms.h"
#include "electromagnets.h"
#include "induction.h"
#include "ir_proximity.h"
#include <Arduino.h>

// --- Tuning ---
static const int      CRANE_IDLE_ANGLE     = 50;
static const float    CRANE_IDLE_SPEED     = 100.0f;  // deg/s
static const uint32_t INDUCTION_TIMEOUT_MS = 5000;
static const int      PICKUPS_BEFORE_RELEASE = 3;

// TODO: confirm rest/active angles on the robot (taken from the old arm test).
static const int      CLEARING_ARM_REST    = 180;
static const int      CLEARING_ARM_ACTIVE  = 70;
static const int      RELEASE_ARM_REST     = 0;
static const int      RELEASE_ARM_ACTIVE   = 120;
static const uint32_t ARM_HOLD_MS          = 1000;  // time held at the active angle
static const uint32_t ARM_RETURN_MS        = 700;   // time allowed to swing back to rest

// --- Pickup sequence (same moves as crane_test(), starting from idle) ---
enum MagnetAction { MAG_NONE, MAG_ON, MAG_OFF };

struct CraneStep {
  int angle;
  float speed;          // deg/s
  uint32_t dwell_ms;    // wait after arriving
  MagnetAction action;  // done after the dwell
};

static const CraneStep PICKUP_STEPS[] = {
  {63,  15, 1000, MAG_ON },   // lower slowly onto the weight, magnets on
  {63,  15, 1000, MAG_NONE},  // let the magnets grab
  {37,  20,  500, MAG_NONE},  // lift
  {42,  15, 2000, MAG_OFF },  // settle, drop the weight
  {63, 100, 1000, MAG_NONE},
  {CRANE_IDLE_ANGLE, CRANE_IDLE_SPEED, 0, MAG_NONE},  // back to idle
};
static const int PICKUP_STEP_COUNT = sizeof(PICKUP_STEPS) / sizeof(PICKUP_STEPS[0]);

// --- State ---
static WeightState state = WC_SCANNING;
static uint32_t state_start_ms = 0;
static int pickup_count = 0;

static int step_index = 0;
static bool step_arrived = false;
static uint32_t step_arrived_ms = 0;

static bool arm_returning = false;

static const char *state_name(WeightState s) {
  switch (s) {
    case WC_SCANNING:              return "SCANNING";
    case WC_WAITING_FOR_INDUCTION: return "WAITING_FOR_INDUCTION";
    case WC_PICKUP:                return "PICKUP";
    case WC_CLEARING:              return "CLEARING";
    case WC_RELEASING:             return "RELEASING";
  }
  return "?";
}

static void set_state(WeightState s) {
  state = s;
  state_start_ms = millis();
  Serial.print("Weight collection: ");
  Serial.println(state_name(s));
}

// Release/clearing arms may only move with the magnets off and the crane parked at idle.
static bool arms_safe_to_move() {
  return !electromagnets_are_on() && state != WC_PICKUP && crane_at_target();
}

static void start_step(int i) {
  step_index = i;
  step_arrived = false;
  move_crane_to(PICKUP_STEPS[i].angle, PICKUP_STEPS[i].speed);
}

static bool release_requested = false;

void weight_collection_request_release() {
  release_requested = true;
}


static void start_pickup() {
  set_state(WC_PICKUP);
  start_step(0);
}

static void update_pickup() {
  const CraneStep &step = PICKUP_STEPS[step_index];

  if (!step_arrived) {
    if (!crane_at_target()) return;
    step_arrived = true;
    step_arrived_ms = millis();
  }
  if (millis() - step_arrived_ms < step.dwell_ms) return;

  if (step.action == MAG_ON)  { electromagnets_on();  Serial.println("Electromagnets ON"); }
  if (step.action == MAG_OFF) { electromagnets_off(); Serial.println("Electromagnets OFF"); }

  if (step_index + 1 < PICKUP_STEP_COUNT) {
    start_step(step_index + 1);
    return;
  }

  // Sequence finished, crane back at idle.
  pickup_count++;
  Serial.print("Pickup complete, count = ");
  Serial.println(pickup_count);
  set_state(WC_SCANNING);
}

// Tries to start an arm swing; stays in SCANNING if the interlock blocks it.
static bool start_arm(WeightState arm_state) {
  if (!arms_safe_to_move()) return false;
  arm_returning = false;
  set_state(arm_state);
  if (arm_state == WC_CLEARING) set_clearing_arm_angle(CLEARING_ARM_ACTIVE);
  else                          set_release_arm_angle(RELEASE_ARM_ACTIVE);
  return true;
}

static void update_arm() {
  uint32_t elapsed = millis() - state_start_ms;

  if (!arm_returning) {
    if (elapsed < ARM_HOLD_MS) return;
    if (state == WC_CLEARING) set_clearing_arm_angle(CLEARING_ARM_REST);
    else                      set_release_arm_angle(RELEASE_ARM_REST);
    arm_returning = true;
    return;
  }
  if (elapsed < ARM_HOLD_MS + ARM_RETURN_MS) return;

  if (state == WC_RELEASING) pickup_count = 0;
  set_state(WC_SCANNING);
}

void weight_collection_init() {
  electromagnets_off();
  set_clearing_arm_angle(CLEARING_ARM_REST);
  set_release_arm_angle(RELEASE_ARM_REST);
  move_crane_to(CRANE_IDLE_ANGLE, CRANE_IDLE_SPEED);  // jumps on first move
  pickup_count = 0;
  set_state(WC_SCANNING);
}

void weight_collection_update() {
  crane_update();

  bool proximity = ir_proximity_detected();
  bool induction = induction_detected();

  switch (state) {
    case WC_SCANNING:
      if (pickup_count >= PICKUPS_BEFORE_RELEASE) {
        if (release_requested) {
          start_arm(WC_RELEASING);
          release_requested = false;
        }
      } else if (proximity && induction) {
        start_pickup();
      } else if (proximity) {
        set_state(WC_WAITING_FOR_INDUCTION);
      }
      break;

    case WC_WAITING_FOR_INDUCTION:
      // Proximity already saw the object, so keep waiting even if it drops out.
      if (induction) {
        start_pickup();
      } else if (millis() - state_start_ms >= INDUCTION_TIMEOUT_MS) {
        if (!start_arm(WC_CLEARING)) set_state(WC_SCANNING);
      }
      break;

    case WC_PICKUP:
      update_pickup();
      break;

    case WC_CLEARING:
    case WC_RELEASING:
      update_arm();
      break;
  }
}

bool weight_collection_busy() {
  return state != WC_SCANNING;
}

WeightState weight_collection_state() {
  return state;
}

int weight_collection_count() {
  return pickup_count;
}