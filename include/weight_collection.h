#ifndef WEIGHT_COLLECTION_H
#define WEIGHT_COLLECTION_H

// Non-blocking weight pickup / clearing / release logic.
// Call weight_collection_init() once in setup(), then weight_collection_update()
// every loop. Nothing here uses delay(), so the main loop keeps running fast.
//
// Behaviour:
//  - Induction detects (regardless of IR)  -> run the crane pickup sequence.
//  - IR proximity only                     -> feed forward up to 5s for induction;
//                                             if it never detects, swing the clearing arm.
//  - After 3 pickups, release when requested at home with the crane at idle.
//  - Release/clearing arms never move while the magnets are on or the crane is busy.
//  - Crane sits at idle (50 deg) when not picking up.

enum WeightState {
  WC_SCANNING,             // idle, watching the sensors
  WC_WAITING_FOR_INDUCTION,// proximity only, giving induction time to trigger
  WC_PICKUP,               // crane sequence running
  WC_CLEARING,             // clearing arm swinging
  WC_RELEASING             // release arm swinging
};

void weight_collection_init();
void weight_collection_update();

// True whenever collection owns driving (feeding, picking up, or moving an arm).
// Hold still unless weight_collection_feeding() is true.
bool weight_collection_busy();

// Drive forward while waiting for induction, until the timeout expires.
bool weight_collection_feeding();

WeightState weight_collection_state();
int weight_collection_count();   // pickups since the last release

void weight_collection_request_release();

#endif
