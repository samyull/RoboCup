#ifndef WEIGHT_COLLECTION_H
#define WEIGHT_COLLECTION_H

// Non-blocking weight pickup / clearing / release logic.
// Call weight_collection_init() once in setup(), then weight_collection_update()
// every loop. Nothing here uses delay(), so the main loop keeps running fast.
//
// Behaviour:
//  - IR proximity + induction both detect  -> run the crane pickup sequence.
//  - IR proximity only                     -> wait up to 5s for induction;
//                                             if it never detects, swing the clearing arm.
//  - After 3 pickups, once the crane is back at idle -> swing the release arm.
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

// True whenever the robot should hold still (waiting, picking up, or moving an arm).
bool weight_collection_busy();

WeightState weight_collection_state();
int weight_collection_count();   // pickups since the last release

void weight_collection_request_release();

#endif