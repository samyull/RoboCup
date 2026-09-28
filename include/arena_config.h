// arena_config.h - new small header
#ifndef ARENA_CONFIG_H
#define ARENA_CONFIG_H

// Measured physically: how far the robot's centre of rotation sits from
// the two walls forming its starting corner, along whichever directions
// become +x (forward) and +y (left) at boot. Re-anchors pose's local
// (0,0) to the arena's TRUE corner, not wherever the robot is parked
// within the 600x600mm base.
#define ROBOT_START_X_M 0.3f   // TODO: measure on the day
#define ROBOT_START_Y_M 0.3f   // TODO: measure on the day

#endif