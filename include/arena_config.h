#ifndef ARENA_CONFIG_H
#define ARENA_CONFIG_H

#include "color_sensor.h"

// ---------------------------------------------------------------------------
// STARTING LAYOUT - set before every round.
// Leave exactly ONE block below uncommented.
//
// Arena frame: x along the long wall (0 to 4.9 m), y along the short wall
// (0 to 2.4 m), heading measured anticlockwise from +x. Both start corners are
// at the x = 0 short end. Standing at that end looking down the arena, the
// BLUE corner is on the right (the origin) and the GREEN corner on the left.
//
// X/Y: where the robot's centre of rotation sits at the button press - measure
// on the day. Heading: the direction the robot faces at the press.
// ---------------------------------------------------------------------------

// BLUE corner, facing along the long wall
// #define ROBOT_START_X_M         0.3f
// #define ROBOT_START_Y_M         0.3f
// #define ROBOT_START_HEADING_DEG 0.0f
// #define ROBOT_HOME_COLOR        COLOR_BLUE

// BLUE corner, facing along the short wall (towards green)
// #define ROBOT_START_X_M         0.3f
// #define ROBOT_START_Y_M         0.3f
// #define ROBOT_START_HEADING_DEG 90.0f
// #define ROBOT_HOME_COLOR        COLOR_BLUE

// GREEN corner, facing along the long wall
#define ROBOT_START_X_M         0.3f
#define ROBOT_START_Y_M         2.1f
#define ROBOT_START_HEADING_DEG 0.0f
#define ROBOT_HOME_COLOR        COLOR_GREEN

// GREEN corner, facing along the short wall (towards blue)
// #define ROBOT_START_X_M         0.3f
// #define ROBOT_START_Y_M         2.1f
// #define ROBOT_START_HEADING_DEG -90.0f
// #define ROBOT_HOME_COLOR        COLOR_GREEN

#endif
