#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <Arduino.h>
#include "pose.h"

// Grid geometry. 100 x 100 cells at 5 cm = 5 m x 5 m, using ~10 KB of RAM.
// The world origin (where the robot boots) sits in the middle of the grid.
#define MAP_CELL_SIZE_M     0.05f
#define MAP_MARGIN_CELLS    6        // padding around the arena, in cells

#define MAP_GRID_SIZE_X_M   4.9f     // long side - TODO: confirm before match
#define MAP_GRID_SIZE_Y_M   2.4f     // short side - TODO: confirm before match

#define MAP_GRID_W ((int)(MAP_GRID_SIZE_X_M / MAP_CELL_SIZE_M) + 2 * MAP_MARGIN_CELLS)
#define MAP_GRID_H ((int)(MAP_GRID_SIZE_Y_M / MAP_CELL_SIZE_M) + 2 * MAP_MARGIN_CELLS)

#define MAP_X_ZERO MAP_MARGIN_CELLS
#define MAP_Y_ZERO MAP_MARGIN_CELLS

enum MapCell : int8_t {
  MAP_CELL_UNKNOWN  = -1,
  MAP_CELL_FREE     = 0,
  MAP_CELL_OBSTACLE = 1,
  MAP_CELL_ROBOT    = 2,
  MAP_CELL_WEIGHT   = 3
};

void map_init();
void map_update(const Pose &pose);
bool world_to_grid(float x_m, float y_m, int &gx, int &gy);
void grid_to_world(int gx, int gy, float &x_m, float &y_m);
int8_t map_get_cell(int gx, int gy);
void map_set_cell(int gx, int gy, int8_t value);
void map_ray_trace(float x0_m, float y0_m, float x1_m, float y1_m);

#endif // GRID_MAP_H