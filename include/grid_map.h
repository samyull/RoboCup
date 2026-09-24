#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <Arduino.h>
#include "pose.h"

// Grid geometry. 100 x 100 cells at 5 cm = 5 m x 5 m, using ~10 KB of RAM.
// The world origin (where the robot boots) sits in the middle of the grid.
#define MAP_CELL_SIZE_M     0.05f
#define MAP_MARGIN_CELLS    6        // padding around the arena, in cells

#define MAP_GRID_SIZE_X_M   4.0f     // long side - TODO: confirm before match
#define MAP_GRID_SIZE_Y_M   2.0f     // short side - TODO: confirm before match

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

/** Fill the grid with MAP_CELL_UNKNOWN. Call once from setup(). */
void map_init();

/**
 * Mark the cell the robot is in as MAP_CELL_ROBOT, and the cell it just left
 * as MAP_CELL_FREE. Call every loop with the latest pose. Prints
 * "C,gx,gy,value" over Serial for each cell that changes.
 */
void map_update(const Pose &pose);

/**
 * Convert world coordinates (metres) to grid indices.
 * @return true if the point lies inside the grid.
 */
bool world_to_grid(float x_m, float y_m, int &gx, int &gy);

void grid_to_world(int gx, int gy, float &x_m, float &y_m);

/** @return the cell value, or MAP_CELL_UNKNOWN if (gx, gy) is off the grid. */
int8_t map_get_cell(int gx, int gy);

/** Write a cell value (ignored if off the grid) and log it over Serial. */
void map_set_cell(int gx, int gy, int8_t value);

// Traces a line of free spaces in the grid from robot's pose to a given TOF reading
void map_ray_trace(float x0_m, float y0_m, float x1_m, float y1_m);

#endif // GRID_MAP_H