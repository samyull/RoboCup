#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <Arduino.h>
#include "pose.h"

// 5 cm confidence cells, with origin at the arena corner.
#define MAP_CELL_SIZE_M     0.05f
#define MAP_MARGIN_CELLS    1        // padding around the arena, in cells

#define MAP_GRID_SIZE_X_M   4.9f     // long side - TODO: confirm before match
#define MAP_GRID_SIZE_Y_M   2.4f     // short side - TODO: confirm before match

#define MAP_GRID_W ((int)(MAP_GRID_SIZE_X_M / MAP_CELL_SIZE_M + 0.5f) + 2 * MAP_MARGIN_CELLS)
#define MAP_GRID_H ((int)(MAP_GRID_SIZE_Y_M / MAP_CELL_SIZE_M + 0.5f) + 2 * MAP_MARGIN_CELLS)

#define MAP_X_ZERO MAP_MARGIN_CELLS
#define MAP_Y_ZERO MAP_MARGIN_CELLS

enum MapCell : int8_t {
  MAP_CELL_BORDER   = -2,
  MAP_CELL_UNKNOWN  = -1,
  MAP_CELL_FREE     = 0,
  MAP_CELL_OBSTACLE = 1,
  MAP_CELL_ROBOT    = 2,
  MAP_CELL_WEIGHT   = 3
};

void map_init();
void map_mark_boundaries();
// Print a Q, line for every cell that differs from blank free space.
void map_publish_all();
// Call every loop: sends changed cells (one Q, line each, latest value only),
// a sweep at most every 200 ms and a bounded number of lines per call.
void map_publish_changes();
bool world_to_grid(float x_m, float y_m, int &gx, int &gy);
void grid_to_world(int gx, int gy, float &x_m, float &y_m);
int8_t map_get_cell(int gx, int gy);
// Raw score, -100 (wall) to +100 (weight); MAP_BOUNDARY_SCORE outside/on the border.
int map_get_confidence(int gx, int gy);
// Scores are independent of MapCell labels returned by map_get_cell().
static const int MAP_BOUNDARY_SCORE = -101;
static const int MAP_ENTER_SCORE = 20;
static const int MAP_EXIT_SCORE = 10;
static const int MAP_HIT_EVIDENCE = 10;
static const int MAP_FREE_EVIDENCE = 4;
// Time for a full-confidence cell (+/-100) to decay below MAP_EXIT_SCORE and
// revert to free. Weights get moved around by other robots; walls do not.
static const uint32_t MAP_WEIGHT_DECAY_MS = 20000;
static const uint32_t MAP_WALL_DECAY_MS = 90000;
void map_observe_weight(int gx, int gy);
void map_observe_wall(int gx, int gy);
// protect_weights: never lowers positive (weight) evidence, only wall evidence.
void map_observe_free(int gx, int gy, bool protect_weights = false);
void map_reject_weight(int gx, int gy);
void map_decay();
void map_ray_trace(float x0_m, float y0_m, float x1_m, float y1_m, bool upper_beam = false);

#endif // GRID_MAP_H
