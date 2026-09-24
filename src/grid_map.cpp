#include "grid_map.h"

// Set to false to silence the "C,gx,gy,value" cell log.
static const bool MAP_LOG_CELLS = true;

static int8_t grid[MAP_GRID_H][MAP_GRID_W];
static int last_gx = -1;
static int last_gy = -1;

void map_init() {
  for (int y = 0; y < MAP_GRID_H; y++) {
    for (int x = 0; x < MAP_GRID_W; x++) {
      grid[y][x] = MAP_CELL_UNKNOWN;
    }
  }
  last_gx = -1;
  last_gy = -1;
}

bool world_to_grid(float x_m, float y_m, int &gx, int &gy) {
  gx = (int)roundf(x_m / MAP_CELL_SIZE_M) + MAP_X_ZERO;
  gy = (int)roundf(y_m / MAP_CELL_SIZE_M) + MAP_Y_ZERO;
  return gx >= 0 && gx < MAP_GRID_W && gy >= 0 && gy < MAP_GRID_H;
}

void grid_to_world(int gx, int gy, float &x_m, float &y_m) {
  x_m = (float)(gx - MAP_X_ZERO)*MAP_CELL_SIZE_M;
  y_m = (float)(gy - MAP_Y_ZERO)*MAP_CELL_SIZE_M;
}

int8_t map_get_cell(int gx, int gy) {
  if (gx < 0 || gx >= MAP_GRID_W || gy < 0 || gy >= MAP_GRID_H) return MAP_CELL_UNKNOWN;
  return grid[gy][gx];
}

void map_set_cell(int gx, int gy, int8_t value) {
  if (gx < 0 || gx >= MAP_GRID_W || gy < 0 || gy >= MAP_GRID_H) return;
  grid[gy][gx] = value;

  if (MAP_LOG_CELLS) {
    Serial.print("C,"); Serial.print(gx);
    Serial.print(","); Serial.print(gy);
    Serial.print(","); Serial.println(value);
  }
}

void map_ray_trace(float x0_m, float y0_m, float x1_m, float y1_m) {
  // Function traces a line of free space up to whatever the TOF reading is seeing
  int x0, y0, x1, y1;
  world_to_grid(x0_m, y0_m, x0, y0);
  world_to_grid(x1_m, y1_m, x1, y1);

  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0, y = y0;
 
  while (!(x == x1 && y == y1)) {
    int e2 = 2 * err;
    if(e2 >= dy) {
      err += dy;
      x += sx;
    }
    if(e2 <= dx) {
      err += dx;
      y += sy;
    }
    if (x == x1 && y == y1) break;   // Stops before setting the actual end point
    map_set_cell(x, y, MAP_CELL_FREE);
  }
}

void map_update(const Pose &pose) {
  int gx, gy;
  if (!world_to_grid(pose.x, pose.y, gx, gy)) return;   // robot has left the mapped area
  if (gx == last_gx && gy == last_gy) return;           // still in the same cell

  // The cell we just left is known free (we drove through it).
  if (last_gx >= 0) map_set_cell(last_gx, last_gy, MAP_CELL_FREE);
  map_set_cell(gx, gy, MAP_CELL_ROBOT);

  last_gx = gx;
  last_gy = gy;
}