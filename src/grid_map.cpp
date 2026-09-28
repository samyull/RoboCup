#include "grid_map.h"

static int8_t confidence[MAP_GRID_H][MAP_GRID_W];
static int8_t labels[MAP_GRID_H][MAP_GRID_W];
static uint32_t last_decay_ms = 0;

static bool inside(int x, int y) {
  return x >= 0 && x < MAP_GRID_W && y >= 0 && y < MAP_GRID_H;
}

static void publish(int x, int y) {
  // Include the firmware label: the visualiser must not guess hysteresis state.
  Serial.print("Q,"); Serial.print(x); Serial.print(","); Serial.print(y);
  Serial.print(","); Serial.print(confidence[y][x]);
  Serial.print(","); Serial.println(labels[y][x]);
}

static void update_score(int x, int y, int score) {
  if (!inside(x, y) || confidence[y][x] == MAP_BOUNDARY_SCORE) return;
  score = constrain(score, -100, 100);
  int8_t label = labels[y][x];
  if (score >= MAP_ENTER_SCORE) label = MAP_CELL_WEIGHT;
  else if (score <= -MAP_ENTER_SCORE) label = MAP_CELL_OBSTACLE;
  else if (label == MAP_CELL_WEIGHT && score >= MAP_EXIT_SCORE) {}
  else if (label == MAP_CELL_OBSTACLE && score <= -MAP_EXIT_SCORE) {}
  else label = MAP_CELL_FREE;
  if (confidence[y][x] == score && labels[y][x] == label) return;
  confidence[y][x] = score;
  labels[y][x] = label;
  publish(x, y);
}

void map_mark_boundaries() {
  for (int y = 0; y < MAP_GRID_H; ++y) {
    for (int x = 0; x < MAP_GRID_W; ++x) {
      if (x != 0 && y != 0 && x != MAP_GRID_W - 1 && y != MAP_GRID_H - 1) continue;
      confidence[y][x] = MAP_BOUNDARY_SCORE;
      labels[y][x] = MAP_CELL_BORDER;
      publish(x, y);
    }
  }
}

void map_init() {
  for (int y = 0; y < MAP_GRID_H; ++y)
    for (int x = 0; x < MAP_GRID_W; ++x) {
      confidence[y][x] = 0;
      labels[y][x] = MAP_CELL_FREE;
    }
  last_decay_ms = millis();
  map_mark_boundaries();
}

bool world_to_grid(float x_m, float y_m, int &gx, int &gy) {
  gx = (int)roundf(x_m / MAP_CELL_SIZE_M) + MAP_X_ZERO;
  gy = (int)roundf(y_m / MAP_CELL_SIZE_M) + MAP_Y_ZERO;
  return inside(gx, gy);
}

void grid_to_world(int gx, int gy, float &x_m, float &y_m) {
  x_m = (gx - MAP_X_ZERO) * MAP_CELL_SIZE_M;
  y_m = (gy - MAP_Y_ZERO) * MAP_CELL_SIZE_M;
}

int8_t map_get_cell(int gx, int gy) {
  return inside(gx, gy) ? labels[gy][gx] : MAP_CELL_UNKNOWN;
}

int map_get_confidence(int gx, int gy) {
  return inside(gx, gy) ? confidence[gy][gx] : MAP_BOUNDARY_SCORE;
}

void map_observe_weight(int x, int y) {
  if (inside(x, y)) update_score(x, y, confidence[y][x] + MAP_HIT_EVIDENCE);
}
void map_observe_wall(int x, int y) {
  if (inside(x, y)) update_score(x, y, confidence[y][x] - MAP_HIT_EVIDENCE);
}
void map_observe_free(int x, int y, bool upper_beam) {
  if (!inside(x, y)) return;
  int score = confidence[y][x];
  if (upper_beam && score > 0) return; // clear above a weight is not contrary evidence
  if (score > 0) score = max(0, score - MAP_FREE_EVIDENCE);
  else if (score < 0) score = min(0, score + MAP_FREE_EVIDENCE);
  update_score(x, y, score);
}
void map_reject_weight(int x, int y) {
  // Positive evidence at an explicitly uncollectable endpoint is withdrawn.
  if (inside(x, y) && confidence[y][x] > 0) update_score(x, y, 0);
}

void map_decay() {
  const uint32_t ticks = (uint32_t)(millis() - last_decay_ms) / MAP_DECAY_INTERVAL_MS;
  if (!ticks) return;
  last_decay_ms += ticks * MAP_DECAY_INTERVAL_MS;
  const int amount = ticks > 100 ? 100 : (int)ticks;
  for (int y = 0; y < MAP_GRID_H; ++y)
    for (int x = 0; x < MAP_GRID_W; ++x) {
      const int score = confidence[y][x];
      if (score == 0 || score == MAP_BOUNDARY_SCORE) continue;
      update_score(x, y, score > 0 ? max(0, score - amount) : min(0, score + amount));
    }
}

void map_ray_trace(float x0_m, float y0_m, float x1_m, float y1_m, bool upper_beam) {
  int x0, y0, x1, y1;
  world_to_grid(x0_m, y0_m, x0, y0);
  world_to_grid(x1_m, y1_m, x1, y1);
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (x0 != x1 || y0 != y1) {
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
    if (x0 == x1 && y0 == y1) break;
    map_observe_free(x0, y0, upper_beam);
  }
}
