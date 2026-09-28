#include "grid_map.h"

static int8_t confidence[MAP_GRID_H][MAP_GRID_W];
static int8_t labels[MAP_GRID_H][MAP_GRID_W];
// Cells changed since they were last sent; map_publish_changes() sends each
// once at its latest value, however many times it changed in between.
static bool dirty[MAP_GRID_H][MAP_GRID_W];
static const uint32_t PUBLISH_PERIOD_MS = 200;  // start a new sweep at most 5x per second
static const int PUBLISH_MAX_CELLS = 150;       // per call; the rest wait for the next loop
static int publish_cursor = 0;                  // next cell index in the current sweep
static uint32_t last_sweep_start_ms = 0;
// Decay removes one point per interval: 100 -> MAP_EXIT_SCORE - 1 in the configured time.
static const uint32_t DECAY_STEPS = 100 - (MAP_EXIT_SCORE - 1);
static const uint32_t WEIGHT_DECAY_INTERVAL_MS = MAP_WEIGHT_DECAY_MS / DECAY_STEPS;  // ~220 ms
static const uint32_t WALL_DECAY_INTERVAL_MS = MAP_WALL_DECAY_MS / DECAY_STEPS;      // ~990 ms
static uint32_t last_weight_decay_ms = 0;
static uint32_t last_wall_decay_ms = 0;

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
  dirty[y][x] = true;
}

void map_mark_boundaries() {
  for (int y = 0; y < MAP_GRID_H; ++y) {
    for (int x = 0; x < MAP_GRID_W; ++x) {
      if (x != 0 && y != 0 && x != MAP_GRID_W - 1 && y != MAP_GRID_H - 1) continue;
      confidence[y][x] = MAP_BOUNDARY_SCORE;
      labels[y][x] = MAP_CELL_BORDER;
      dirty[y][x] = true;
    }
  }
}

void map_publish_changes() {
  if (publish_cursor == 0) {
    if (millis() - last_sweep_start_ms < PUBLISH_PERIOD_MS) return;
    last_sweep_start_ms = millis();
  }
  const int total = MAP_GRID_W * MAP_GRID_H;
  int sent = 0;
  while (publish_cursor < total && sent < PUBLISH_MAX_CELLS) {
    const int x = publish_cursor % MAP_GRID_W;
    const int y = publish_cursor / MAP_GRID_W;
    ++publish_cursor;
    if (!dirty[y][x]) continue;
    dirty[y][x] = false;
    publish(x, y);
    ++sent;
  }
  if (publish_cursor >= total) publish_cursor = 0;
}

void map_publish_all() {
  // Free cells with zero confidence are the visualiser's default; skip them.
  for (int y = 0; y < MAP_GRID_H; ++y)
    for (int x = 0; x < MAP_GRID_W; ++x)
      if (confidence[y][x] != 0 || labels[y][x] != MAP_CELL_FREE) publish(x, y);
}

void map_init() {
  for (int y = 0; y < MAP_GRID_H; ++y)
    for (int x = 0; x < MAP_GRID_W; ++x) {
      confidence[y][x] = 0;
      labels[y][x] = MAP_CELL_FREE;
      dirty[y][x] = false;
    }
  publish_cursor = 0;
  last_weight_decay_ms = last_wall_decay_ms = millis();
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
void map_observe_free(int x, int y, bool protect_weights) {
  if (!inside(x, y)) return;
  int score = confidence[y][x];
  // Top beam clear above a weight, or the edge of a cone that may simply have
  // missed a small weight, is not contrary evidence.
  if (protect_weights && score > 0) return;
  if (score > 0) score = max(0, score - MAP_FREE_EVIDENCE);
  else if (score < 0) score = min(0, score + MAP_FREE_EVIDENCE);
  update_score(x, y, score);
}
void map_reject_weight(int x, int y) {
  // Positive evidence at an explicitly uncollectable endpoint is withdrawn.
  if (inside(x, y) && confidence[y][x] > 0) update_score(x, y, 0);
}

// Whole intervals elapsed since last_ms (capped at a full decay), advancing last_ms.
static int decay_ticks(uint32_t &last_ms, uint32_t interval_ms) {
  const uint32_t ticks = (uint32_t)(millis() - last_ms) / interval_ms;
  last_ms += ticks * interval_ms;
  return ticks > 100 ? 100 : (int)ticks;
}

void map_decay() {
  // Positive scores are weight evidence, negative are wall evidence.
  const int weight_amount = decay_ticks(last_weight_decay_ms, WEIGHT_DECAY_INTERVAL_MS);
  const int wall_amount = decay_ticks(last_wall_decay_ms, WALL_DECAY_INTERVAL_MS);
  if (!weight_amount && !wall_amount) return;
  for (int y = 0; y < MAP_GRID_H; ++y)
    for (int x = 0; x < MAP_GRID_W; ++x) {
      const int score = confidence[y][x];
      if (score == 0 || score == MAP_BOUNDARY_SCORE) continue;
      if (score > 0 && weight_amount) update_score(x, y, max(0, score - weight_amount));
      else if (score < 0 && wall_amount) update_score(x, y, min(0, score + wall_amount));
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
