#include "path_planner.h"
#include "grid_map.h"
#include "navigation.h"
#include "exploration.h"
#include <Arduino.h>

static const float    ROBOT_HALF_WIDTH_M  = 0.155f;
static const uint16_t STEP_COST           = 10;   // per 5 cm straight
static const uint16_t DIAG_COST           = 14;
// Robot centre is kept at least this far from a mapped obstacle cell's centre.
// Also decides which weights are reachable: a weight closer than this to a
// wall is never chosen (the funnel can't collect it there anyway).
static const float    OBSTACLE_CLEARANCE_M = ROBOT_HALF_WIDTH_M;
static const uint16_t BAND_ESCAPE_PENALTY = 200;  // per keep-out cell when escaping one (band or margin)
static const float    WAYPOINT_REACHED_M  = 0.12f;
static const int      MAX_PATH_CELLS      = 600;
static const int      MAX_WAYPOINTS       = 48;

enum CellClass : uint8_t {
  CELL_OPEN,       // free to drive
  CELL_BAND,       // within a half-width of the arena border: never entered, except to escape it
  CELL_MARGIN,     // within OBSTACLE_CLEARANCE_M of a mapped obstacle: same rule as the band
  CELL_BLOCKED,    // wall, border, or (going home) unseen
};

static const int N_CELLS = MAP_GRID_W * MAP_GRID_H;
static uint8_t  cell_class[MAP_GRID_H][MAP_GRID_W];
static uint16_t dist[MAP_GRID_H][MAP_GRID_W];
static int16_t  parent[MAP_GRID_H][MAP_GRID_W];
static bool     have_plan = false;

// Indexed binary min-heap of cell indices keyed on dist.
static int16_t heap[N_CELLS];
static int16_t heap_pos[N_CELLS];  // -1 when not in the heap
static int     heap_size = 0;

static uint16_t key(int idx) { return dist[idx / MAP_GRID_W][idx % MAP_GRID_W]; }

static void heap_swap(int a, int b) {
  const int16_t t = heap[a]; heap[a] = heap[b]; heap[b] = t;
  heap_pos[heap[a]] = a; heap_pos[heap[b]] = b;
}
static void heap_up(int i) {
  while (i > 0) {
    const int p = (i - 1) / 2;
    if (key(heap[p]) <= key(heap[i])) break;
    heap_swap(i, p);
    i = p;
  }
}
static void heap_down(int i) {
  for (;;) {
    const int l = 2 * i + 1, r = l + 1;
    int m = i;
    if (l < heap_size && key(heap[l]) < key(heap[m])) m = l;
    if (r < heap_size && key(heap[r]) < key(heap[m])) m = r;
    if (m == i) return;
    heap_swap(i, m);
    i = m;
  }
}
static void heap_push_or_decrease(int idx) {
  if (heap_pos[idx] < 0) {
    heap[heap_size] = idx;
    heap_pos[idx] = heap_size;
    ++heap_size;
  }
  heap_up(heap_pos[idx]);
}
static int heap_pop() {
  const int top = heap[0];
  heap_pos[top] = -1;
  --heap_size;
  if (heap_size > 0) {
    heap[0] = heap[heap_size];
    heap_pos[heap[0]] = 0;
    heap_down(0);
  }
  return top;
}

static bool in_grid(int gx, int gy) {
  return gx >= 0 && gx < MAP_GRID_W && gy >= 0 && gy < MAP_GRID_H;
}

// Cells the robot never enters, except to drive out of after ending up in one.
static bool keep_out(uint8_t c) {
  return c == CELL_BAND || c == CELL_MARGIN;
}

static void classify_cells(PlanMode mode) {
  for (int gy = 0; gy < MAP_GRID_H; ++gy)
    for (int gx = 0; gx < MAP_GRID_W; ++gx) {
      const int8_t label = map_get_cell(gx, gy);
      float x, y;
      grid_to_world(gx, gy, x, y);
      uint8_t c = CELL_OPEN;
      if (label == MAP_CELL_BORDER || label == MAP_CELL_OBSTACLE) c = CELL_BLOCKED;
      else if (x < ROBOT_HALF_WIDTH_M || x > MAP_GRID_SIZE_X_M - ROBOT_HALF_WIDTH_M ||
               y < ROBOT_HALF_WIDTH_M || y > MAP_GRID_SIZE_Y_M - ROBOT_HALF_WIDTH_M) c = CELL_BAND;
      else if (mode == PLAN_SEEN_ONLY && !exploration_fine_seen(gx, gy)) c = CELL_BLOCKED;
      cell_class[gy][gx] = c;
    }
  // Keep-out margin around mapped obstacles (the border has the band instead).
  const float clearance = OBSTACLE_CLEARANCE_M;
  const int r = (int)ceilf(clearance / MAP_CELL_SIZE_M);
  for (int gy = 0; gy < MAP_GRID_H; ++gy)
    for (int gx = 0; gx < MAP_GRID_W; ++gx) {
      if (map_get_cell(gx, gy) != MAP_CELL_OBSTACLE) continue;
      for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx) {
          if ((dx * dx + dy * dy) * MAP_CELL_SIZE_M * MAP_CELL_SIZE_M > clearance * clearance) continue;
          const int nx = gx + dx, ny = gy + dy;
          if (in_grid(nx, ny) && cell_class[ny][nx] == CELL_OPEN) cell_class[ny][nx] = CELL_MARGIN;
        }
    }
}

void planner_plan(const Pose &pose, PlanMode mode) {
  classify_cells(mode);
  int sgx, sgy;
  world_to_grid(pose.x, pose.y, sgx, sgy);
  sgx = constrain(sgx, 0, MAP_GRID_W - 1);
  sgy = constrain(sgy, 0, MAP_GRID_H - 1);
  // Started in a keep-out cell (drifted into the band, pushed near a wall, or a
  // wall got mapped under the robot): allow keep-out cells, at a cost, to get out.
  const bool escape = cell_class[sgy][sgx] != CELL_OPEN;

  for (int gy = 0; gy < MAP_GRID_H; ++gy)
    for (int gx = 0; gx < MAP_GRID_W; ++gx) {
      dist[gy][gx] = PLAN_UNREACHABLE;
      parent[gy][gx] = -1;
    }
  for (int i = 0; i < N_CELLS; ++i) heap_pos[i] = -1;
  heap_size = 0;

  // Always seed from the robot's own cell, whatever it is, so the robot can
  // plan its way out rather than stalling.
  dist[sgy][sgx] = 0;
  heap_push_or_decrease(sgy * MAP_GRID_W + sgx);
  while (heap_size > 0) {
    const int u = heap_pop();
    const int ux = u % MAP_GRID_W, uy = u / MAP_GRID_W;
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx) {
        if (!dx && !dy) continue;
        const int vx = ux + dx, vy = uy + dy;
        if (!in_grid(vx, vy)) continue;
        const uint8_t c = cell_class[vy][vx];
        if (c == CELL_BLOCKED || (keep_out(c) && !escape)) continue;
        // No cutting diagonally past a blocked corner.
        if (dx && dy && (cell_class[uy][vx] == CELL_BLOCKED || cell_class[vy][ux] == CELL_BLOCKED ||
            (!escape && (keep_out(cell_class[uy][vx]) || keep_out(cell_class[vy][ux]))))) continue;
        uint32_t step = (dx && dy) ? DIAG_COST : STEP_COST;
        if (keep_out(c)) step += BAND_ESCAPE_PENALTY;
        const uint32_t nd = (uint32_t)dist[uy][ux] + step;
        if (nd >= PLAN_UNREACHABLE || nd >= dist[vy][vx]) continue;
        dist[vy][vx] = (uint16_t)nd;
        parent[vy][vx] = (int16_t)u;
        heap_push_or_decrease(vy * MAP_GRID_W + vx);
      }
  }
  have_plan = true;
}

uint16_t planner_cost(int gx, int gy) {
  if (!have_plan || !in_grid(gx, gy)) return PLAN_UNREACHABLE;
  return dist[gy][gx];
}

float planner_distance_m(int gx, int gy) {
  const uint16_t c = planner_cost(gx, gy);
  if (c == PLAN_UNREACHABLE) return 1e6f;
  return c * (MAP_CELL_SIZE_M / STEP_COST);
}

// --- Path extraction, smoothing and following ---
static Target waypoints[MAX_WAYPOINTS];
static int waypoint_count = 0;
static int waypoint_index = 0;
static float path_start_x = 0.0f, path_start_y = 0.0f;

// Check the end cell, every cell on the line and both side cells on a diagonal
// step. A line starting in a keep-out cell may cross keep-out cells to get out,
// but shortcuts may not enter them from open ground. The start cell itself is
// not checked, so the robot can always drive out of wherever it is.
static bool cells_clear(int x0, int y0, int x1, int y1) {
  if (!in_grid(x0, y0) || !in_grid(x1, y1)) return false;
  const bool escape = cell_class[y0][x0] != CELL_OPEN;
  auto passable = [escape](int x, int y) {
    return cell_class[y][x] == CELL_OPEN || (escape && keep_out(cell_class[y][x]));
  };
  if (!passable(x1, y1)) return false;
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0, y = y0;
  while (x != x1 || y != y1) {
    const int old_x = x, old_y = y;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }
    if (!passable(x, y)) return false;
    if (x != old_x && y != old_y && (!passable(x, old_y) || !passable(old_x, y))) return false;
  }
  return true;
}

static void publish_path() {
  Serial.print("PATH,"); Serial.print(path_start_x, 2); Serial.print(","); Serial.print(path_start_y, 2);
  for (int i = 0; i < waypoint_count; ++i) {
    Serial.print(","); Serial.print(waypoints[i].x, 2);
    Serial.print(","); Serial.print(waypoints[i].y, 2);
  }
  Serial.println();
  if (waypoint_count > 0) {
    Serial.print("T,"); Serial.print(waypoints[waypoint_count - 1].x, 2);
    Serial.print(","); Serial.println(waypoints[waypoint_count - 1].y, 2);
  }
}

bool planner_set_goal(int gx, int gy) {
  waypoint_count = 0;
  waypoint_index = 0;
  if (planner_cost(gx, gy) == PLAN_UNREACHABLE) return false;

  static int16_t cells[MAX_PATH_CELLS];
  int n = 0;
  for (int idx = gy * MAP_GRID_W + gx; idx >= 0; idx = parent[idx / MAP_GRID_W][idx % MAP_GRID_W]) {
    if (n >= MAX_PATH_CELLS) return false;
    cells[n++] = (int16_t)idx;
  }
  for (int i = 0; i < n / 2; ++i) { const int16_t t = cells[i]; cells[i] = cells[n - 1 - i]; cells[n - 1 - i] = t; }
  grid_to_world(cells[0] % MAP_GRID_W, cells[0] / MAP_GRID_W, path_start_x, path_start_y);

  // Keep only the corners: from each anchor, jump to the furthest cell in clear line of sight.
  int anchor = 0;
  while (anchor < n - 1 && waypoint_count < MAX_WAYPOINTS) {
    int j = anchor + 1;
    while (j + 1 < n && cells_clear(cells[anchor] % MAP_GRID_W, cells[anchor] / MAP_GRID_W,
                                    cells[j + 1] % MAP_GRID_W, cells[j + 1] / MAP_GRID_W)) ++j;
    if (waypoint_count == MAX_WAYPOINTS - 1 && j != n - 1) {
      waypoint_count = 0;  // Never replace a long safe route with an unchecked shortcut.
      return false;
    }
    grid_to_world(cells[j] % MAP_GRID_W, cells[j] / MAP_GRID_W, waypoints[waypoint_count].x,
                  waypoints[waypoint_count].y);
    ++waypoint_count;
    anchor = j;
  }
  if (waypoint_count == 0) {  // already in the goal cell
    grid_to_world(gx, gy, waypoints[0].x, waypoints[0].y);
    waypoint_count = 1;
  }
  publish_path();
  return true;
}

// planner_follow() holding still because its straight line is blocked.
static bool follow_blocked = false;
static uint32_t follow_blocked_since_ms = 0;
static uint32_t last_follow_ms = 0;

void planner_clear_path() {
  waypoint_count = 0;
  waypoint_index = 0;
  follow_blocked = false;
}

bool planner_follow(const Pose &pose, float arrive_m, int &left_pct, int &right_pct) {
  left_pct = right_pct = 0;
  const uint32_t now = millis();
  // A gap in calls means a different state was running: start the blocked count afresh.
  if (now - last_follow_ms > 200) follow_blocked = false;
  last_follow_ms = now;
  if (waypoint_count == 0) return false;
  while (waypoint_index < waypoint_count - 1 &&
         distance_to(pose, waypoints[waypoint_index]) < WAYPOINT_REACHED_M &&
         planner_straight_clear(pose.x, pose.y, waypoints[waypoint_index + 1].x,
                                waypoints[waypoint_index + 1].y)) ++waypoint_index;
  if (waypoint_index == waypoint_count - 1 && distance_to(pose, waypoints[waypoint_index]) <= arrive_m) {
    return true;
  }
  if (planner_straight_clear(pose.x, pose.y, waypoints[waypoint_index].x, waypoints[waypoint_index].y)) {
    navigate_to_target(pose, waypoints[waypoint_index], left_pct, right_pct);
    follow_blocked = false;
  } else if (!follow_blocked) {
    // Held still; the next replan usually finds a new route from here.
    follow_blocked = true;
    follow_blocked_since_ms = now;
  }
  return false;
}

uint32_t planner_blocked_ms() {
  if (!follow_blocked || millis() - last_follow_ms > 200) return 0;
  return millis() - follow_blocked_since_ms;
}

bool planner_cell_open(int gx, int gy) {
  return have_plan && in_grid(gx, gy) && cell_class[gy][gx] == CELL_OPEN;
}

bool planner_straight_clear(float x0, float y0, float x1, float y1) {
  if (!have_plan) return false;
  int ax, ay, bx, by;
  world_to_grid(x0, y0, ax, ay);
  world_to_grid(x1, y1, bx, by);
  return cells_clear(ax, ay, bx, by);
}

bool planner_goal_clear(int gx, int gy) {
  // Viewpoints need an extra cell of breathing room beyond the drive footprint.
  for (int dy = -1; dy <= 1; ++dy)
    for (int dx = -1; dx <= 1; ++dx)
      if (!in_grid(gx + dx, gy + dy) || cell_class[gy + dy][gx + dx] != CELL_OPEN) return false;
  return planner_cost(gx, gy) != PLAN_UNREACHABLE;
}
