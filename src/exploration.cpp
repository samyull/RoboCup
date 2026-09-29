#include "exploration.h"
#include "path_planner.h"
#include "arena_config.h"
#include <Arduino.h>
#include <string.h>

static const float    EXPLORED_FRACTION  = 0.6f;    // of a coarse cell's fine cells
static const float    FOOTPRINT_RADIUS_M = 0.155f;  // robot half-width

// Frontier score = 0.5 * distance of cluster centre from the start
//                + 0.05 * cluster size (coarse cells)
//                - 1.0 * path length (m)
//                - 1.0 if the centre is within 0.4 m of two walls (corner).
// Path cost outweighs distance from start, so near frontiers win, and among
// similar ones the robot works outwards towards the far end of the arena.
static const float    SCORE_FROM_START_PER_M = 0.5f;
static const float    SCORE_PER_CELL         = 0.05f;
static const float    SCORE_PATH_PER_M       = 1.0f;
static const float    SCORE_CORNER_PENALTY   = 1.0f;
static const float    CORNER_MARGIN_M        = 0.4f;
static const float    GOAL_HYSTERESIS        = 0.3f;    // switch goal only if this much better
static const uint32_t SKIP_GOAL_MS           = 10000;   // unreachable/stuck goal cooldown
static const int      PUBLISH_MAX_PER_CALL   = 40;
static const float    VIEW_GAIN_PER_CELL     = 0.02f;
static const float    MIN_GOAL_DISTANCE_M    = 0.20f; // beyond the 15 cm arrival radius

static bool     fine_seen[MAP_GRID_H][MAP_GRID_W];
static uint8_t  seen_count[EXPLORE_H][EXPLORE_W];
static uint8_t  cell_total[EXPLORE_H][EXPLORE_W];
static bool     explored[EXPLORE_H][EXPLORE_W];
static bool     unsent[EXPLORE_H][EXPLORE_W];
static uint32_t skip_until_ms[EXPLORE_H][EXPLORE_W];
static bool     reset_unsent = false;

static bool have_goal = false;
static int  goal_cx = 0, goal_cy = 0;

static bool interior(int gx, int gy) {
  return gx >= 1 && gx <= MAP_GRID_W - 2 && gy >= 1 && gy <= MAP_GRID_H - 2;
}

static void fine_to_coarse(int gx, int gy, int &cx, int &cy) {
  cx = (gx - 1) / EXPLORE_FINE_PER_CELL;
  cy = (gy - 1) / EXPLORE_FINE_PER_CELL;
}

static void coarse_fine_range(int cx, int cy, int &gx0, int &gy0, int &gx1, int &gy1) {
  gx0 = 1 + cx * EXPLORE_FINE_PER_CELL;
  gy0 = 1 + cy * EXPLORE_FINE_PER_CELL;
  gx1 = min(gx0 + EXPLORE_FINE_PER_CELL - 1, MAP_GRID_W - 2);
  gy1 = min(gy0 + EXPLORE_FINE_PER_CELL - 1, MAP_GRID_H - 2);
}

static void coarse_centre(int cx, int cy, float &x, float &y) {
  int gx0, gy0, gx1, gy1;
  coarse_fine_range(cx, cy, gx0, gy0, gx1, gy1);
  float x0, y0, x1, y1;
  grid_to_world(gx0, gy0, x0, y0);
  grid_to_world(gx1, gy1, x1, y1);
  x = 0.5f * (x0 + x1);
  y = 0.5f * (y0 + y1);
}

static void clear_seen() {
  memset(fine_seen, 0, sizeof(fine_seen));
  memset(seen_count, 0, sizeof(seen_count));
  memset(explored, 0, sizeof(explored));
  memset(unsent, 0, sizeof(unsent));
  have_goal = false;
}

void exploration_init() {
  clear_seen();
  memset(skip_until_ms, 0, sizeof(skip_until_ms));
  memset(cell_total, 0, sizeof(cell_total));
  for (int gy = 1; gy <= MAP_GRID_H - 2; ++gy)
    for (int gx = 1; gx <= MAP_GRID_W - 2; ++gx) {
      int cx, cy;
      fine_to_coarse(gx, gy, cx, cy);
      ++cell_total[cy][cx];
    }
  reset_unsent = true;  // the visualiser may still show a previous round
}

void exploration_reset() {
  clear_seen();
  reset_unsent = true;
  Serial.println("Arena fully explored - starting a second exploration pass");
}

void exploration_mark_fine_seen(int gx, int gy) {
  if (!interior(gx, gy) || fine_seen[gy][gx]) return;
  fine_seen[gy][gx] = true;
  int cx, cy;
  fine_to_coarse(gx, gy, cx, cy);
  ++seen_count[cy][cx];
  if (!explored[cy][cx] && seen_count[cy][cx] >= EXPLORED_FRACTION * cell_total[cy][cx]) {
    explored[cy][cx] = true;
    unsent[cy][cx] = true;
  }
}

void exploration_mark_footprint(const Pose &pose) {
  // A weight under the robot would have triggered the catchment, so the
  // ground it covers counts as seen.
  int gx0, gy0;
  world_to_grid(pose.x, pose.y, gx0, gy0);
  const int r = (int)ceilf(FOOTPRINT_RADIUS_M / MAP_CELL_SIZE_M);
  for (int gy = gy0 - r; gy <= gy0 + r; ++gy)
    for (int gx = gx0 - r; gx <= gx0 + r; ++gx) {
      float x, y;
      grid_to_world(gx, gy, x, y);
      const float dx = x - pose.x, dy = y - pose.y;
      if (dx * dx + dy * dy <= FOOTPRINT_RADIUS_M * FOOTPRINT_RADIUS_M) exploration_mark_fine_seen(gx, gy);
    }
}

bool exploration_fine_seen(int gx, int gy) {
  return interior(gx, gy) && fine_seen[gy][gx];
}

bool exploration_explored(int cx, int cy) {
  return cx >= 0 && cx < EXPLORE_W && cy >= 0 && cy < EXPLORE_H && explored[cy][cx];
}

void exploration_publish_changes() {
  if (reset_unsent) {
    Serial.println("V_RESET");
    reset_unsent = false;
  }
  int sent = 0;
  for (int cy = 0; cy < EXPLORE_H && sent < PUBLISH_MAX_PER_CALL; ++cy)
    for (int cx = 0; cx < EXPLORE_W && sent < PUBLISH_MAX_PER_CALL; ++cx) {
      if (!unsent[cy][cx]) continue;
      unsent[cy][cx] = false;
      Serial.print("V,"); Serial.print(cx); Serial.print(","); Serial.println(cy);
      ++sent;
    }
}

void exploration_publish_all() {
  Serial.println("V_RESET");
  reset_unsent = false;
  for (int cy = 0; cy < EXPLORE_H; ++cy)
    for (int cx = 0; cx < EXPLORE_W; ++cx) {
      unsent[cy][cx] = false;
      if (!explored[cy][cx]) continue;
      Serial.print("V,"); Serial.print(cx); Serial.print(","); Serial.println(cy);
    }
}

// Coarse cells that are mostly confirmed wall are not worth exploring.
static bool mostly_wall(int cx, int cy) {
  int gx0, gy0, gx1, gy1, walls = 0;
  coarse_fine_range(cx, cy, gx0, gy0, gx1, gy1);
  for (int gy = gy0; gy <= gy1; ++gy)
    for (int gx = gx0; gx <= gx1; ++gx)
      if (map_get_cell(gx, gy) == MAP_CELL_OBSTACLE) ++walls;
  return 2 * walls >= cell_total[cy][cx];
}

// Unexplored, worth visiting, and next to explored ground (or to the robot,
// which bootstraps the search before anything has been explored).
static bool is_frontier(int cx, int cy, int robot_cx, int robot_cy, uint32_t now) {
  if (explored[cy][cx] || (int32_t)(skip_until_ms[cy][cx] - now) > 0 || mostly_wall(cx, cy)) return false;
  static const int8_t DX[4] = {1, -1, 0, 0}, DY[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    const int nx = cx + DX[k], ny = cy + DY[k];
    if (nx < 0 || nx >= EXPLORE_W || ny < 0 || ny >= EXPLORE_H) continue;
    if (explored[ny][nx] || (nx == robot_cx && ny == robot_cy)) return true;
  }
  return false;
}

// Potential visibility after turning at a viewpoint: eight rays, at most
// 60 cm, stopping at mapped walls. Count unseen cells in this frontier cluster,
// beyond the robot footprint. This is a goal score, not a new seen observation.
static int visible_unseen(int gx, int gy, int cluster,
                          const int16_t clusters[EXPLORE_H][EXPLORE_W]) {
  int gain = 0;
  for (int dy = -1; dy <= 1; ++dy)
    for (int dx = -1; dx <= 1; ++dx) {
      if (!dx && !dy) continue;
      for (int step = 1; step <= 12; ++step) {
        if (step * MAP_CELL_SIZE_M * (dx && dy ? 1.414214f : 1.0f) > 0.60f) break;
        const int x = gx + step * dx, y = gy + step * dy;
        if (!interior(x, y) || map_get_cell(x, y) == MAP_CELL_OBSTACLE) break;
        if (dx && dy && (map_get_cell(x - dx, y) == MAP_CELL_OBSTACLE ||
                         map_get_cell(x, y - dy) == MAP_CELL_OBSTACLE)) break;
        int cx, cy;
        fine_to_coarse(x, y, cx, cy);
        if (step >= 4 && !fine_seen[y][x] && clusters[cy][cx] == cluster) ++gain;
      }
    }
  return gain;
}

bool exploration_choose_frontier(const Pose &pose, FrontierGoal &goal) {
  const uint32_t now = millis();
  int rgx, rgy, robot_cx, robot_cy;
  world_to_grid(pose.x, pose.y, rgx, rgy);
  fine_to_coarse(constrain(rgx, 1, MAP_GRID_W - 2), constrain(rgy, 1, MAP_GRID_H - 2), robot_cx, robot_cy);

  static bool frontier[EXPLORE_H][EXPLORE_W];
  static int16_t cluster_of[EXPLORE_H][EXPLORE_W];
  for (int cy = 0; cy < EXPLORE_H; ++cy)
    for (int cx = 0; cx < EXPLORE_W; ++cx) {
      frontier[cy][cx] = is_frontier(cx, cy, robot_cx, robot_cy, now);
      cluster_of[cy][cx] = -1;
    }

  bool found_best = false, found_current = false;
  FrontierGoal best = {}, current = {};
  int best_source = 0, current_source = 0;
  static int16_t queue[EXPLORE_W * EXPLORE_H];
  int cluster_id = 0;

  for (int sy = 0; sy < EXPLORE_H; ++sy)
    for (int sx = 0; sx < EXPLORE_W; ++sx) {
      if (!frontier[sy][sx] || cluster_of[sy][sx] >= 0) continue;
      // Flood-fill one 8-connected cluster.
      int head = 0, tail = 0, size = 0;
      float sum_x = 0.0f, sum_y = 0.0f;
      bool holds_current_goal = false;
      queue[tail++] = sy * EXPLORE_W + sx;
      cluster_of[sy][sx] = cluster_id;
      while (head < tail) {
        const int cx = queue[head] % EXPLORE_W, cy = queue[head] / EXPLORE_W;
        ++head;
        float x, y;
        coarse_centre(cx, cy, x, y);
        sum_x += x; sum_y += y; ++size;
        if (have_goal && cx == goal_cx && cy == goal_cy) holds_current_goal = true;
        for (int dy = -1; dy <= 1; ++dy)
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = cx + dx, ny = cy + dy;
            if (nx < 0 || nx >= EXPLORE_W || ny < 0 || ny >= EXPLORE_H) continue;
            if (!frontier[ny][nx] || cluster_of[ny][nx] >= 0) continue;
            cluster_of[ny][nx] = cluster_id;
            queue[tail++] = ny * EXPLORE_W + nx;
          }
      }
      const float centre_x = sum_x / size, centre_y = sum_y / size;

      // Approach from observed ground in or beside the frontier. Require room
      // to manoeuvre and an unobstructed view of unseen cells in this cluster.
      int goal_gx = -1, goal_gy = -1;
      int goal_gain = 0, goal_source = 0;
      float best_view_score = -1e9f;
      for (int i = 0; i < tail; ++i) {
        int gx0, gy0, gx1, gy1;
        coarse_fine_range(queue[i] % EXPLORE_W, queue[i] / EXPLORE_W, gx0, gy0, gx1, gy1);
        for (int gy = max(1, gy0 - 4); gy <= min(MAP_GRID_H - 2, gy1 + 4); ++gy)
          for (int gx = max(1, gx0 - 4); gx <= min(MAP_GRID_W - 2, gx1 + 4); ++gx) {
            if (!fine_seen[gy][gx] || !planner_goal_clear(gx, gy)) continue;
            float x, y;
            grid_to_world(gx, gy, x, y);
            if (hypotf(x - pose.x, y - pose.y) < MIN_GOAL_DISTANCE_M) continue;
            int cx, cy;
            fine_to_coarse(gx, gy, cx, cy);
            if ((int32_t)(skip_until_ms[cy][cx] - now) > 0) continue;
            const int gain = visible_unseen(gx, gy, cluster_id, cluster_of);
            if (gain < 3) continue;
            const float score = VIEW_GAIN_PER_CELL * gain - planner_distance_m(gx, gy) -
                                0.15f * hypotf(x - centre_x, y - centre_y);
            if (score > best_view_score) {
              best_view_score = score; goal_gx = gx; goal_gy = gy;
              goal_gain = gain; goal_source = queue[i];
            }
          }
      }
      ++cluster_id;
      if (goal_gx < 0) continue;  // nothing in this cluster can be reached

      const bool corner =
          (centre_x < CORNER_MARGIN_M || centre_x > MAP_GRID_SIZE_X_M - CORNER_MARGIN_M) &&
          (centre_y < CORNER_MARGIN_M || centre_y > MAP_GRID_SIZE_Y_M - CORNER_MARGIN_M);
      const float from_start = hypotf(centre_x - ROBOT_START_X_M, centre_y - ROBOT_START_Y_M);
      FrontierGoal candidate;
      candidate.gx = goal_gx;
      candidate.gy = goal_gy;
      grid_to_world(goal_gx, goal_gy, candidate.x, candidate.y);
      candidate.score = SCORE_FROM_START_PER_M * from_start + SCORE_PER_CELL * size -
                        SCORE_PATH_PER_M * planner_distance_m(goal_gx, goal_gy) -
                        (corner ? SCORE_CORNER_PENALTY : 0.0f) + VIEW_GAIN_PER_CELL * goal_gain;
      if (!found_best || candidate.score > best.score) {
        best = candidate; best_source = goal_source; found_best = true;
      }
      if (holds_current_goal) { current = candidate; current_source = goal_source; found_current = true; }
    }

  if (!found_best) {
    have_goal = false;
    return false;
  }
  const bool keep = found_current && best.score < current.score + GOAL_HYSTERESIS;
  goal = keep ? current : best;
  have_goal = true;
  const int source = keep ? current_source : best_source;
  goal_cx = source % EXPLORE_W;
  goal_cy = source / EXPLORE_W;
  return true;
}

void exploration_skip_current_goal() {
  if (!have_goal) return;
  // Retire the neighbourhood too: otherwise a goal a few cm away can repeat
  // the same blocked approach on the very next plan.
  for (int dy = -1; dy <= 1; ++dy)
    for (int dx = -1; dx <= 1; ++dx) {
      const int cx = goal_cx + dx, cy = goal_cy + dy;
      if (cx >= 0 && cx < EXPLORE_W && cy >= 0 && cy < EXPLORE_H)
        skip_until_ms[cy][cx] = millis() + SKIP_GOAL_MS;
    }
  have_goal = false;
}
