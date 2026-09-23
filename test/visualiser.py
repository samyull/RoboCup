#!/usr/bin/env python3
"""
Live visualiser for the robot's occupancy grid.

Reads lines over serial and renders:
  - the grid as coloured squares (unknown / free / obstacle / weight / robot cell)
  - the robot's continuous pose (pose.x, pose.y) as a dark green dot
  - the current navigation target as an orange dot

Expected serial line formats (one per line, comma-separated):
  C,gx,gy,value      -> a grid cell update (from map_set_cell's logging)
  P,x_m,y_m,theta    -> the robot's pose in world coordinates (metres, radians)
  T,x_m,y_m          -> the current navigation target in world coordinates

The C, line format already matches what map_set_cell() prints.
P and T are NOT currently printed by main.cpp - add these two lines
wherever convenient (e.g. once per loop, after map_update()):

    Serial.print("P,"); Serial.print(pose.x, 3); Serial.print(",");
    Serial.print(pose.y, 3); Serial.print(","); Serial.println(pose.theta, 4);

    Serial.print("T,"); Serial.print(target.x, 3);
    Serial.print(","); Serial.println(target.y, 3);

If no P/T lines ever arrive, the grid still updates fine - the pose/target
dots just won't be drawn.

Usage:
    python visualiser.py COM5          (Windows)
    python visualiser.py /dev/ttyACM0  (Linux/Mac)

Requires: pyserial, matplotlib, numpy
    pip install pyserial matplotlib numpy
"""

import sys
import re
import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.animation import FuncAnimation

# --- Must match grid_map.h on the robot ---
GRID_W = 100
GRID_H = 100
CELL_SIZE_M = 0.05
X_ZERO = GRID_W // 2
Y_ZERO = GRID_H // 2

BAUD_RATE = 115200

# --- Cell values, matching the MapCell enum ---
CELL_UNKNOWN  = -1
CELL_FREE     = 0
CELL_OBSTACLE = 1
CELL_ROBOT    = 2
CELL_WEIGHT   = 3

# Colours, indexed in the same order as the sorted cell values above
# (-1, 0, 1, 2, 3) -> (unknown, free, obstacle, robot, weight)
CELL_COLORS = [
    "#808080",  # unknown       - gray
    "#E6F0FA",  # free          - off-white blue
    "#DC143C",  # obstacle/wall - crimson
    "#32CD32",  # robot cell    - lime green
    "#DAA520",  # weight        - goldish
]
POSE_COLOR = "#145A14"    # dark green
TARGET_COLOR = "#FF8C00"  # orange

CELL_VALUES = [CELL_UNKNOWN, CELL_FREE, CELL_OBSTACLE, CELL_ROBOT, CELL_WEIGHT]

# Matches optional lines like "[pose] ... x=1.234 y=-0.567 theta_deg=12.3"
# as a fallback if you're not printing a dedicated P, line yet.
POSE_DEBUG_RE = re.compile(
    r"x=\s*(-?\d+\.?\d*).*?y=\s*(-?\d+\.?\d*).*?theta_deg=\s*(-?\d+\.?\d*)"
)


def grid_to_world(gx, gy):
    x_m = (gx - X_ZERO) * CELL_SIZE_M
    y_m = (gy - Y_ZERO) * CELL_SIZE_M
    return x_m, y_m


class Visualiser:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD_RATE, timeout=0)
        self.grid = np.full((GRID_H, GRID_W), CELL_UNKNOWN, dtype=int)
        self.pose = None    # (x_m, y_m, theta_rad or None)
        self.target = None  # (x_m, y_m)
        self._buf = ""

        cmap = ListedColormap(CELL_COLORS)
        bounds = [v - 0.5 for v in CELL_VALUES] + [CELL_VALUES[-1] + 0.5]
        norm = BoundaryNorm(bounds, cmap.N)

        world_x0, world_y0 = grid_to_world(0, 0)
        world_x1, world_y1 = grid_to_world(GRID_W, GRID_H)

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.im = self.ax.imshow(
            self.grid, cmap=cmap, norm=norm, origin="lower",
            extent=[world_x0, world_x1, world_y0, world_y1],
            interpolation="nearest",
        )
        self.pose_dot, = self.ax.plot([], [], "o", color=POSE_COLOR,
                                       markersize=10, label="pose")
        self.target_dot, = self.ax.plot([], [], "o", color=TARGET_COLOR,
                                         markersize=10, label="target")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title("Occupancy grid")
        self.ax.legend(loc="upper right")

    def _handle_line(self, line):
        parts = line.strip().split(",")
        if not parts or not parts[0]:
            return

        tag = parts[0]
        try:
            if tag == "C" and len(parts) == 4:
                gx, gy, value = int(parts[1]), int(parts[2]), int(parts[3])
                if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                    self.grid[gy, gx] = value

            elif tag == "P" and len(parts) == 4:
                self.pose = (float(parts[1]), float(parts[2]), float(parts[3]))

            elif tag == "T" and len(parts) == 3:
                self.target = (float(parts[1]), float(parts[2]))

            else:
                m = POSE_DEBUG_RE.search(line)
                if m:
                    x, y, theta_deg = map(float, m.groups())
                    self.pose = (x, y, np.radians(theta_deg))

        except ValueError:
            pass  # malformed/partial line, just skip it

    def _read_serial(self):
        n = self.ser.in_waiting
        if n:
            self._buf += self.ser.read(n).decode(errors="replace")
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            self._handle_line(line)

    def update(self, _frame):
        self._read_serial()

        self.im.set_data(self.grid)

        if self.pose is not None:
            self.pose_dot.set_data([self.pose[0]], [self.pose[1]])
        if self.target is not None:
            self.target_dot.set_data([self.target[0]], [self.target[1]])

        return self.im, self.pose_dot, self.target_dot

    def run(self):
        anim = FuncAnimation(self.fig, self.update, interval=100, blit=False)
        plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <serial-port>")
        print("  e.g. python visualiser.py COM5")
        print("       python visualiser.py /dev/ttyACM0")
        sys.exit(1)

    Visualiser(sys.argv[1]).run()
