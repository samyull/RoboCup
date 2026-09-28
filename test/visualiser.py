#!/usr/bin/env python3
"""Live robot grid. Install: pip install pyserial matplotlib numpy

Run: python visualiser.py COM5
     python visualiser.py COM5 --no-reset
     python visualiser.py COM5 --debug

Sends R once per launch by default (bench testing: resets robot state).
Reconnects to the same port after USB reset, without sending R again.
On every other connect it sends M, and the robot resends its whole map.
"""

import argparse
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap, ListedColormap, Normalize
from matplotlib.patches import Patch
import numpy as np
import serial

# Must match grid_map.h, including its grid dimensions.
CELL_SIZE_M = 0.05
MARGIN_CELLS = 1
GRID_SIZE_X_M = 4.9
GRID_SIZE_Y_M = 2.4
# round avoids truncating a nominal 48 cells to 47 due to float rounding.
GRID_W = round(GRID_SIZE_X_M / CELL_SIZE_M) + 2 * MARGIN_CELLS
GRID_H = round(GRID_SIZE_Y_M / CELL_SIZE_M) + 2 * MARGIN_CELLS
X_ZERO = Y_ZERO = MARGIN_CELLS
BAUD_RATE = 115200

CELL_BORDER, CELL_UNKNOWN, CELL_FREE = -2, -1, 0
CELL_OBSTACLE, CELL_ROBOT, CELL_WEIGHT = 1, 2, 3
# Cells are coloured by confidence: grey at 0, shading to solid red (wall) at
# -MAP_ENTER_SCORE and solid gold (weight) at +MAP_ENTER_SCORE, and staying
# solid beyond. Border cells (score -101) are drawn black.
MAP_ENTER_SCORE = 20  # must match grid_map.h
CONFIDENCE_CMAP = LinearSegmentedColormap.from_list(
    "confidence", ["red", "grey", "gold"]).with_extremes(under="black")
# Coarse exploration grid (exploration.h): 4x4 map cells over the arena interior.
EXPLORE_FINE_PER_CELL = 4
EXPLORE_W = (GRID_W - 2 + EXPLORE_FINE_PER_CELL - 1) // EXPLORE_FINE_PER_CELL
EXPLORE_H = (GRID_H - 2 + EXPLORE_FINE_PER_CELL - 1) // EXPLORE_FINE_PER_CELL
UNEXPLORED_SHADE = (0.0, 0.0, 0.0, 0.35)  # translucent dark over unexplored ground
PATH_COLOR = "#FF8C00"
HEADING_ARROW_M = 0.2  # length of the heading arrow drawn on the pose dot
POSE_COLOR = "#39FF14"  # bright green, stands out on the grey grid
SERIAL_ERRORS =(serial.SerialException, OSError)


def grid_to_world(gx, gy):
    return (gx - X_ZERO) * CELL_SIZE_M, (gy - Y_ZERO) * CELL_SIZE_M


class Visualiser:
    def __init__(self, port, reset=True, debug=False):
        self.port = port
        self.debug = debug
        self.ser = None
        self.reset_pending = reset
        self.next_connect = 0.0
        self.last_rx = None
        self.connected_at = None
        self.status = f"Connecting to {port}"
        self.last_message = ""
        self.last_error = ""
        self.closed = False
        self._buf = ""
        self.anim = None
        self.grid = np.full((GRID_H, GRID_W), CELL_UNKNOWN, dtype=int)
        self.scores = np.zeros((GRID_H, GRID_W), dtype=int)
        self.explored = np.zeros((EXPLORE_H, EXPLORE_W), dtype=bool)
        self._clear_state()

        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.fig.subplots_adjust(bottom=0.18)
        # Firmware coordinates name cell centres.
        x_edges = (np.arange(GRID_W + 1) - X_ZERO - 0.5) * CELL_SIZE_M
        y_edges = (np.arange(GRID_H + 1) - Y_ZERO - 0.5) * CELL_SIZE_M
        self.mesh = self.ax.pcolormesh(
            x_edges, y_edges, self._display_scores(), cmap=CONFIDENCE_CMAP,
            norm=Normalize(vmin=-MAP_ENTER_SCORE, vmax=MAP_ENTER_SCORE),
            edgecolors="black", linewidth=0.4, shading="flat",
        )
        colorbar = self.fig.colorbar(self.mesh, ax=self.ax, fraction=0.025, pad=0.02)
        colorbar.set_label("confidence (border = black)")
        colorbar.set_ticks([-MAP_ENTER_SCORE, 0, MAP_ENTER_SCORE])
        colorbar.set_ticklabels([f"<=-{MAP_ENTER_SCORE}\nwall", "0\nfree",
                                 f">={MAP_ENTER_SCORE}\nweight"])
        # Exploration overlay: coarse cells, dark until explored. Edges follow the
        # map cells each coarse cell covers.
        cx_edges = [(1 + cx * EXPLORE_FINE_PER_CELL - X_ZERO - 0.5) * CELL_SIZE_M
                    for cx in range(EXPLORE_W)] + [(GRID_W - 2 - X_ZERO + 0.5) * CELL_SIZE_M]
        cy_edges = [(1 + cy * EXPLORE_FINE_PER_CELL - Y_ZERO - 0.5) * CELL_SIZE_M
                    for cy in range(EXPLORE_H)] + [(GRID_H - 2 - Y_ZERO + 0.5) * CELL_SIZE_M]
        self.explore_mesh = self.ax.pcolormesh(
            cx_edges, cy_edges, self._unexplored(), shading="flat", zorder=2,
            cmap=ListedColormap([(0.0, 0.0, 0.0, 0.0), UNEXPLORED_SHADE]),
            norm=Normalize(vmin=0, vmax=1),
        )
        self.path_line, = self.ax.plot([], [], "-", color=PATH_COLOR, linewidth=1.5,
                                       label="planned path", zorder=3)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title("Grid confidence")
        self.pose_dot, = self.ax.plot([], [], "o", color=POSE_COLOR, markeredgecolor="black",
                                      markersize=7, label="pose", zorder=4)
        # Heading arrow from the pose dot, fixed length in metres.
        self.heading_arrow = self.ax.quiver(
            [0.0], [0.0], [0.0], [0.0], color=POSE_COLOR, edgecolor="black", linewidth=0.5,
            angles="xy", scale_units="xy", scale=1, width=0.005, zorder=3,
        )
        self.heading_arrow.set_visible(False)
        self.target_dot, = self.ax.plot([], [], "o", color=PATH_COLOR,
                                        markersize=10, label="goal", zorder=4)
        self.ax.legend(handles=[self.pose_dot, self.target_dot, self.path_line,
                                Patch(facecolor=UNEXPLORED_SHADE, label="unexplored")],
                       loc="upper right")
        self.info_text = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes, va="top",
            fontsize=9, family="monospace",
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"),
        )
        self.connection_text = self.fig.text(0.08, 0.025, "", fontsize=9)
        self.fig.canvas.mpl_connect("close_event", self.close)

    def _display_scores(self):
        # Clamp to the threshold so confirmed cells are solid; borders stay below range (black).
        clipped = np.clip(self.scores, -MAP_ENTER_SCORE, MAP_ENTER_SCORE)
        return np.where(self.scores == -101, -101, clipped)

    def _unexplored(self):
        return (~self.explored).astype(float)

    def _clear_state(self):
        self.explored.fill(False)
        self.path = None
        self.grid.fill(CELL_FREE)
        self.scores.fill(0)
        self.scores[0, :] = self.scores[-1, :] = -101
        self.scores[:, 0] = self.scores[:, -1] = -101
        self.grid[0, :] = self.grid[-1, :] = CELL_BORDER
        self.grid[:, 0] = self.grid[:, -1] = CELL_BORDER
        self.pose = self.target = self.tel = self.tofs = None
        self.cell_updates = self.telemetry_count = self.bad_lines = 0

    def _close_port(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except SERIAL_ERRORS:
                pass
        self.ser = None

    def _disconnect(self, exc):
        self._close_port()
        self._buf = ""  # Never join fragments from separate connections.
        self.status = f"Disconnected from {self.port}; retrying"
        message = str(exc)
        if message != self.last_error:
            print(f"{self.status}: {message}", flush=True)
        self.last_error = message
        self.next_connect = time.monotonic() + 0.5

    def _connect(self):
        if time.monotonic() < self.next_connect:
            return False
        try:
            self.ser = serial.Serial(
                self.port, BAUD_RATE, timeout=0, write_timeout=0.5
            )
            self.connected_at = time.monotonic()
            self.last_rx = None
            self.last_error = ""
            self.status = f"Connected to {self.port}; waiting for data"
            print(self.status, flush=True)
            if self.reset_pending:
                # Discard OLD data only, before requesting restart.
                self.ser.reset_input_buffer()
                self._buf = ""
                self._clear_state()
                # Consume the request BEFORE writing: a write error may mean
                # the device already reset. Retrying R could cause a loop.
                self.reset_pending = False
                self.status = "Restart requested; waiting for robot / USB"
                print(self.status, flush=True)
                if self.ser.write(b"R") != 1:
                    raise serial.SerialException("Restart byte was not written")
                # No flush is needed here; subsequent read errors also trigger
                # reconnection if the board resets immediately after write.
            else:
                # Cells are only sent when they change, so ask for the whole
                # current map rather than showing just what changes from now.
                self._clear_state()
                if self.ser.write(b"M") != 1:
                    raise serial.SerialException("Map request byte was not written")
            return True
        except SERIAL_ERRORS as exc:
            self._disconnect(exc)
            return False

    def _handle_line(self, line):
        line = line.strip()
        if not line:
            return
        if self.debug:
            print(line, flush=True)
        if line == "=== BOOT ===" or line.startswith("=== ROUND START"):
            self._clear_state()  # fresh map and exploration either way
            self.last_message = line
            return
        if line == "V_RESET":
            self.explored.fill(False)
            return
        parts = line.split(",")
        tag = parts[0]
        try:
            if tag == "Q" and len(parts) == 5:
                gx, gy, score, label = map(int, parts[1:])
                if not (0 <= gx < GRID_W and 0 <= gy < GRID_H and -101 <= score <= 100
                        and label in (CELL_BORDER, CELL_FREE, CELL_OBSTACLE, CELL_WEIGHT)
                        and (score == -101) == (label == CELL_BORDER)):
                    raise ValueError("invalid confidence cell")
                self.scores[gy, gx] = score
                self.grid[gy, gx] = label
                self.cell_updates += 1
            elif tag == "TEL":
                tel = dict(part.split("=", 1) for part in parts[1:])
                pose = tuple(float(tel[k]) for k in ("x", "y", "th"))
                if not all(np.isfinite(pose)):
                    raise ValueError("non-finite coordinates")
                self.pose = (pose[0], pose[1], np.radians(pose[2]))
                self.tel = tel
            elif tag == "TOF":
                # Labelled mm readings in firmware order, e.g. LST=812, "-" = none.
                tofs = [tuple(part.split("=", 1)) for part in parts[1:]]
                if not tofs or not all(len(t) == 2 for t in tofs):
                    raise ValueError("invalid ToF field")
                self.tofs = tofs
            elif tag == "V" and len(parts) == 3:
                cx, cy = map(int, parts[1:])
                if not (0 <= cx < EXPLORE_W and 0 <= cy < EXPLORE_H):
                    raise ValueError("invalid explored cell")
                self.explored[cy, cx] = True
            elif tag == "PATH" and len(parts) >= 3 and len(parts) % 2 == 1:
                values = list(map(float, parts[1:]))
                if not all(np.isfinite(values)):
                    raise ValueError("non-finite path")
                self.path = (values[0::2], values[1::2])
            elif tag == "T" and len(parts) == 3:
                values = tuple(map(float, parts[1:]))
                if not all(np.isfinite(values)):
                    raise ValueError("non-finite coordinates")
                self.target = values
            else:
                self.last_message = line[:160]
                return
            self.telemetry_count += 1
        except (ValueError, OverflowError, KeyError):
            self.bad_lines += 1

    def _read_serial(self):
        if self.ser is None and not self._connect():
            return
        try:
            # Limit work per animation frame so a busy stream cannot freeze UI.
            n = min(self.ser.in_waiting, 65536)
            if n:
                data = self.ser.read(n)
                if data:
                    self.last_rx = time.monotonic()
                    self.status = f"Receiving from {self.port}"
                    self._buf += data.decode("utf-8", errors="replace")
        except SERIAL_ERRORS as exc:
            self._disconnect(exc)
            return
        for _ in range(2000):
            if "\n" not in self._buf:
                break
            line, self._buf = self._buf.split("\n", 1)
            self._handle_line(line)
        if len(self._buf) > 262144:
            self._buf = ""
            self.last_message = "Serial backlog/unterminated data discarded"

    def update(self, _frame):
        if self.closed:
            return ()
        self._read_serial()
        self.mesh.set_array(self._display_scores().ravel())
        self.explore_mesh.set_array(self._unexplored().ravel())
        self.path_line.set_data(*(self.path if self.path is not None else ([], [])))
        for dot, position in ((self.pose_dot, self.pose),
                              (self.target_dot, self.target)):
            dot.set_data([position[0]], [position[1]]) if position is not None else dot.set_data([], [])
        if self.pose is not None:
            x, y, theta = self.pose
            self.heading_arrow.set_offsets([[x, y]])
            self.heading_arrow.set_UVC([HEADING_ARROW_M * np.cos(theta)],
                                       [HEADING_ARROW_M * np.sin(theta)])
        self.heading_arrow.set_visible(self.pose is not None)
        lines = []
        if self.tel is not None:
            t = self.tel.get
            left, _, right = t("motors", "?/?").partition("/")
            lines += [
                f"state:   {t('state', '?')}   collect: {t('collect', '?')}",
                f"pose:    x={t('x')} y={t('y')} th={t('th')} deg",
                f"motors:  L={left:>4} R={right:>4}",
                f"catch:   ind={t('ind')} ind_det={t('ind_det')} ir_det={t('ir_det')}",
                f"imu cal: {t('cal', '?')} (sys/gyro/accel/mag, 3 = good)",
            ]
        if self.tofs:
            cells = [f"{name}={value:>4}" for name, value in self.tofs]
            half = (len(cells) + 1) // 2
            lines.append("tof mm:  " + "  ".join(cells[:half]))
            lines.append("         " + "  ".join(cells[half:]))
        self.info_text.set_text("\n".join(lines))
        status = self.status
        if self.ser is not None:
            since = self.last_rx if self.last_rx is not None else self.connected_at
            age = time.monotonic() - since
            if age > 3:
                status += f" | No incoming data for {age:.1f}s"
        self.connection_text.set_text(
            f"{status}\nTelemetry: {self.telemetry_count} | Cell updates: "
            f"{self.cell_updates} | Invalid records: {self.bad_lines}\n"
            f"Robot: {self.last_message}"
        )
        return (self.mesh, self.explore_mesh, self.path_line, self.pose_dot, self.heading_arrow,
                self.target_dot, self.info_text, self.connection_text)

    def close(self, _event=None):
        self.closed = True
        if self.anim is not None and self.anim.event_source is not None:
            self.anim.event_source.stop()
        self._close_port()

    def run(self):
        self.anim = FuncAnimation(
            self.fig, self.update, interval=100, blit=False,
            cache_frame_data=False,
        )
        try:
            plt.show()
        finally:
            self.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", help="Serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--no-reset", action="store_true", help="Connect without sending R")
    parser.add_argument("--debug", action="store_true", help="Print all received lines")
    args = parser.parse_args()
    Visualiser(args.port, reset=not args.no_reset, debug=args.debug).run()


if __name__ == "__main__":
    main()
