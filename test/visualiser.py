#!/usr/bin/env python3
"""Live robot grid. Install: pip install pyserial matplotlib numpy

Run: python visualiser.py COM5
     python visualiser.py COM5 --no-reset
     python visualiser.py COM5 --debug

Sends R once per launch by default (bench testing: resets robot state).
Reconnects to the same port after USB reset, without sending R again.
Firmware must call check_serial_commands() at the start of loop().
For startup-only map updates, firmware must wait for the serial client
before map_init(), or provide a full-map resend command. Python cannot
recover updates sent while USB was disconnected.
"""

import argparse
import re
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import BoundaryNorm, ListedColormap
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
CELL_VALUES = [-2, -1, 0, 1, 2, 3]
CELL_COLORS = ["black", "grey", "grey", "red", "grey", "gold"]
STATE_NAMES = ["NAVIGATION", "APPROACH_VERIFY", "APPROACH_WEIGHT",
               "SCANNING", "RETURN_HOME", "DROP_OFF"]
NUMBER = r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
POSE_DEBUG_RE = re.compile(
    r"\bx=\s*" + NUMBER + r"\s+y=\s*" + NUMBER + r"\s+theta_deg=\s*" + NUMBER
)
SERIAL_ERRORS = (serial.SerialException, OSError)


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
        self._clear_state()

        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.fig.subplots_adjust(bottom=0.18)
        cmap = ListedColormap(CELL_COLORS)
        norm = BoundaryNorm(np.arange(-2.5, 4.0, 1.0), cmap.N)
        # Firmware coordinates name cell centres.
        x_edges = (np.arange(GRID_W + 1) - X_ZERO - 0.5) * CELL_SIZE_M
        y_edges = (np.arange(GRID_H + 1) - Y_ZERO - 0.5) * CELL_SIZE_M
        self.mesh = self.ax.pcolormesh(
            x_edges, y_edges, self.grid, cmap=cmap, norm=norm,
            edgecolors="black", linewidth=0.4, shading="flat",
        )
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title("Grid classifications")
        self.pose_dot, = self.ax.plot([], [], "o", color="#145A14",
                                      markersize=4, label="pose")
        self.target_dot, = self.ax.plot([], [], "o", color="#FF8C00",
                                        markersize=10, label="target")
        self.ax.legend(handles=[
            Patch(facecolor="black", label="border"),
            Patch(facecolor="grey", label="free / unconfirmed"),
            Patch(facecolor="red", label="wall"),
            Patch(facecolor="gold", label="weight"),
            self.pose_dot, self.target_dot,
        ], loc="upper right")
        self.info_text = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes, va="top",
            fontsize=9, family="monospace",
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"),
        )
        self.connection_text = self.fig.text(0.08, 0.025, "", fontsize=9)
        self.fig.canvas.mpl_connect("close_event", self.close)

    def _clear_state(self):
        self.grid.fill(CELL_FREE)
        self.scores.fill(0)
        self.scores[0, :] = self.scores[-1, :] = -101
        self.scores[:, 0] = self.scores[:, -1] = -101
        self.grid[0, :] = self.grid[-1, :] = CELL_BORDER
        self.grid[:, 0] = self.grid[:, -1] = CELL_BORDER
        self.pose = self.target = self.motors = self.state_info = None
        self.diag = None
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
        if line == "=== BOOT ===":
            self._clear_state()
            self.last_message = line
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
            elif tag == "C" and len(parts) == 4:
                gx, gy, value = map(int, parts[1:])
                if not (0 <= gx < GRID_W and 0 <= gy < GRID_H
                        and value in CELL_VALUES):
                    raise ValueError("invalid cell")
                self.grid[gy, gx] = value
                self.scores[gy, gx] = {CELL_BORDER: -101, CELL_OBSTACLE: -100, CELL_WEIGHT: 100}.get(value, 0)
                self.cell_updates += 1
            elif tag in ("P", "T") and len(parts) == (4 if tag == "P" else 3):
                values = tuple(map(float, parts[1:]))
                if not all(np.isfinite(values)):
                    raise ValueError("non-finite coordinates")
                if tag == "P":
                    self.pose = values
                else:
                    self.target = values
            elif tag == "M" and len(parts) == 3:
                self.motors = tuple(map(int, parts[1:]))
            elif tag == "S" and len(parts) == 3:
                self.state_info = tuple(map(int, parts[1:]))
            elif tag == "D":
                self.diag = line[2:]
            else:
                match = POSE_DEBUG_RE.search(line)
                if match:
                    x, y, theta_deg = map(float, match.groups())
                    self.pose = (x, y, np.radians(theta_deg))
                else:
                    self.last_message = line[:160]
                    return
            self.telemetry_count += 1
        except (ValueError, OverflowError):
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
        self.mesh.set_array(self.grid.ravel())
        for dot, position in ((self.pose_dot, self.pose),
                              (self.target_dot, self.target)):
            dot.set_data([position[0]], [position[1]]) if position is not None else dot.set_data([], [])
        lines = []
        if self.motors is not None:
            lines.append(f"motors: L={self.motors[0]:>4} R={self.motors[1]:>4}")
        if self.state_info is not None:
            state, busy = self.state_info
            name = STATE_NAMES[state] if 0 <= state < len(STATE_NAMES) else f"?{state}"
            lines.append(f"state: {name}  busy={bool(busy)}")
        if self.diag is not None:
            lines.append(f"sensors: {self.diag}")
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
        return self.mesh, self.pose_dot, self.target_dot, self.info_text, self.connection_text

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
