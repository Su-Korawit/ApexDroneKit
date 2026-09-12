"""
production.py - Drag-control GUI for the APEX GD-149 drone.

Two axes only: X = forward/backward, Y = altitude (up/down).
  Planning mode - drag to draw a path, press Run to fly it.
  Realtime mode - drag a joystick point; the drone follows live.

See docs/superpowers/specs/2026-09-09-drag-drone-gui-design.md for the design.
"""

from __future__ import annotations

import math

from apexdrone import DIST_MAX_CM, DIST_MIN_CM
from drone_worker import PlannedStep

LINK = "ble"                     # "sim" to practise without a drone, "ble" to fly for real
DRONE = "APEX_USART_218001"      # your drone's name from scan_drones.py

CANVAS_SIZE = 400
PX_PER_CM = 3
WAYPOINT_MIN_PX = 20            # minimum drag distance before a new planning waypoint is recorded
JOYSTICK_RADIUS_PX = 120
JOG_INTERVAL_MS = 150
JOG_DURATION_S = 0.15
JOG_MIN_POWER = 15              # offsets smaller than this percent of the radius are ignored
JOG_MAX_POWER = 100

GRID_STEP_CM = 10               # one grid cell, and the drone's smallest move
GRID_STEP_PX = GRID_STEP_CM * PX_PER_CM
GRID_MAJOR_EVERY = 3            # label every third line, so every 30cm
HANDLE_HIT_PX = 12              # grab radius; must stay under half a cell
CANVAS_CENTER = CANVAS_SIZE // 2
GRID_STEPS_EACH_WAY = CANVAS_CENTER // GRID_STEP_PX
GRID_MIN_PX = CANVAS_CENTER - GRID_STEPS_EACH_WAY * GRID_STEP_PX
GRID_MAX_PX = CANVAS_CENTER + GRID_STEPS_EACH_WAY * GRID_STEP_PX


# ---------------------------------------------------------------------------
# Pure conversion helpers - no tkinter, no drone, easy to unit test
# ---------------------------------------------------------------------------

def path_to_commands(points: list[tuple[float, float]],
                      px_per_cm: float) -> list[tuple[str, float]]:
    """Convert a drawn path (canvas points) into (direction, distance_cm)
    commands for Drone.forward()/back()/up()/down().

    Canvas y grows downward; altitude grows upward, so the y delta is
    inverted. Movement below DIST_MIN_CM keeps accumulating with the next
    segment on that axis; a leftover below DIST_MIN_CM at the end of the
    path is dropped rather than sent - it is too small to be worth a command.
    """
    commands: list[tuple[str, float]] = []
    acc_x = 0.0
    acc_y = 0.0
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        acc_x += (x2 - x1) / px_per_cm
        acc_y += (y1 - y2) / px_per_cm
        if abs(acc_x) >= DIST_MIN_CM:
            commands.extend(_split_distance("forward" if acc_x > 0 else "back", abs(acc_x)))
            acc_x = 0.0
        if abs(acc_y) >= DIST_MIN_CM:
            commands.extend(_split_distance("up" if acc_y > 0 else "down", abs(acc_y)))
            acc_y = 0.0
    return commands


def _split_distance(direction: str, distance_cm: float) -> list[tuple[str, float]]:
    chunks: list[tuple[str, float]] = []
    remaining = distance_cm
    while remaining > DIST_MAX_CM:
        chunks.append((direction, float(DIST_MAX_CM)))
        remaining -= DIST_MAX_CM
    if remaining >= DIST_MIN_CM:
        chunks.append((direction, remaining))
    return chunks


def offset_to_moves(dx_px: float, dy_px: float, radius_px: float) -> list[tuple[str, int]]:
    """Convert a joystick offset from center into (direction, power) pairs
    for Drone.move(). Offsets smaller than JOG_MIN_POWER percent of the
    radius are ignored so releasing near the center does not jitter."""
    moves: list[tuple[str, int]] = []
    power_x = round(min(JOG_MAX_POWER, abs(dx_px) / radius_px * 100))
    if power_x >= JOG_MIN_POWER:
        moves.append(("forward" if dx_px > 0 else "back", power_x))
    power_y = round(min(JOG_MAX_POWER, abs(dy_px) / radius_px * 100))
    if power_y >= JOG_MIN_POWER:
        moves.append(("up" if dy_px < 0 else "down", power_y))
    return moves


def commands_to_steps(drone, commands: list[tuple[str, float]]) -> list[PlannedStep]:
    """Turn path_to_commands() output into queueable PlannedStep objects.

    Each (direction, distance_cm) pair becomes a call to the matching
    Drone.forward()/back()/up()/down() method with its `dist` keyword - the
    glue between the mouse and the motors, kept out of the GUI so it can be
    unit tested without tkinter or a drone.
    """
    return [
        PlannedStep(getattr(drone, direction), kwargs={"dist": dist},
                    label=f"{direction} {dist:.0f}cm")
        for direction, dist in commands
    ]


def snap_to_grid(x: float, y: float) -> tuple[int, int]:
    """Snap a canvas point to the nearest 10cm grid intersection.

    Every leg between snapped nodes then has axis deltas that are whole
    multiples of DIST_MIN_CM, so path_to_commands emits each one exactly
    instead of holding a sub-minimum remainder it discards on the next emit.
    """
    def axis(value: float) -> int:
        steps = round((value - CANVAS_CENTER) / GRID_STEP_PX)
        return max(GRID_MIN_PX,
                   min(GRID_MAX_PX, CANVAS_CENTER + steps * GRID_STEP_PX))
    return axis(x), axis(y)


def grid_lines() -> list[tuple[int, int, bool]]:
    """Every grid line as (position_px, cm_from_centre, is_major).

    The geometry is identical for both axes, so the caller draws each entry
    once vertically and once horizontally.
    """
    return [(CANVAS_CENTER + step * GRID_STEP_PX,
             step * GRID_STEP_CM,
             step % GRID_MAJOR_EVERY == 0)
            for step in range(-GRID_STEPS_EACH_WAY, GRID_STEPS_EACH_WAY + 1)]


def commands_to_ghost_points(commands: list[tuple[str, float]],
                             start: tuple[float, float]) -> list[tuple[float, float]]:
    """Trace where a command list actually takes the drone, in canvas px.

    Each command moves one axis and they run one at a time, so a diagonal
    leg becomes two points: the drone flies the horizontal, then the
    vertical, never the diagonal that was drawn.
    """
    x, y = start
    points = [(x, y)]
    for direction, distance_cm in commands:
        px = distance_cm * PX_PER_CM
        if direction == "forward":
            x += px
        elif direction == "back":
            x -= px
        elif direction == "up":
            y -= px
        elif direction == "down":
            y += px
        points.append((x, y))
    return points


def find_hit(nodes: list[tuple[int, int]], x: float, y: float,
             radius: float = HANDLE_HIT_PX) -> tuple[str, int] | None:
    """What the user grabbed: ("end", index) for either end of the chain,
    ("leg", index) for the midpoint of the leg starting at that index, or None.

    Ends are checked first: on a one-cell leg the midpoint sits 15px from each
    node, so a radius under 15 keeps the two unambiguous.
    """
    if not nodes:
        return None
    for index in (0, len(nodes) - 1):
        node_x, node_y = nodes[index]
        if math.hypot(x - node_x, y - node_y) <= radius:
            return ("end", index)
    for index, (a, b) in enumerate(zip(nodes, nodes[1:])):
        if math.hypot(x - (a[0] + b[0]) / 2, y - (a[1] + b[1]) / 2) <= radius:
            return ("leg", index)
    return None


import tkinter as tk
from tkinter import ttk

from apexdrone import Drone
from drone_worker import DroneWorker


class DroneGUI:
    def __init__(self, root: tk.Tk, drone: Drone, worker: DroneWorker) -> None:
        self.root = root
        self.drone = drone
        self.worker = worker
        self.mode = tk.StringVar(value="planning")
        self.nodes: list[tuple[int, int]] = []
        self._drag: dict | None = None
        self.plan_running = False
        self._joystick_offset = (0, 0)
        self.jog_active = False
        self._jog_after_id: str | None = None

        self._build_widgets()
        self._poll_status()

    def _build_widgets(self) -> None:
        top = ttk.Frame(self.root)
        top.pack(fill="x", padx=8, pady=4)

        ttk.Button(top, text="Takeoff", command=self._on_takeoff).pack(side="left")
        ttk.Button(top, text="Land", command=self._on_land).pack(side="left", padx=4)
        ttk.Button(top, text="EMERGENCY STOP", command=self._on_emergency,
                   style="Emergency.TButton").pack(side="left", padx=12)

        ttk.Radiobutton(top, text="Planning", variable=self.mode, value="planning",
                         command=self._on_mode_change).pack(side="left", padx=12)
        ttk.Radiobutton(top, text="Realtime", variable=self.mode, value="realtime",
                         command=self._on_mode_change).pack(side="left")

        self.canvas = tk.Canvas(self.root, width=CANVAS_SIZE, height=CANVAS_SIZE,
                                 background="white")
        self.canvas.pack(padx=8, pady=4)
        self._draw_grid()

        self.canvas.bind("<ButtonPress-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)

        controls = ttk.Frame(self.root)
        controls.pack(fill="x", padx=8, pady=4)
        self.run_button = ttk.Button(controls, text="Run", command=self._on_run)
        self.run_button.pack(side="left")
        self.stop_button = ttk.Button(controls, text="Stop", command=self._on_stop_plan,
                                       state="disabled")
        self.stop_button.pack(side="left", padx=4)
        self.clear_button = ttk.Button(controls, text="Clear", command=self._on_clear)
        self.clear_button.pack(side="left")

        self.status_var = tk.StringVar(value="Connecting...")
        ttk.Label(self.root, textvariable=self.status_var).pack(fill="x", padx=8)

        self.log = tk.Listbox(self.root, height=8)
        self.log.pack(fill="both", expand=True, padx=8, pady=4)

        self._on_mode_change()

    def _draw_grid(self) -> None:
        for position, cm, major in grid_lines():
            colour = "#b0b0b0" if major else "#ececec"
            self.canvas.create_line(position, GRID_MIN_PX, position, GRID_MAX_PX,
                                     fill=colour, tags="grid")
            self.canvas.create_line(GRID_MIN_PX, position, GRID_MAX_PX, position,
                                     fill=colour, tags="grid")
            if major and cm != 0:
                self.canvas.create_text(position, GRID_MAX_PX + 8, text=f"{cm:+d}",
                                         font=("Helvetica", 7), fill="#808080",
                                         tags="grid")
                # Canvas y grows downward, altitude upward, so the sign flips.
                self.canvas.create_text(GRID_MIN_PX - 12, position, text=f"{-cm:+d}",
                                         font=("Helvetica", 7), fill="#808080",
                                         tags="grid")
        self.canvas.create_text(GRID_MIN_PX - 12, GRID_MAX_PX + 8, text="cm",
                                 font=("Helvetica", 7), fill="#808080", tags="grid")

    def _on_takeoff(self) -> None:
        self.worker.enqueue(self.drone.takeoff, label="takeoff")

    def _on_land(self) -> None:
        self.worker.enqueue(self.drone.land, label="land")

    def _on_emergency(self) -> None:
        self.worker.emergency_stop()
        # Cutting the motors is not enough on its own: whatever was queued
        # would resume flying once the emergency window clears.
        self.worker.flush()
        self._cancel_jog_loop()

    def _on_mode_change(self) -> None:
        planning = self.mode.get() == "planning"
        state = "normal" if planning else "disabled"
        self.run_button.configure(state=state)
        self.clear_button.configure(state=state)
        self._on_clear()
        self._cancel_jog_loop()
        self._reset_joystick()

    def _on_canvas_press(self, event: tk.Event) -> None:
        if self.mode.get() != "planning":
            self._cancel_jog_loop()  # never run two jog chains at once
            self.jog_active = True
            self._update_joystick(event.x, event.y)
            self._jog_tick()
            return
        if self.plan_running:
            return
        hit = find_hit(self.nodes, event.x, event.y)
        point = snap_to_grid(event.x, event.y)
        if hit is None and self.nodes:
            self.log.insert("end", "Drag from an end point to extend, "
                                   "or a leg's middle to bend it.")
            return
        if hit is None:
            self._drag = {"kind": "first", "anchor": point, "preview": point}
        elif hit[0] == "end":
            self._drag = {"kind": "extend", "at": hit[1], "preview": point}
        else:
            self._drag = {"kind": "bend", "leg": hit[1], "preview": point}
        self._redraw_path()

    def _on_canvas_drag(self, event: tk.Event) -> None:
        if self.mode.get() != "planning":
            self._update_joystick(event.x, event.y)
            return
        if self._drag is None:
            return
        self._drag["preview"] = snap_to_grid(event.x, event.y)
        self._redraw_path()

    def _on_canvas_release(self, event: tk.Event) -> None:
        if self.mode.get() != "planning":
            self._cancel_jog_loop()
            # Drop jog commands queued but not yet started, so the drone stops
            # shortly after the mouse does instead of flying out the backlog.
            self.worker.flush()
            self._reset_joystick()
            return
        if self._drag is None:
            return
        point = snap_to_grid(event.x, event.y)
        kind = self._drag["kind"]
        if kind == "first":
            if point != self._drag["anchor"]:
                self.nodes = [self._drag["anchor"], point]
        elif kind == "extend":
            if point not in self.nodes:
                if self._drag["at"] == 0:
                    self.nodes.insert(0, point)
                else:
                    self.nodes.append(point)
        else:
            leg = self._drag["leg"]
            if point not in (self.nodes[leg], self.nodes[leg + 1]):
                self.nodes.insert(leg + 1, point)
        self._drag = None
        self._redraw_path()

    def _redraw_path(self) -> None:
        self.canvas.delete("path")
        if self.nodes:
            ghost = commands_to_ghost_points(
                path_to_commands(self.nodes, PX_PER_CM), self.nodes[0])
            if len(ghost) > 1:
                self.canvas.create_line(*[c for point in ghost for c in point],
                                         fill="#c9c9c9", width=6, tags="path")
            self.canvas.create_line(*[c for node in self.nodes for c in node],
                                     fill="blue", width=2, tags="path")
            for index, (x, y) in enumerate(self.nodes):
                is_end = index in (0, len(self.nodes) - 1)
                self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5,
                                         fill="red" if is_end else "white",
                                         outline="blue", tags="path")
            for a, b in zip(self.nodes, self.nodes[1:]):
                mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
                self.canvas.create_rectangle(mx - 3, my - 3, mx + 3, my + 3,
                                              fill="orange", outline="", tags="path")
        if self._drag is not None:
            preview = self._drag["preview"]
            if self._drag["kind"] == "bend":
                leg = self._drag["leg"]
                anchors = (self.nodes[leg], self.nodes[leg + 1])
            elif self._drag["kind"] == "extend":
                anchors = (self.nodes[self._drag["at"]],)
            else:
                anchors = (self._drag["anchor"],)
            for anchor in anchors:
                self.canvas.create_line(*anchor, *preview, fill="gray",
                                         dash=(3, 3), tags="path")

    def _update_joystick(self, x: float, y: float) -> None:
        center = CANVAS_SIZE // 2
        dx = max(-JOYSTICK_RADIUS_PX, min(JOYSTICK_RADIUS_PX, x - center))
        dy = max(-JOYSTICK_RADIUS_PX, min(JOYSTICK_RADIUS_PX, y - center))
        self._joystick_offset = (dx, dy)
        self._draw_joystick(center + dx, center + dy)

    def _reset_joystick(self) -> None:
        self._joystick_offset = (0, 0)
        center = CANVAS_SIZE // 2
        self._draw_joystick(center, center)

    def _draw_joystick(self, x: float, y: float) -> None:
        self.canvas.delete("joystick")
        self.canvas.create_oval(x - 8, y - 8, x + 8, y + 8, fill="orange", tags="joystick")

    def _cancel_jog_loop(self) -> None:
        """Stop the realtime jog loop: no new ticks, and no stale scheduled
        tick left alive to start a second chain on the next press."""
        self.jog_active = False
        if self._jog_after_id is not None:
            self.root.after_cancel(self._jog_after_id)
            self._jog_after_id = None

    def _jog_tick(self) -> None:
        self._jog_after_id = None
        if not self.jog_active:
            return
        # Only queue a new jog once the previous one has been consumed: a
        # move(seconds=0.15) takes far longer than JOG_INTERVAL_MS to run, so
        # queueing every tick would build a backlog the drone flies out long
        # after the mouse stops moving.
        if self.worker.is_idle():
            dx, dy = self._joystick_offset
            for direction, power in offset_to_moves(dx, dy, JOYSTICK_RADIUS_PX):
                self.worker.enqueue(self.drone.move, direction, seconds=JOG_DURATION_S,
                                     power=power, label=f"jog {direction} {power}")
        self._jog_after_id = self.root.after(JOG_INTERVAL_MS, self._jog_tick)

    def _on_run(self) -> None:
        if self.mode.get() != "planning" or len(self.path_points) < 2:
            self.log.insert("end", "Draw a path before pressing Run.")
            return
        commands = path_to_commands(self.path_points, PX_PER_CM)
        if not commands:
            self.log.insert("end", "Path too short to produce any commands.")
            return
        self.worker.enqueue_plan(commands_to_steps(self.drone, commands))
        self.plan_running = True
        self.stop_button.configure(state="normal")
        self.run_button.configure(state="disabled")
        self.clear_button.configure(state="disabled")

    def _on_stop_plan(self) -> None:
        self.worker.stop_plan()
        self._finish_plan()

    def _finish_plan(self) -> None:
        self.plan_running = False
        self.stop_button.configure(state="disabled")
        if self.mode.get() == "planning":
            self.run_button.configure(state="normal")
            self.clear_button.configure(state="normal")

    def _on_clear(self) -> None:
        self.canvas.delete("path")
        self.path_points = []
        self.drawing = False

    def _poll_status(self) -> None:
        for line in self.worker.drain_status():
            self.log.insert("end", line)
            self.log.yview_moveto(1.0)
        if self.plan_running and self.worker.is_idle():
            self._finish_plan()
        info = self.drone.status()
        if info is not None:
            self.status_var.set(
                f"{info['battery_volt']}V  {info['fly_status']}  {info['altitude_cm']}cm")
        elif not self.plan_running:
            self.status_var.set("Connected (no sensor data on this link).")
        self.root.after(100, self._poll_status)


def main() -> None:
    root = tk.Tk()
    root.title("APEX Drone - Drag Control")

    style = ttk.Style()
    style.configure("Emergency.TButton", foreground="red")

    drone = Drone(LINK, device_name=DRONE or None).connect()
    worker = DroneWorker(drone)
    worker.start()

    DroneGUI(root, drone, worker)

    def on_close() -> None:
        worker.close()
        drone.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
