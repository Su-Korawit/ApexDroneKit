"""
production.py - Drag-control GUI for the APEX GD-149 drone.

Two axes only: X = forward/backward, Y = altitude (up/down).
  Planning mode - drag to draw a path, press Run to fly it.
  Realtime mode - drag a joystick point; the drone follows live.

See docs/superpowers/specs/2026-09-09-drag-drone-gui-design.md for the design.
"""

from __future__ import annotations

from apexdrone import DIST_MAX_CM, DIST_MIN_CM

LINK = "ble"                     # "sim" to practise without a drone, "ble" to fly for real
DRONE = "APEX_USART_751F02"      # your drone's name from scan_drones.py

CANVAS_SIZE = 400
PX_PER_CM = 3
WAYPOINT_MIN_PX = 20            # minimum drag distance before a new planning waypoint is recorded
JOYSTICK_RADIUS_PX = 120
JOG_INTERVAL_MS = 150
JOG_DURATION_S = 0.15
JOG_MIN_POWER = 15              # offsets smaller than this percent of the radius are ignored
JOG_MAX_POWER = 100


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


import tkinter as tk
from tkinter import ttk

from apexdrone import Drone
from drone_worker import DroneWorker, PlannedStep


class DroneGUI:
    def __init__(self, root: tk.Tk, drone: Drone, worker: DroneWorker) -> None:
        self.root = root
        self.drone = drone
        self.worker = worker
        self.mode = tk.StringVar(value="planning")
        self.path_points: list[tuple[float, float]] = []
        self.drawing = False
        self.plan_running = False

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
        center = CANVAS_SIZE // 2
        self.canvas.create_line(0, center, CANVAS_SIZE, center, fill="#ccc")
        self.canvas.create_line(center, 0, center, CANVAS_SIZE, fill="#ccc")

    def _on_takeoff(self) -> None:
        self.worker.enqueue(self.drone.takeoff, label="takeoff")

    def _on_land(self) -> None:
        self.worker.enqueue(self.drone.land, label="land")

    def _on_emergency(self) -> None:
        self.worker.emergency_stop()

    def _on_mode_change(self) -> None:
        planning = self.mode.get() == "planning"
        state = "normal" if planning else "disabled"
        self.run_button.configure(state=state)
        self.clear_button.configure(state=state)
        self._on_clear()

    # Planning/Realtime button handlers are added in later tasks.
    def _on_run(self) -> None:
        pass

    def _on_stop_plan(self) -> None:
        pass

    def _on_clear(self) -> None:
        self.canvas.delete("path")
        self.path_points = []
        self.drawing = False

    def _poll_status(self) -> None:
        for line in self.worker.drain_status():
            self.log.insert("end", line)
            self.log.yview_moveto(1.0)
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
