# Drag-Control Drone GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `production.py`, a tkinter GUI that flies the APEX GD-149 (via `apexdrone.py`) along two axes only — X (forward/back) and Y (altitude up/down) — with a Planning mode (drag to draw a path, press Run) and a Realtime mode (drag a joystick point, the drone follows live).

**Architecture:** A new `drone_worker.py` runs drone commands on a background thread through a single queue, so tkinter's main loop never blocks and Emergency Stop can always fire immediately. `production.py` holds pure, unit-testable conversion functions (drawn path → distance commands, joystick offset → move commands) plus a `DroneGUI` class that wires tkinter widgets to `DroneWorker` and those pure functions.

**Tech Stack:** Python 3.13, tkinter (standard library), `apexdrone.Drone` (existing library, not modified), `unittest` (standard library — this repo has no pytest dependency at the root level).

**Spec:** `docs/superpowers/specs/2026-09-09-drag-drone-gui-design.md`

## Global Constraints

- Only X (forward/back) and Y (altitude up/down) axes are controlled — no left/right strafing, no rotation.
- tkinter only. No new third-party dependencies.
- `production.py` must not open a window merely by being imported — GUI startup happens only inside `if __name__ == "__main__":` (`main()`), so its pure functions stay importable by tests.
- Distances sent to `Drone.forward()/back()/up()/down()` must respect `apexdrone.DIST_MIN_CM` (10) and `apexdrone.DIST_MAX_CM` (180).
- Emergency Stop must bypass the command queue entirely and stay clickable no matter what else is queued or running.
- Connection constants at the top of `production.py`: `LINK = "ble"`, `DRONE = "APEX_USART_751F02"` — the same real drone `console.py` is already configured for in this repo.
- Tests run with the standard library: `python -m unittest <module> -v` from the project root (no pytest install needed or expected). These are unaffected by the real-drone link, since Tasks 1-2 test pure functions and a `FakeDrone`, never a live connection.

**Safety — this plan flies a real drone.** Before any manual verification step in Tasks 3-5:
fly in an open space at least 3 m clear of people/objects, with 2 m of ceiling clearance
(`takeoff()` climbs to ~110 cm). Closing the window lands the drone but is **not** an
emergency stop — if the motors won't stop, unplug the battery (`README.md` mentions an
`emergency_stop.py` script, but it does not exist yet in this repo, so unplugging is the
real fallback). If testing on a bench with propellers off, use the GUI's Emergency Stop
button instead of Land, since `land()` never registers as airborne with no propellers.
See `README.md`'s "Safety - read before flying" section for the full list.

---

### Task 1: `drone_worker.py` — background command queue

**Files:**
- Create: `drone_worker.py`
- Test: `test_drone_worker.py`

**Interfaces:**
- Produces:
  - `PlannedStep(fn: Callable, args: tuple = (), kwargs: dict = {}, label: str = "")` — a dataclass.
  - `DroneWorker(drone)` with methods:
    - `start() -> None`
    - `enqueue(fn, *args, label: str | None = None, **kwargs) -> None`
    - `enqueue_plan(steps: list[PlannedStep]) -> None`
    - `stop_plan() -> None`
    - `emergency_stop() -> None`
    - `drain_status() -> list[str]`
    - `is_idle() -> bool`
    - `close() -> None`

- [ ] **Step 1: Write the failing tests**

Create `test_drone_worker.py`:

```python
import threading
import time
import unittest

from drone_worker import DroneWorker, PlannedStep


class FakeDrone:
    def __init__(self):
        self.emergency_calls = 0

    def emergency_stop(self):
        self.emergency_calls += 1


def _wait_for(predicate, timeout=2.0, interval=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class DroneWorkerTests(unittest.TestCase):
    def setUp(self):
        self.drone = FakeDrone()
        self.worker = DroneWorker(self.drone)
        self.worker.start()

    def tearDown(self):
        self.worker.close()

    def test_enqueue_runs_in_order(self):
        calls = []
        self.worker.enqueue(calls.append, "a", label="step-a")
        self.worker.enqueue(calls.append, "b", label="step-b")
        self.assertTrue(_wait_for(lambda: calls == ["a", "b"]))

    def test_drain_status_reports_completed_step(self):
        self.worker.enqueue(lambda: None, label="noop")
        collected = []

        def has_status():
            collected.extend(self.worker.drain_status())
            return any("noop: done" in line for line in collected)

        self.assertTrue(_wait_for(has_status))

    def test_value_error_is_reported_not_raised(self):
        def boom():
            raise ValueError("dist must be between 10 and 180 cm, got 500")

        self.worker.enqueue(boom, label="forward")
        collected = []

        def has_status():
            collected.extend(self.worker.drain_status())
            return any("forward:" in line and "500" in line for line in collected)

        self.assertTrue(_wait_for(has_status))

    def test_stop_plan_drops_unstarted_steps(self):
        started = threading.Event()
        release = threading.Event()
        ran = []

        def slow_first():
            started.set()
            release.wait(timeout=2)
            ran.append("first")

        def second():
            ran.append("second")

        self.worker.enqueue_plan([
            PlannedStep(slow_first, label="first"),
            PlannedStep(second, label="second"),
        ])
        self.assertTrue(_wait_for(started.is_set))
        self.worker.stop_plan()
        release.set()
        self.assertTrue(_wait_for(lambda: "first" in ran, timeout=1))
        time.sleep(0.2)
        self.assertNotIn("second", ran)

    def test_emergency_stop_runs_even_while_queue_busy(self):
        blocker = threading.Event()
        started = threading.Event()

        def slow():
            started.set()
            blocker.wait(timeout=2)

        self.worker.enqueue(slow, label="slow")
        self.assertTrue(_wait_for(started.is_set))
        self.worker.emergency_stop()
        self.assertTrue(_wait_for(lambda: self.drone.emergency_calls == 1))
        blocker.set()

    def test_is_idle_reflects_busy_and_queue_state(self):
        gate = threading.Event()
        started = threading.Event()

        def blocking():
            started.set()
            gate.wait(timeout=2)

        self.assertTrue(self.worker.is_idle())
        self.worker.enqueue(blocking, label="blocking")
        self.assertTrue(_wait_for(started.is_set))
        self.assertFalse(self.worker.is_idle())
        gate.set()
        self.assertTrue(_wait_for(self.worker.is_idle))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m unittest test_drone_worker -v`
Expected: FAIL / ERROR — `ModuleNotFoundError: No module named 'drone_worker'`

- [ ] **Step 3: Implement `drone_worker.py`**

```python
"""
drone_worker.py - Runs Drone commands on a background thread so a tkinter
GUI never freezes while a command is in flight.

Commands are queued from the main thread and executed one at a time on a
dedicated worker thread. Results and errors come back as short strings on
a second queue that the GUI drains on a timer (see production.py).
Emergency stop bypasses both queues, so it fires immediately no matter
what else is queued or running.
"""

from __future__ import annotations

import queue
import threading
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class PlannedStep:
    fn: Callable[..., None]
    args: tuple = ()
    kwargs: dict = field(default_factory=dict)
    label: str = ""


class DroneWorker:
    def __init__(self, drone) -> None:
        self._drone = drone
        self._command_queue: "queue.Queue[tuple[int, PlannedStep]]" = queue.Queue()
        self._status_queue: "queue.Queue[str]" = queue.Queue()
        self._generation = 0
        self._busy = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def enqueue(self, fn: Callable[..., None], *args, label: str | None = None,
                **kwargs) -> None:
        label = label or getattr(fn, "__name__", "command")
        self._command_queue.put((self._generation, PlannedStep(fn, args, kwargs, label)))

    def enqueue_plan(self, steps: list[PlannedStep]) -> None:
        generation = self._generation
        for step in steps:
            self._command_queue.put((generation, step))

    def stop_plan(self) -> None:
        """Drop every not-yet-started queued step. A step already running
        finishes normally - it cannot be interrupted mid-command."""
        self._generation += 1
        self._status_queue.put("Plan stopped - remaining steps dropped.")

    def emergency_stop(self) -> None:
        """Calls drone.emergency_stop() on its own thread immediately,
        bypassing the command queue entirely."""
        def run() -> None:
            try:
                self._drone.emergency_stop()
                self._status_queue.put("EMERGENCY STOP sent.")
            except Exception as error:  # noqa: BLE001
                self._status_queue.put(f"Emergency stop error: {error}")
        threading.Thread(target=run, daemon=True).start()

    def drain_status(self) -> list[str]:
        """Non-blocking: return every status line queued since the last call."""
        lines = []
        while True:
            try:
                lines.append(self._status_queue.get_nowait())
            except queue.Empty:
                break
        return lines

    def is_idle(self) -> bool:
        return self._command_queue.empty() and not self._busy.is_set()

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                generation, step = self._command_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if generation != self._generation:
                continue  # dropped by stop_plan()
            self._busy.set()
            try:
                step.fn(*step.args, **step.kwargs)
                self._status_queue.put(f"{step.label}: done")
            except ValueError as error:
                self._status_queue.put(f"{step.label}: {error}")
            except Exception as error:  # noqa: BLE001
                self._status_queue.put(f"{step.label}: error - {error}")
            finally:
                self._busy.clear()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m unittest test_drone_worker -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add drone_worker.py test_drone_worker.py
git commit -m "$(cat <<'EOF'
Add DroneWorker background command queue

Runs drone commands on a dedicated thread so a future tkinter GUI never
blocks on a command, with generation-tagged plans (so Stop can drop
queued-but-not-started steps) and an emergency stop that bypasses the
queue entirely.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: `production.py` — pure path/joystick conversion helpers

**Files:**
- Create: `production.py`
- Test: `test_production_logic.py`

**Interfaces:**
- Consumes: `apexdrone.DIST_MIN_CM` (10), `apexdrone.DIST_MAX_CM` (180) — read directly, not modified.
- Produces:
  - `path_to_commands(points: list[tuple[float, float]], px_per_cm: float) -> list[tuple[str, float]]` — direction is one of `"forward"`, `"back"`, `"up"`, `"down"`.
  - `offset_to_moves(dx_px: float, dy_px: float, radius_px: float) -> list[tuple[str, int]]` — direction is one of `"forward"`, `"back"`, `"up"`, `"down"`; power is `0..100`.
  - Constants: `CANVAS_SIZE = 400`, `PX_PER_CM = 3`, `WAYPOINT_MIN_PX = 20`, `JOYSTICK_RADIUS_PX = 120`, `JOG_INTERVAL_MS = 150`, `JOG_DURATION_S = 0.15`, `JOG_MIN_POWER = 15`, `JOG_MAX_POWER = 100`, `LINK = "ble"`, `DRONE = "APEX_USART_751F02"`.

- [ ] **Step 1: Write the failing tests**

Create `test_production_logic.py`:

```python
import unittest

from production import path_to_commands, offset_to_moves


class PathToCommandsTests(unittest.TestCase):
    def test_single_forward_segment(self):
        points = [(0, 0), (60, 0)]  # 60 px right, no vertical movement
        self.assertEqual(path_to_commands(points, px_per_cm=3), [("forward", 20.0)])

    def test_backward_and_up_together(self):
        points = [(0, 100), (-30, 40)]  # left 30px = back 10cm; up 60px = up 20cm
        self.assertEqual(
            path_to_commands(points, px_per_cm=3),
            [("back", 10.0), ("up", 20.0)],
        )

    def test_segments_below_threshold_accumulate(self):
        points = [(0, 0), (5, 0), (10, 0), (40, 0)]
        commands = path_to_commands(points, px_per_cm=3)
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0][0], "forward")
        self.assertAlmostEqual(commands[0][1], 40 / 3, places=6)

    def test_long_segment_is_split_at_180cm(self):
        points = [(0, 0), (600, 0)]  # 600px / 3 = 200cm
        self.assertEqual(
            path_to_commands(points, px_per_cm=3),
            [("forward", 180.0), ("forward", 20.0)],
        )

    def test_trailing_remainder_below_min_is_dropped(self):
        points = [(0, 0), (549, 0)]  # 549px / 3 = 183cm -> 180 + 3 (3 < 10cm, dropped)
        self.assertEqual(path_to_commands(points, px_per_cm=3), [("forward", 180.0)])

    def test_too_short_path_produces_no_commands(self):
        points = [(0, 0), (5, 0)]  # 5px / 3 = 1.67cm, well under the 10cm minimum
        self.assertEqual(path_to_commands(points, px_per_cm=3), [])


class OffsetToMovesTests(unittest.TestCase):
    def test_no_movement_below_threshold(self):
        self.assertEqual(offset_to_moves(2, 2, radius_px=100), [])

    def test_pure_forward(self):
        self.assertEqual(offset_to_moves(50, 0, radius_px=100), [("forward", 50)])

    def test_pure_back(self):
        self.assertEqual(offset_to_moves(-50, 0, radius_px=100), [("back", 50)])

    def test_pure_up(self):
        self.assertEqual(offset_to_moves(0, -80, radius_px=100), [("up", 80)])

    def test_diagonal_forward_and_down(self):
        self.assertEqual(
            offset_to_moves(40, 60, radius_px=100),
            [("forward", 40), ("down", 60)],
        )

    def test_power_is_clamped_to_100(self):
        self.assertEqual(offset_to_moves(150, 0, radius_px=100), [("forward", 100)])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m unittest test_production_logic -v`
Expected: FAIL / ERROR — `ModuleNotFoundError: No module named 'production'`

- [ ] **Step 3: Implement the pure helpers in `production.py`**

```python
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


if __name__ == "__main__":
    pass  # the GUI (main()) is added in a later task
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m unittest test_production_logic -v`
Expected: PASS (12 tests)

- [ ] **Step 5: Commit**

```bash
git add production.py test_production_logic.py
git commit -m "$(cat <<'EOF'
Add path/joystick conversion helpers to production.py

Pure functions that turn a drawn path into forward/back/up/down distance
commands (respecting apexdrone's 10-180cm range) and a joystick offset
into move() direction/power pairs. No tkinter or drone dependency yet,
so they're unit-tested directly.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: GUI skeleton — window, connection, takeoff/land/emergency, status bar

**Files:**
- Modify: `production.py` (add `DroneGUI` class and `main()`)

**Interfaces:**
- Consumes: `DroneWorker`, `PlannedStep` from `drone_worker.py` (Task 1); `Drone` from `apexdrone.py`; `CANVAS_SIZE`, `LINK`, `DRONE` constants from Task 2.
- Produces: `DroneGUI(root, drone, worker)` with a `mode: tk.StringVar` (`"planning"` / `"realtime"`), a `self.canvas` (`tk.Canvas`, size `CANVAS_SIZE`×`CANVAS_SIZE`), and `main()` that wires everything together and calls `root.mainloop()`.

- [ ] **Step 1: Add the `DroneGUI` class and `main()` to `production.py`**

Replace the `if __name__ == "__main__": pass` placeholder at the bottom of `production.py` with:

```python
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
```

Also move the existing `LINK`/`DRONE` constants (from Task 2) so they still sit near the top of the file — they are unchanged, just confirm they are not duplicated.

- [ ] **Step 2: Run the unit tests to confirm nothing broke**

Run: `python -m unittest test_production_logic -v`
Expected: PASS (12 tests) — adding `tkinter` imports and the `DroneGUI` class must not affect the pure functions.

- [ ] **Step 3: Manual verification**

Switch the drone on and place it in an open space with 3 m clearance and 2 m of ceiling
(see the Safety note under Global Constraints). Run: `python production.py`

Check:
- The terminal shows `Connected to APEX_USART_751F02 (protocol ...)` (from `_BleLink.open()`)
  before the window finishes opening — if it instead prints "No drone named ... found",
  the drone is off, out of range, or already connected to something else; fix that before
  continuing (see `README.md`'s Troubleshooting table).
- The window opens titled "APEX Drone - Drag Control". The status bar shows a battery
  voltage and "on ground" once the first telemetry packet arrives.
- Click **Takeoff** — the drone physically takes off to ~110 cm; log panel shows
  `takeoff: done` within a few seconds (no window freeze).
- Click **Land** — the drone lands; log panel shows `land: done`.
- Click **EMERGENCY STOP** — motors cut immediately (drone drops if airborne — only test
  this one on the ground or hovering low); log panel shows `EMERGENCY STOP sent.`.
- Switch to **Realtime** — Run and Clear buttons become disabled. Switch back to
  **Planning** — they re-enable.
- Close the window — the drone lands first (see `Drone.close()`), the terminal shows it
  disconnecting, and the process exits cleanly (no hang).

- [ ] **Step 4: Commit**

```bash
git add production.py
git commit -m "$(cat <<'EOF'
Add DroneGUI skeleton: window, connection, takeoff/land/emergency

Wires production.py's tkinter window to DroneWorker: takeoff/land run
through the queue, emergency stop bypasses it, and a status bar plus log
panel poll the worker every 100ms. Planning/Realtime mode switch already
enables and disables the plan controls; the drag interactions themselves
land in the next two tasks.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: Planning mode — draw a path, Run / Stop / Clear

**Files:**
- Modify: `production.py`

**Interfaces:**
- Consumes: `path_to_commands` (Task 2), `PlannedStep`, `worker.enqueue_plan`/`stop_plan`/`is_idle` (Task 1), `self.canvas`/`self.mode`/`self.path_points`/`self.drawing`/`self.plan_running` (Task 3).
- Produces: canvas mouse bindings and working Run/Stop/Clear buttons for Planning mode. `self.plan_running: bool` now reflects whether a plan is queued/executing.

- [ ] **Step 1: Bind canvas mouse events and implement Run/Stop, in `production.py`**

In `_build_widgets`, after `self._draw_grid()`, add:

```python
        self.canvas.bind("<ButtonPress-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
```

Add these methods (replacing the placeholder `_on_run` and `_on_stop_plan` from Task 3):

```python
    def _on_canvas_press(self, event: tk.Event) -> None:
        if self.mode.get() != "planning":
            return
        self._on_clear()
        self.path_points = [(event.x, event.y)]
        self.drawing = True

    def _on_canvas_drag(self, event: tk.Event) -> None:
        if self.mode.get() != "planning" or not self.drawing:
            return
        last = self.path_points[-1]
        if ((event.x - last[0]) ** 2 + (event.y - last[1]) ** 2) ** 0.5 >= WAYPOINT_MIN_PX:
            self.canvas.create_line(*last, event.x, event.y, fill="blue", width=2, tags="path")
            self.path_points.append((event.x, event.y))

    def _on_canvas_release(self, event: tk.Event) -> None:
        self.drawing = False

    def _on_run(self) -> None:
        if self.mode.get() != "planning" or len(self.path_points) < 2:
            self.log.insert("end", "Draw a path before pressing Run.")
            return
        commands = path_to_commands(self.path_points, PX_PER_CM)
        if not commands:
            self.log.insert("end", "Path too short to produce any commands.")
            return
        steps = [
            PlannedStep(getattr(self.drone, direction), kwargs={"dist": dist},
                        label=f"{direction} {dist:.0f}cm")
            for direction, dist in commands
        ]
        self.worker.enqueue_plan(steps)
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
```

Update `_poll_status` to auto-finish a plan once the worker drains its queue, by adding this check right after the `for line in ...` loop:

```python
        if self.plan_running and self.worker.is_idle():
            self._finish_plan()
```

Also import `path_to_commands` at the top of the file's tkinter-import block (it already lives in the same module, so no new import line is needed — just confirm it is defined above `DroneGUI` per Task 2).

- [ ] **Step 2: Run the unit tests to confirm nothing broke**

Run: `python -m unittest test_production_logic test_drone_worker -v`
Expected: PASS (18 tests total)

- [ ] **Step 3: Manual verification**

The drone will physically fly each command in the drawn path — recheck the 3 m/2 m
clearance from the Safety note before starting. Run: `python production.py`, take off
first, then stay in **Planning** mode.

Check:
- Drag a short diagonal line on the canvas (keep it small for the first test — a few cm of
  travel) — a blue polyline follows the mouse.
- Release the mouse, click **Run** — Run/Clear disable, Stop enables; the drone physically
  moves through each segment while the log panel fills with lines like `forward 40cm: done`,
  `up 20cm: done` in order, ending with Run/Clear re-enabling automatically once the queue
  drains.
- Draw a new path, click Run, then click **Stop** partway through — the drone finishes the
  segment already in progress and then holds; log shows `Plan stopped - remaining steps
  dropped.`, no further `...: done` lines appear afterward, and Run/Clear re-enable.
- Click **Run** with no path drawn — log shows "Draw a path before pressing Run." and
  nothing is queued.
- Click **Clear** after drawing (not running) — the blue line disappears.
- Land the drone (**Land** button) once done testing.

- [ ] **Step 4: Commit**

```bash
git add production.py
git commit -m "$(cat <<'EOF'
Wire Planning mode: drag to draw, Run/Stop/Clear execute the path

Dragging on the canvas records waypoints every 20px; Run converts them
through path_to_commands into a queued plan of forward/back/up/down
calls, Stop drops whatever hasn't started yet, and the buttons
re-enable automatically once the worker's queue drains.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: Realtime mode — joystick drag

**Files:**
- Modify: `production.py`

**Interfaces:**
- Consumes: `offset_to_moves` (Task 2), `worker.enqueue` (Task 1), `self.mode`/`self.canvas` (Task 3), the canvas press/drag/release bindings added in Task 4.
- Produces: a visible joystick dot that follows the mouse in Realtime mode and repeatedly jogs the drone while dragged.

- [ ] **Step 1: Extend the canvas handlers and add the jog loop, in `production.py`**

Add `self._joystick_offset = (0, 0)` and `self.jog_active = False` to `DroneGUI.__init__`, alongside the other instance variables set before `_build_widgets()` is called.

Replace `_on_canvas_press`, `_on_canvas_drag`, and `_on_canvas_release` (from Task 4) with:

```python
    def _on_canvas_press(self, event: tk.Event) -> None:
        if self.mode.get() == "planning":
            self._on_clear()
            self.path_points = [(event.x, event.y)]
            self.drawing = True
        else:
            self.jog_active = True
            self._update_joystick(event.x, event.y)
            self._jog_tick()

    def _on_canvas_drag(self, event: tk.Event) -> None:
        if self.mode.get() == "planning":
            if not self.drawing:
                return
            last = self.path_points[-1]
            if ((event.x - last[0]) ** 2 + (event.y - last[1]) ** 2) ** 0.5 >= WAYPOINT_MIN_PX:
                self.canvas.create_line(*last, event.x, event.y, fill="blue", width=2,
                                         tags="path")
                self.path_points.append((event.x, event.y))
        else:
            self._update_joystick(event.x, event.y)

    def _on_canvas_release(self, event: tk.Event) -> None:
        self.drawing = False
        self.jog_active = False
        self._reset_joystick()
```

Add these new methods:

```python
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

    def _jog_tick(self) -> None:
        if not self.jog_active:
            return
        dx, dy = self._joystick_offset
        for direction, power in offset_to_moves(dx, dy, JOYSTICK_RADIUS_PX):
            self.worker.enqueue(self.drone.move, direction, seconds=JOG_DURATION_S,
                                 power=power, label=f"jog {direction} {power}")
        self.root.after(JOG_INTERVAL_MS, self._jog_tick)
```

Update `_on_mode_change` to reset the joystick whenever the mode changes, by adding a call at the end of the method:

```python
    def _on_mode_change(self) -> None:
        planning = self.mode.get() == "planning"
        state = "normal" if planning else "disabled"
        self.run_button.configure(state=state)
        self.clear_button.configure(state=state)
        self._on_clear()
        self.jog_active = False
        self._reset_joystick()
```

Finally, call `self._reset_joystick()` once at the end of `_build_widgets` (after `self._on_mode_change()` is already called there, this draws the initial centered dot — confirm it isn't drawn twice; `_on_mode_change` already calls it, so no extra call is needed here).

- [ ] **Step 2: Run the unit tests to confirm nothing broke**

Run: `python -m unittest test_production_logic test_drone_worker -v`
Expected: PASS (18 tests total)

- [ ] **Step 3: Manual verification**

Dragging the joystick now jogs a real, airborne drone — recheck the 3 m/2 m clearance from
the Safety note and keep a hand near Emergency Stop. Run: `python production.py`, take off,
then switch to **Realtime** mode.

Check:
- An orange dot sits at the canvas center.
- Press and drag the mouse slightly to the right — the dot follows (clamped to the joystick
  radius), the drone noticeably leans/drifts forward, and the log panel shows repeated lines
  like `jog forward 40: done` roughly every 150ms.
- Drag up-and-right simultaneously — the drone climbs while drifting forward; log shows
  interleaved `jog forward ...` and `jog up ...` lines.
- Release the mouse — the dot springs back to center, the drone returns to a hover, and no
  new `jog ...` lines appear (the last one or two already queued still finish).
- Drag only a few pixels from center — the drone stays put; no `jog ...` lines appear at all
  (below `JOG_MIN_POWER`).
- Switch to Planning mid-drag (release first) — the joystick dot disappears/resets and
  dragging on the canvas now draws a path instead.
- Land the drone (**Land** button) once done testing.

- [ ] **Step 4: Commit**

```bash
git add production.py
git commit -m "$(cat <<'EOF'
Wire Realtime mode: joystick drag jogs the drone live

Dragging in Realtime mode moves an on-canvas joystick dot; while held, a
150ms loop converts its offset from center through offset_to_moves into
short Drone.move() calls queued on the worker. Releasing springs the dot
back to center and stops scheduling new jogs.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```
