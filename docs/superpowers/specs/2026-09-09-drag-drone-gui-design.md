# Drag-Control Drone GUI (`production.py`) — Design

## Goal

Give `production.py` a tkinter GUI that controls the APEX GD-149 drone (via
`apexdrone.py`) along two axes only:

- **X axis** — forward / backward
- **Y axis** — altitude (up / down)

Two modes, selectable from the same window:

- **Planning** — drag on a canvas to draw a path first; pressing **Run**
  executes the whole path as a sequence of drone commands.
- **Realtime** — drag a joystick-style point; the drone responds live while
  dragging, like a physical stick.

Left/right strafing and rotation are explicitly out of scope.

## Files

- `drone_worker.py` (new) — background-thread command queue for the `Drone`,
  following the pattern `README.md` already documents (but the file itself
  does not exist yet in this repo).
- `production.py` (currently empty) — the GUI application.

## `drone_worker.py`

```python
class DroneWorker:
    def __init__(self, drone: Drone): ...
    def start(self) -> None: ...                       # spawn the worker thread
    def enqueue(self, fn, *args, **kwargs) -> None: ... # queue one drone call
    def enqueue_plan(self, steps: list[Callable]) -> None: ...  # queue a tagged batch
    def stop_plan(self) -> None: ...                    # drop remaining queued plan steps
    def emergency_stop(self) -> None: ...                # calls drone.emergency_stop() directly, bypassing the queue
    def drain_status(self) -> list[str]: ...             # non-blocking pop of queued status/log strings
```

- One `queue.Queue` holds pending drone calls; a single daemon thread runs
  them one at a time, so tkinter's main loop never blocks on a drone command.
- Every executed call (and every exception it raises, including the
  library's `ValueError` for out-of-range values) is turned into a short
  string and pushed onto a `status_queue`. The GUI drains this queue with
  `root.after(100, poll)` and appends lines to the log panel.
- `enqueue_plan` tags its items with a generation counter. `stop_plan`
  increments the counter, so already-queued-but-not-yet-run steps from the
  previous generation are discarded when the worker dequeues them; the step
  currently executing still finishes normally (a `move()`/`forward()` call
  cannot be interrupted mid-flight without touching the drone's internals).
- `emergency_stop()` is called directly from the GUI's button callback in
  its own short-lived thread — it does not go through `enqueue`, so it fires
  immediately even while a plan or realtime jog is mid-command.

## `production.py`

### Layout

- Top bar: **Takeoff**, **Land**, **Emergency Stop** (red), and a mode
  selector (Planning / Realtime).
- Center: a `tk.Canvas` (400×400 px) with grid lines and a centered
  crosshair. `PX_PER_CM = 3` maps pixels to centimetres for both axes,
  giving roughly ±65 cm of reach per axis — comfortably inside the
  library's 10–180 cm move range on the X axis component.
- Bottom: connection/battery/altitude status line + a scrolling log
  (fed from `DroneWorker.drain_status()`).

### Planning mode

1. Mouse-down on the canvas starts capturing points; mouse-move while the
   button is held draws a live polyline and appends a point whenever the
   cursor has moved at least 20 px from the last recorded point.
2. Mouse-up finalizes the path (drawn, not yet sent).
3. **Run** converts consecutive waypoint pairs into commands:
   - `dx = (x2 - x1) / PX_PER_CM`, `dy = (y1 - y2) / PX_PER_CM` (canvas y
     grows downward, altitude grows upward, so it is inverted).
   - Segments under 10 cm (the library's `DIST_MIN_CM`) are accumulated
     into the next segment instead of being sent, since `forward()`/`up()`
     reject anything below 10 cm.
   - Segments over 180 cm (`DIST_MAX_CM`) are split into multiple calls.
   - Each resulting segment becomes a `d.forward()`/`d.back()` call for its
     `dx` (skipped if `dx` rounds to 0) followed by a `d.up()`/`d.down()`
     call for its `dy` (skipped if `dy` rounds to 0), queued via
     `enqueue_plan`. A segment with both `dx` and `dy` below 10 cm keeps
     accumulating with the next segment until at least one axis clears the
     threshold.
4. **Stop** calls `worker.stop_plan()`. **Clear** erases the drawn path
   (only enabled when no plan is running).

### Realtime mode

1. A joystick dot starts centered on the canvas. Dragging moves the dot,
   clamped to the canvas radius.
2. While the mouse button is held, a `root.after(150, jog)` loop reads the
   dot's offset from center, converts it to a direction + power (0–100)
   per axis exactly as in `Drone.move()`, and calls
   `worker.enqueue(d.move, direction, seconds=0.15, power=power)` for each
   axis that has a non-zero offset.
3. Mouse-up stops scheduling new jog calls and springs the dot back to
   center. Any already-queued `move()` call finishes normally (max 0.15 s).

### Connection

- `LINK = "sim"` and `DRONE = ""` constants at the top of the file, matching
  `console.py`'s pattern — edited by hand before flying for real.
- The `Drone` connects once when the window opens (`Drone(LINK, ...).connect()`)
  and `d.close()` is called from the `WM_DELETE_WINDOW` handler so the
  window closing always lands the drone first.

### Error handling

- Any `ValueError` raised by the library's `_check()` (e.g. a computed
  distance still outside range after the splitting/accumulation above) is
  caught inside `DroneWorker`'s run loop and reported as a log line —
  it never crashes the GUI or the worker thread.
- Emergency Stop remains clickable regardless of what else is queued or
  running, per the `drone_worker.py` design above.

## Testing

There is no automated test coverage for the existing GUI-shaped scripts in
this repo (`console.py` has none either). Verification is manual, run in
`sim` mode:

1. Launch `production.py`, confirm it connects in `sim` mode and shows
   "Practice mode" in the log.
2. Planning mode: draw a path, confirm the log shows a sensible sequence of
   forward/back/up/down calls with distances in range, press Stop mid-run
   and confirm remaining steps are dropped.
3. Realtime mode: drag the joystick in each direction and confirm the log
   shows repeated short `move()` calls with the expected direction/power,
   and that releasing the mouse stops new calls from being queued.
4. Confirm Emergency Stop works while a plan is running.
