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
