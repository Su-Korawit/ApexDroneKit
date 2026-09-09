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
