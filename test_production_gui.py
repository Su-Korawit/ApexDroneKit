"""Regression tests for the canvas drag/right-click handlers in
production.DroneGUI.

These exercise the handlers headlessly - no real Tk window, no drone -
by binding the real DroneGUI methods onto a bare harness object that
supplies just the attributes they touch (self.mode, self.canvas,
self.log, self.nodes, self._drag, self.plan_running). self.canvas is a
recording no-op stand-in for tkinter.Canvas, exposing only the handful
of create_*/delete/itemconfigure calls _redraw_path makes.
"""

import unittest

from production import DroneGUI


class _FakeMode:
    """Stands in for the tk.StringVar driving Planning/Realtime mode."""

    def get(self):
        return "planning"


class _FakeCanvas:
    """No-op stand-in for tkinter.Canvas - enough for _redraw_path and
    the click handlers to run without raising, without a real Tk window."""

    def create_line(self, *args, **kwargs):
        pass

    def create_oval(self, *args, **kwargs):
        pass

    def create_rectangle(self, *args, **kwargs):
        pass

    def create_text(self, *args, **kwargs):
        pass

    def delete(self, *args, **kwargs):
        pass

    def itemconfigure(self, *args, **kwargs):
        pass


class _FakeLog:
    def insert(self, *args, **kwargs):
        pass


class _Event:
    """Stands in for a tkinter mouse event - only .x/.y are read."""

    def __init__(self, x, y):
        self.x = x
        self.y = y


class _Harness:
    """Minimal DroneGUI stand-in: real handler methods, fake canvas/log."""

    _on_canvas_press = DroneGUI._on_canvas_press
    _on_canvas_drag = DroneGUI._on_canvas_drag
    _on_canvas_release = DroneGUI._on_canvas_release
    _on_canvas_right_click = DroneGUI._on_canvas_right_click
    _redraw_path = DroneGUI._redraw_path

    def __init__(self, nodes=None):
        self.mode = _FakeMode()
        self.canvas = _FakeCanvas()
        self.log = _FakeLog()
        self.nodes = list(nodes) if nodes else []
        self._drag = None
        self.plan_running = False


class RightClickDuringDragTests(unittest.TestCase):
    """Regression for the cross-task race: Tk delivers <ButtonPress-3>
    even while button 1 is held, so a right-click can land mid-drag and
    pop a node whose index the in-flight self._drag still references."""

    def test_no_operation_sequence_leaves_a_single_node(self):
        """A single leftover node is a state _redraw_path cannot render
        (create_line needs >= 2 points) - no press/drag/release/right-click
        sequence should ever produce it."""
        h = _Harness(nodes=[(100, 100), (200, 100)])
        h._on_canvas_press(_Event(200, 100))          # grab back end -> "extend", at=1
        try:
            h._on_canvas_right_click(_Event(100, 100))  # right-click the other end mid-drag
        except Exception:
            pass  # a real Tk mainloop would swallow this and keep delivering events
        try:
            h._on_canvas_release(_Event(250, 100))
        except Exception:
            pass
        self.assertNotEqual(len(h.nodes), 1,
                             "a single orphaned node cannot be redrawn")
        h._redraw_path()  # must not raise regardless of the outcome above

    def test_right_click_during_extend_drag_is_ignored(self):
        h = _Harness(nodes=[(100, 100), (200, 100)])
        h._on_canvas_press(_Event(200, 100))  # grab back end -> "extend", at=1
        self.assertEqual(h._drag["kind"], "extend")
        before_nodes = list(h.nodes)
        before_drag = dict(h._drag)

        h._on_canvas_right_click(_Event(100, 100))  # right-click the other end mid-drag

        self.assertEqual(h.nodes, before_nodes)
        self.assertEqual(h._drag, before_drag)

    def test_right_click_during_bend_drag_is_ignored(self):
        h = _Harness(nodes=[(100, 100), (200, 100)])
        h._on_canvas_press(_Event(150, 100))  # leg midpoint -> "bend", leg=0
        self.assertEqual(h._drag["kind"], "bend")
        before_nodes = list(h.nodes)
        before_drag = dict(h._drag)

        h._on_canvas_right_click(_Event(100, 100))  # right-click an end mid-drag

        self.assertEqual(h.nodes, before_nodes)
        self.assertEqual(h._drag, before_drag)


if __name__ == "__main__":
    unittest.main()
