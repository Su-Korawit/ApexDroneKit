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

    def test_stop_plan_announces_itself(self):
        self.worker.stop_plan()
        self.assertIn("Plan stopped - remaining steps dropped.",
                      self.worker.drain_status())

    def test_flush_drops_unstarted_steps_without_a_status_line(self):
        started = threading.Event()
        release = threading.Event()
        ran = []

        def slow_first():
            started.set()
            release.wait(timeout=2)
            ran.append("first")

        self.worker.enqueue(slow_first, label="first")
        self.worker.enqueue(lambda: ran.append("second"), label="second")
        self.assertTrue(_wait_for(started.is_set))
        self.worker.flush()
        release.set()
        self.assertTrue(_wait_for(lambda: "first" in ran, timeout=1))
        self.assertTrue(_wait_for(self.worker.is_idle, timeout=1))
        self.assertNotIn("second", ran)
        self.assertNotIn("Plan stopped - remaining steps dropped.",
                         self.worker.drain_status())

    def test_enqueue_after_flush_still_runs(self):
        self.worker.flush()
        ran = []
        self.worker.enqueue(lambda: ran.append("after"), label="after")
        self.assertTrue(_wait_for(lambda: ran == ["after"]))

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
