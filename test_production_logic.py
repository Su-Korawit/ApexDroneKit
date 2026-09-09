import unittest

from production import commands_to_steps, path_to_commands, offset_to_moves


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


class StubDrone:
    """Just the four movement methods production.py plans with, recording
    how they were called."""

    def __init__(self):
        self.calls = []

    def forward(self, dist):
        self.calls.append(("forward", dist))

    def back(self, dist):
        self.calls.append(("back", dist))

    def up(self, dist):
        self.calls.append(("up", dist))

    def down(self, dist):
        self.calls.append(("down", dist))


class CommandsToStepsTests(unittest.TestCase):
    def test_no_commands_gives_no_steps(self):
        self.assertEqual(commands_to_steps(StubDrone(), []), [])

    def test_steps_bind_the_matching_drone_methods(self):
        drone = StubDrone()
        steps = commands_to_steps(drone, [("forward", 40.0), ("up", 20.0)])
        self.assertEqual([step.fn for step in steps], [drone.forward, drone.up])

    def test_steps_pass_distance_as_the_dist_keyword(self):
        steps = commands_to_steps(StubDrone(), [("forward", 40.0), ("up", 20.0)])
        self.assertEqual([step.kwargs for step in steps],
                         [{"dist": 40.0}, {"dist": 20.0}])
        self.assertEqual([step.args for step in steps], [(), ()])

    def test_step_labels_round_the_distance(self):
        steps = commands_to_steps(StubDrone(), [("back", 10.0), ("down", 13.333333)])
        self.assertEqual([step.label for step in steps], ["back 10cm", "down 13cm"])

    def test_running_the_steps_calls_the_real_methods(self):
        drone = StubDrone()
        for step in commands_to_steps(drone, [("forward", 40.0), ("down", 20.0)]):
            step.fn(*step.args, **step.kwargs)
        self.assertEqual(drone.calls, [("forward", 40.0), ("down", 20.0)])

    def test_every_direction_path_to_commands_emits_is_a_real_method(self):
        drone = StubDrone()
        commands = path_to_commands([(0, 0), (60, -60), (0, 0)], px_per_cm=3)
        self.assertTrue(commands)
        for step in commands_to_steps(drone, commands):
            step.fn(*step.args, **step.kwargs)
        self.assertEqual(len(drone.calls), len(commands))


if __name__ == "__main__":
    unittest.main()
