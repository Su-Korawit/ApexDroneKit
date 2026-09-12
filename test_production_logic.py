import unittest

from production import (CANVAS_CENTER, GRID_MAX_PX, GRID_MIN_PX, GRID_STEP_PX,
                        PX_PER_CM, commands_to_ghost_points, commands_to_steps,
                        find_hit, grid_lines, offset_to_moves, path_to_commands,
                        snap_to_grid)


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


class SnapToGridTests(unittest.TestCase):
    def test_centre_snaps_to_itself(self):
        self.assertEqual(snap_to_grid(CANVAS_CENTER, CANVAS_CENTER),
                         (CANVAS_CENTER, CANVAS_CENTER))

    def test_rounds_to_the_nearest_intersection(self):
        self.assertEqual(snap_to_grid(CANVAS_CENTER + 11, CANVAS_CENTER - 11),
                         (CANVAS_CENTER, CANVAS_CENTER))
        self.assertEqual(snap_to_grid(CANVAS_CENTER + 19, CANVAS_CENTER - 19),
                         (CANVAS_CENTER + GRID_STEP_PX, CANVAS_CENTER - GRID_STEP_PX))

    def test_clamps_inside_the_drawable_grid(self):
        self.assertEqual(snap_to_grid(-500, 9999), (GRID_MIN_PX, GRID_MAX_PX))

    def test_every_snapped_point_sits_on_an_intersection(self):
        for raw in (0, 37, 88, 150, 201, 260, 333, 400):
            x, y = snap_to_grid(raw, raw)
            self.assertEqual((x - CANVAS_CENTER) % GRID_STEP_PX, 0)
            self.assertEqual((y - CANVAS_CENTER) % GRID_STEP_PX, 0)


class GridSnappedExactnessTests(unittest.TestCase):
    """The reason snapping exists: no travel is silently discarded."""

    def _net(self, commands, positive, negative):
        return sum(cm if d == positive else -cm
                   for d, cm in commands if d in (positive, negative))

    def test_snapped_chain_commands_the_full_drawn_distance(self):
        nodes = [snap_to_grid(37, 211), snap_to_grid(140, 96), snap_to_grid(305, 268)]
        commands = path_to_commands(nodes, PX_PER_CM)
        expected_x = (nodes[-1][0] - nodes[0][0]) / PX_PER_CM
        expected_y = (nodes[0][1] - nodes[-1][1]) / PX_PER_CM
        self.assertAlmostEqual(self._net(commands, "forward", "back"), expected_x)
        self.assertAlmostEqual(self._net(commands, "up", "down"), expected_y)

    def test_every_snapped_leg_is_a_whole_number_of_minimum_moves(self):
        nodes = [snap_to_grid(12, 390), snap_to_grid(207, 118), snap_to_grid(377, 44)]
        for direction, distance in path_to_commands(nodes, PX_PER_CM):
            self.assertAlmostEqual(distance % 10, 0,
                                   msg=f"{direction} {distance} is not a multiple of 10cm")


class GridLinesTests(unittest.TestCase):
    def test_line_count_spans_the_canvas(self):
        self.assertEqual(len(grid_lines()), 13)

    def test_centre_line_is_zero_and_major(self):
        centre = [line for line in grid_lines() if line[0] == CANVAS_CENTER]
        self.assertEqual(centre, [(CANVAS_CENTER, 0, True)])

    def test_majors_are_every_thirty_cm(self):
        majors = [cm for _, cm, major in grid_lines() if major]
        self.assertEqual(majors, [-60, -30, 0, 30, 60])

    def test_positions_stay_within_the_drawable_grid(self):
        for position, _, _ in grid_lines():
            self.assertGreaterEqual(position, GRID_MIN_PX)
            self.assertLessEqual(position, GRID_MAX_PX)


class CommandsToGhostPointsTests(unittest.TestCase):
    def test_no_commands_gives_just_the_start(self):
        self.assertEqual(commands_to_ghost_points([], (50, 200)), [(50, 200)])

    def test_each_command_adds_one_point(self):
        ghost = commands_to_ghost_points([("forward", 30.0), ("up", 30.0)], (50, 200))
        self.assertEqual(ghost, [(50, 200), (140.0, 200), (140.0, 110.0)])

    def test_back_and_down_move_the_other_way(self):
        ghost = commands_to_ghost_points([("back", 10.0), ("down", 10.0)], (200, 200))
        self.assertEqual(ghost, [(200, 200), (170.0, 200), (170.0, 230.0)])

    def test_a_diagonal_leg_flies_as_an_L(self):
        nodes = [(50, 200), (140, 110)]
        ghost = commands_to_ghost_points(path_to_commands(nodes, PX_PER_CM), nodes[0])
        self.assertEqual(ghost, [(50, 200), (140.0, 200), (140.0, 110.0)])
        self.assertNotEqual(ghost, nodes)


class FindHitTests(unittest.TestCase):
    NODES = [(50, 200), (140, 200), (140, 110)]

    def test_no_nodes_hits_nothing(self):
        self.assertIsNone(find_hit([], 100, 100))

    def test_first_and_last_nodes_are_ends(self):
        self.assertEqual(find_hit(self.NODES, 50, 200), ("end", 0))
        self.assertEqual(find_hit(self.NODES, 140, 110), ("end", 2))

    def test_leg_midpoint_is_a_leg_hit(self):
        self.assertEqual(find_hit(self.NODES, 95, 200), ("leg", 0))
        self.assertEqual(find_hit(self.NODES, 140, 155), ("leg", 1))

    def test_interior_node_is_not_an_end(self):
        self.assertIsNone(find_hit(self.NODES, 140, 200))

    def test_empty_space_hits_nothing(self):
        self.assertIsNone(find_hit(self.NODES, 300, 300))

    def test_ends_win_over_legs_on_a_one_cell_leg(self):
        nodes = [(200, 200), (230, 200)]
        self.assertEqual(find_hit(nodes, 201, 200), ("end", 0))
        self.assertEqual(find_hit(nodes, 215, 200), ("leg", 0))


if __name__ == "__main__":
    unittest.main()
