import math
import unittest

from tb4_rectangle_patrol.rectangle_geometry import (
    compute_rectangle_waypoints,
    compute_rear_dock_pose,
    simulate_forward_offset,
)


class RectangleGeometryTest(unittest.TestCase):

    def test_generates_left_turn_rectangle_from_current_pose(self):
        waypoints = compute_rectangle_waypoints(
            start_x=0.0,
            start_y=0.0,
            start_yaw=0.0,
            width=0.8,
            length=1.3,
        )

        expected = [
            (0.8, 0.0, math.pi / 2.0),
            (0.8, 1.3, math.pi),
            (0.0, 1.3, -math.pi / 2.0),
            (0.0, 0.0, 0.0),
        ]

        self.assertEqual(len(waypoints), len(expected))
        for waypoint, expected_values in zip(waypoints, expected):
            self.assertAlmostEqual(waypoint.x, expected_values[0], places=6)
            self.assertAlmostEqual(waypoint.y, expected_values[1], places=6)
            self.assertAlmostEqual(waypoint.yaw, expected_values[2], places=6)

    def test_respects_nonzero_start_yaw(self):
        waypoints = compute_rectangle_waypoints(
            start_x=1.0,
            start_y=2.0,
            start_yaw=math.pi / 2.0,
            width=0.8,
            length=1.3,
        )

        expected = [
            (1.0, 2.8, math.pi),
            (-0.3, 2.8, -math.pi / 2.0),
            (-0.3, 2.0, 0.0),
            (1.0, 2.0, math.pi / 2.0),
        ]

        for waypoint, expected_values in zip(waypoints, expected):
            self.assertAlmostEqual(waypoint.x, expected_values[0], places=6)
            self.assertAlmostEqual(waypoint.y, expected_values[1], places=6)
            self.assertAlmostEqual(waypoint.yaw, expected_values[2], places=6)

    def test_right_bottom_start_faces_top_right_corner(self):
        x, y, yaw = simulate_forward_offset(
            x=1.8,
            y=0.2,
            yaw=math.pi / 2.0,
            distance=0.1,
        )

        self.assertAlmostEqual(x, 1.8, places=6)
        self.assertAlmostEqual(y, 0.3, places=6)
        self.assertAlmostEqual(yaw, math.pi / 2.0, places=6)

        waypoints = compute_rectangle_waypoints(
            start_x=x,
            start_y=y,
            start_yaw=yaw,
            width=0.8,
            length=1.3,
        )

        expected = [
            (1.8, 1.1, math.pi),
            (0.5, 1.1, -math.pi / 2.0),
            (0.5, 0.3, 0.0),
            (1.8, 0.3, math.pi / 2.0),
        ]

        for waypoint, expected_values in zip(waypoints, expected):
            self.assertAlmostEqual(waypoint.x, expected_values[0], places=6)
            self.assertAlmostEqual(waypoint.y, expected_values[1], places=6)
            self.assertAlmostEqual(waypoint.yaw, expected_values[2], places=6)

    def test_dock_pose_is_behind_right_bottom_robot_and_faces_top_right(self):
        x, y, yaw = compute_rear_dock_pose(
            robot_x=1.8,
            robot_y=0.2,
            robot_yaw=math.pi / 2.0,
            offset=0.157,
            dock_yaw=math.pi / 2.0,
        )

        self.assertAlmostEqual(x, 1.8, places=6)
        self.assertAlmostEqual(y, 0.043, places=6)
        self.assertAlmostEqual(yaw, math.pi / 2.0, places=6)


if __name__ == '__main__':
    unittest.main()
