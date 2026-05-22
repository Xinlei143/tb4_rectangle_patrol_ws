import math
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import rclpy
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from tb4_rectangle_patrol.map_bounds import FreeSpaceBounds
from tb4_rectangle_patrol.rectangle_geometry import compute_rectangle_waypoints
from tb4_rectangle_patrol.sim_ready import wait_for_sim_ready
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RectanglePatrol:

    def __init__(self):
        self.navigator = TurtleBot4Navigator()
        self.navigator.declare_parameter('width', 0.8)
        self.navigator.declare_parameter('length', 1.3)
        self.navigator.declare_parameter('frame_id', 'map')
        self.navigator.declare_parameter('base_frame', 'base_link')
        self.navigator.declare_parameter('localizer', 'amcl')
        self.navigator.declare_parameter('initial_pose_enabled', True)
        self.navigator.declare_parameter('initial_x', 1.8)
        self.navigator.declare_parameter('initial_y', 0.2)
        self.navigator.declare_parameter('initial_yaw', math.pi / 2.0)
        self.navigator.declare_parameter('initial_yaw_tolerance', 0.35)
        self.navigator.declare_parameter('undock_mode', 'drive_forward')
        self.navigator.declare_parameter('undock_distance', 0.1)
        self.navigator.declare_parameter('undock_speed', 0.05)
        self.navigator.declare_parameter('map_bounds_enabled', True)
        self.navigator.declare_parameter('wait_for_sim_ready', True)
        self.navigator.declare_parameter('sim_ready_timeout_sec', 45.0)
        self.navigator.declare_parameter('retry_limit', 3)
        self.navigator.declare_parameter('goal_timeout_sec', 120.0)
        self.navigator.declare_parameter('clear_costmaps_on_retry', True)

        self.width = self.navigator.get_parameter('width').value
        self.length = self.navigator.get_parameter('length').value
        self.frame_id = self.navigator.get_parameter('frame_id').value
        self.base_frame = self.navigator.get_parameter('base_frame').value
        self.localizer = self.navigator.get_parameter('localizer').value
        self.initial_pose_enabled = self.navigator.get_parameter('initial_pose_enabled').value
        self.initial_x = self.navigator.get_parameter('initial_x').value
        self.initial_y = self.navigator.get_parameter('initial_y').value
        self.initial_yaw = self.navigator.get_parameter('initial_yaw').value
        self.initial_yaw_tolerance = self.navigator.get_parameter('initial_yaw_tolerance').value
        self.undock_mode = self.navigator.get_parameter('undock_mode').value
        self.undock_distance = self.navigator.get_parameter('undock_distance').value
        self.undock_speed = self.navigator.get_parameter('undock_speed').value
        self.map_bounds_enabled = self.navigator.get_parameter('map_bounds_enabled').value
        self.wait_for_sim_ready = self.navigator.get_parameter('wait_for_sim_ready').value
        self.sim_ready_timeout_sec = self.navigator.get_parameter('sim_ready_timeout_sec').value
        self.retry_limit = self.navigator.get_parameter('retry_limit').value
        self.goal_timeout_sec = self.navigator.get_parameter('goal_timeout_sec').value
        self.clear_costmaps_on_retry = (
            self.navigator.get_parameter('clear_costmaps_on_retry').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.navigator)
        self.map_bounds = None
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.map_subscription = self.navigator.create_subscription(
            OccupancyGrid,
            'map',
            self._map_callback,
            map_qos)
        self.cmd_vel_pub = self.navigator.create_publisher(Twist, 'cmd_vel', 10)
        self.dock_requested = False
        self.dock_service = self.navigator.create_service(
            Trigger,
            'rectangle_patrol/dock',
            self._dock_callback)

    def _map_callback(self, msg):
        free_cells = []
        for row in range(msg.info.height):
            for col in range(msg.info.width):
                value = msg.data[row * msg.info.width + col]
                if value == 0:
                    free_cells.append((col, row))

        if not free_cells:
            self.navigator.warn('Map has no free cells; waypoint bounds checks disabled.')
            return

        cols = [cell[0] for cell in free_cells]
        rows = [cell[1] for cell in free_cells]
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        height = msg.info.height
        self.map_bounds = FreeSpaceBounds(
            min_x=origin_x + (min(cols) + 0.5) * resolution,
            max_x=origin_x + (max(cols) + 0.5) * resolution,
            min_y=origin_y + (height - max(rows) - 0.5) * resolution,
            max_y=origin_y + (height - min(rows) - 0.5) * resolution)

    def _dock_callback(self, request, response):
        del request
        self.dock_requested = True
        response.success = True
        response.message = 'Dock request accepted. Canceling patrol and docking.'
        return response

    def _lookup_start_pose(self):
        deadline = self.navigator.get_clock().now() + Duration(seconds=10.0)
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    self.base_frame,
                    rclpy.time.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                return translation.x, translation.y, yaw_from_quaternion(rotation)
            except TransformException as exc:
                if self.navigator.get_clock().now() > deadline:
                    raise RuntimeError(
                        f'Could not get transform {self.frame_id}->{self.base_frame}: {exc}')
                rclpy.spin_once(self.navigator, timeout_sec=0.1)

        raise RuntimeError('ROS shutdown while waiting for start pose transform.')

    def _make_goal(self, waypoint):
        return self.navigator.getPoseStamped(
            [waypoint.x, waypoint.y],
            math.degrees(waypoint.yaw))

    def _set_initial_pose(self):
        if not self.initial_pose_enabled:
            self.navigator.info('Initial pose publishing disabled.')
            return

        initial_pose = self.navigator.getPoseStamped(
            [float(self.initial_x), float(self.initial_y)],
            math.degrees(float(self.initial_yaw)))
        self.navigator.info(
            'Publishing configured initial pose '
            f'x={float(self.initial_x):.3f}, '
            f'y={float(self.initial_y):.3f}, '
            f'yaw={math.degrees(float(self.initial_yaw)):.1f} deg.')
        self.navigator.setInitialPose(initial_pose)

    def _wait_until_nav2_active(self):
        self.navigator._waitForNodeToActivate(self.localizer)
        self._wait_for_localized_pose()
        self.navigator._waitForNodeToActivate('bt_navigator')
        self.navigator.info('Nav2 is ready for use!')

    def _wait_for_localized_pose(self):
        deadline = self.navigator.get_clock().now() + Duration(seconds=30.0)
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    self.frame_id,
                    self.base_frame,
                    rclpy.time.Time())
                return
            except TransformException as exc:
                if self.navigator.get_clock().now() > deadline:
                    raise RuntimeError(
                        f'Initial localization did not produce '
                        f'{self.frame_id}->{self.base_frame}: {exc}')
                rclpy.spin_once(self.navigator, timeout_sec=0.2)

    def _assert_expected_start_heading(self, yaw):
        yaw_error = abs(math.atan2(
            math.sin(yaw - float(self.initial_yaw)),
            math.cos(yaw - float(self.initial_yaw))))
        if yaw_error > float(self.initial_yaw_tolerance):
            raise RuntimeError(
                'Robot start yaw does not match configured right-bottom heading: '
                f'actual={math.degrees(yaw):.1f} deg, '
                f'expected={math.degrees(float(self.initial_yaw)):.1f} deg.')

    def _simple_undock(self):
        wait_for_sim_ready(
            self.navigator,
            enabled=self.wait_for_sim_ready,
            timeout_sec=self.sim_ready_timeout_sec,
            require_controller=True)

        if self.undock_mode != 'drive_forward':
            self.navigator.info('Using TurtleBot4 undock action.')
            self.navigator.undock()
            return

        distance = float(self.undock_distance)
        speed = float(self.undock_speed)
        if distance <= 0.0 or speed <= 0.0:
            self.navigator.info('Simple undock skipped because distance or speed is zero.')
            return

        self.navigator.info(
            f'Simple undock: driving forward {distance:.3f} m at {speed:.3f} m/s.')
        twist = Twist()
        twist.linear.x = speed
        duration = distance / speed
        started = time.monotonic()
        while rclpy.ok() and time.monotonic() - started < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self.navigator, timeout_sec=0.05)

        self.cmd_vel_pub.publish(Twist())

    def _validate_waypoints_inside_map(self, waypoints):
        if not self.map_bounds_enabled or self.map_bounds is None:
            return
        for index, waypoint in enumerate(waypoints):
            if not self.map_bounds.contains(waypoint.x, waypoint.y):
                raise RuntimeError(
                    f'Rectangle waypoint {index + 1} is outside free map bounds: '
                    f'x={waypoint.x:.3f}, y={waypoint.y:.3f}, '
                    f'bounds=({self.map_bounds.min_x:.3f}..{self.map_bounds.max_x:.3f}, '
                    f'{self.map_bounds.min_y:.3f}..{self.map_bounds.max_y:.3f}).')

    def _run_goal(self, goal, waypoint_index):
        attempts = 0
        while rclpy.ok() and not self.dock_requested:
            attempts += 1
            self.navigator.info(
                f'Navigating to rectangle waypoint {waypoint_index + 1}; attempt {attempts}.')
            self.navigator.goToPose(goal)
            started = self.navigator.get_clock().now()

            while rclpy.ok() and not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator, timeout_sec=0.1)
                if self.dock_requested:
                    self.navigator.cancelTask()
                    return False

                elapsed = self.navigator.get_clock().now() - started
                if elapsed > Duration(seconds=float(self.goal_timeout_sec)):
                    self.navigator.warn('Goal timed out; canceling current Nav2 task.')
                    self.navigator.cancelTask()
                    break

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.navigator.info(f'Reached rectangle waypoint {waypoint_index + 1}.')
                return True

            if self.dock_requested:
                return False

            if attempts >= int(self.retry_limit):
                self.navigator.error(
                    f'Waypoint {waypoint_index + 1} failed after {attempts} attempts.')
                return False

            self.navigator.warn(
                f'Waypoint {waypoint_index + 1} failed; retrying after costmap clear.')
            if self.clear_costmaps_on_retry:
                self.navigator.clearAllCostmaps()
            time.sleep(0.5)

        return False

    def _dock_and_exit(self):
        self.navigator.info('Dock requested. Canceling patrol and docking.')
        try:
            self.navigator.cancelTask()
        except Exception as exc:
            self.navigator.warn(f'Cancel task returned an error before docking: {exc}')
        self.navigator.dock()

    def run(self):
        self.navigator.info('Waiting for Nav2 to become active.')
        self._wait_until_nav2_active()

        start_x, start_y, start_yaw = self._lookup_start_pose()
        self._assert_expected_start_heading(start_yaw)

        self.navigator.info('Undocking before rectangle patrol.')
        self._simple_undock()

        start_x, start_y, start_yaw = self._lookup_start_pose()
        self._assert_expected_start_heading(start_yaw)
        waypoints = compute_rectangle_waypoints(
            start_x=start_x,
            start_y=start_y,
            start_yaw=start_yaw,
            width=float(self.width),
            length=float(self.length))
        self._validate_waypoints_inside_map(waypoints)
        goals = [self._make_goal(waypoint) for waypoint in waypoints]

        self.navigator.info(
            'Rectangle patrol started from '
            f'x={start_x:.3f}, y={start_y:.3f}, yaw={math.degrees(start_yaw):.1f} deg.')

        waypoint_index = 0
        while rclpy.ok() and not self.dock_requested:
            goal = goals[waypoint_index]
            if self._run_goal(goal, waypoint_index):
                waypoint_index = (waypoint_index + 1) % len(goals)
            elif not self.dock_requested:
                self.navigator.warn(
                    f'Keeping patrol on waypoint {waypoint_index + 1} after failure.')
                time.sleep(1.0)

        if self.dock_requested:
            self._dock_and_exit()


def main():
    rclpy.init()
    patrol = None
    exit_code = 0
    try:
        patrol = RectanglePatrol()
        patrol.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        exit_code = 1
        if patrol is not None:
            patrol.navigator.error(str(exc))
        else:
            print(str(exc), file=sys.stderr)
    finally:
        if patrol is not None:
            patrol.navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
