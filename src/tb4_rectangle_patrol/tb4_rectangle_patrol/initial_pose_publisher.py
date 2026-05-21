import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


def make_initial_pose(x, y, yaw, frame_id='map'):
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = frame_id
    pose.pose.pose.position.x = float(x)
    pose.pose.pose.position.y = float(y)
    pose.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    pose.pose.covariance[0] = 0.25
    pose.pose.covariance[7] = 0.25
    pose.pose.covariance[35] = 0.06853892326654787
    return pose


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('initial_x', 1.8)
        self.declare_parameter('initial_y', 0.2)
        self.declare_parameter('initial_yaw', math.pi / 2.0)
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('max_publish_count', 20)

        self.frame_id = self.get_parameter('frame_id').value
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_yaw = self.get_parameter('initial_yaw').value
        self.max_publish_count = int(self.get_parameter('max_publish_count').value)
        self.publish_count = 0
        self.map_received = False
        self.amcl_pose_received = False

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(OccupancyGrid, 'map', self._map_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._amcl_pose_callback,
            10)
        self.timer = self.create_timer(
            float(self.get_parameter('publish_period_sec').value),
            self._publish_initial_pose)

    def _map_callback(self, _msg):
        if not self.map_received:
            self.get_logger().info('Map received; initial pose publishing is enabled.')
        self.map_received = True

    def _amcl_pose_callback(self, _msg):
        if not self.amcl_pose_received:
            self.get_logger().info('AMCL pose received; stopping initial pose publisher.')
        self.amcl_pose_received = True
        self.timer.cancel()

    def _publish_initial_pose(self):
        if self.amcl_pose_received:
            return
        if self.max_publish_count > 0 and self.publish_count >= self.max_publish_count:
            self.get_logger().warn('Initial pose publish limit reached before AMCL pose.')
            self.timer.cancel()
            return
        pose = make_initial_pose(
            x=self.initial_x,
            y=self.initial_y,
            yaw=self.initial_yaw,
            frame_id=self.frame_id)
        self.publisher.publish(pose)
        self.publish_count += 1
        self.get_logger().info(
            'Published initial pose '
            f'x={float(self.initial_x):.3f}, '
            f'y={float(self.initial_y):.3f}, '
            f'yaw={math.degrees(float(self.initial_yaw)):.1f} deg.')


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
