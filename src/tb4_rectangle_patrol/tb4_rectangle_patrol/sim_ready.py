import time

from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
import rclpy


class SimReadyWaiter:

    def __init__(
            self,
            node,
            require_clock=True,
            require_scan=True,
            require_odom=True,
            require_controller=True):
        self.node = node
        self.require_clock = require_clock
        self.require_scan = require_scan
        self.require_odom = require_odom
        self.require_controller = require_controller
        self.clock_ready = not require_clock
        self.scan_ready = not require_scan
        self.odom_ready = not require_odom
        self.controller_ready = not require_controller
        self.controller_client = None
        if require_controller:
            self.controller_client = node.create_client(
                ListControllers,
                '/controller_manager/list_controllers')
        self.clock_sub = None
        self.scan_sub = None
        self.odom_sub = None
        if require_clock:
            self.clock_sub = node.create_subscription(
                Clock,
                '/clock',
                self._clock_callback,
                10)
        if require_scan:
            self.scan_sub = node.create_subscription(
                LaserScan,
                'scan',
                self._scan_callback,
                10)
        if require_odom:
            self.odom_sub = node.create_subscription(
                Odometry,
                'odom',
                self._odom_callback,
                10)

    def _clock_callback(self, msg):
        self.clock_ready = msg.clock.sec > 0 or msg.clock.nanosec > 0

    def _scan_callback(self, _msg):
        self.scan_ready = True

    def _odom_callback(self, _msg):
        self.odom_ready = True

    def _check_controller(self):
        if self.controller_ready or self.controller_client is None:
            return
        if not self.controller_client.service_is_ready():
            return
        future = self.controller_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return
        self.controller_ready = any(
            controller.name == 'diffdrive_controller'
            and controller.state == 'active'
            for controller in future.result().controller)

    def ready(self):
        return (
            self.clock_ready
            and self.scan_ready
            and self.odom_ready
            and self.controller_ready
        )

    def missing(self):
        missing = []
        if self.require_clock and not self.clock_ready:
            missing.append('clock')
        if self.require_scan and not self.scan_ready:
            missing.append('scan')
        if self.require_odom and not self.odom_ready:
            missing.append('odom')
        if self.require_controller and not self.controller_ready:
            missing.append('diffdrive_controller')
        return ', '.join(missing)


def wait_for_sim_ready(
        node,
        enabled=True,
        timeout_sec=45.0,
        require_clock=True,
        require_scan=True,
        require_odom=True,
        require_controller=True):
    if not enabled:
        return

    waiter = SimReadyWaiter(
        node,
        require_clock=require_clock,
        require_scan=require_scan,
        require_odom=require_odom,
        require_controller=require_controller)
    deadline = time.monotonic() + float(timeout_sec)
    last_log = 0.0

    while rclpy.ok() and not waiter.ready():
        waiter._check_controller()
        now = time.monotonic()
        if now - last_log > 2.0:
            node.get_logger().info(
                f'Waiting for simulation readiness: {waiter.missing()}')
            last_log = now
        if now > deadline:
            raise RuntimeError(
                f'Timed out waiting for simulation readiness: {waiter.missing()}')
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Simulation readiness confirmed.')
