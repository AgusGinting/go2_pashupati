"""Go2 EDU driver for Pashupati: the robot contract of src/xtras/README.md over unitree_sdk2py.

Subscribes cmd_vel (Twist, base_link) and sends the sport service's Move while commands keep
coming; StopMove when they stop or go to zero, and after cmd_timeout without one. Publishes odom
(the sport-mode velocity in base_link), battery and diagnostics; serves stand_up, sit_down and
emergency_stop (Damp: the motors go soft and the robot lies down, like the remote's e-stop).
It publishes no TF: Pashupati owns odom -> base_link.
"""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger

from go2_bringup.core import MOVE, STOP, CommandGate, battery_fraction, body_velocity
from go2_bringup.link import connect

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
# body_height above this is standing (0.32 m standing, 0.08 lying on the Go2 EDU): the sport
# mode takes SpeedLevel only then
STANDING_HEIGHT = 0.2


class Go2DriverNode(Node):
    """cmd_vel -> sport Move; sport and low state -> odom, battery, diagnostics."""

    def __init__(self, sport=None):
        """Connect to the robot, or use sport (an object with SportClient's methods) in tests."""
        super().__init__('go2_driver_node')
        interface = self.declare_parameter('network_interface', 'eth0').value
        timeout = self.declare_parameter('cmd_timeout', 0.5).value
        rate = self.declare_parameter('command_rate', 50.0).value
        # a ceiling under the control mux's own limits (robot.limits), never above them
        ceiling = (('max_vx', 1.0), ('max_vy', 0.5), ('max_wz', 1.5))
        limits = tuple(self.declare_parameter(name, value).value for name, value in ceiling)
        self.state_timeout = self.declare_parameter('state_timeout', 1.0).value
        self.stand_settle = self.declare_parameter('stand_settle', 1.5).value
        # the sport mode's speed range: -1 slow, 0 normal, 1 fast; the firmware caps Move by it
        self.speed_level = self.declare_parameter('speed_level', 1).value
        self.level_set = False  # set again whenever the robot stands up, by whatever means

        self.gate = CommandGate(timeout, limits)
        self._sport_state = None
        self._sport_stamp: float | None = None
        self._sport_count = 0
        self._sent_count = 0
        self._low_state = None
        self._low_stamp: float | None = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        # reentrant: an emergency_stop is served while stand_up is still settling
        services = ReentrantCallbackGroup()
        for name, callback in (('stand_up', self.stand_up), ('sit_down', self.sit_down),
                               ('emergency_stop', self.emergency_stop)):  # fmt: skip
            self.create_service(Trigger, name, callback, callback_group=services)

        self._subscribers = []
        if sport is None:
            sport, self._subscribers = connect(interface, self.on_sport_state, self.on_low_state)
            self.get_logger().info(f'Go2 sport service on {interface or "the default interface"}')
        self.sport = sport
        if self.set_speed_level(quiet=True) != 0:
            self.get_logger().info(f'speed level {self.speed_level} once the robot stands')
        self.create_timer(1.0 / rate, self.on_tick)
        self.create_timer(1.0, self.on_status)

    # SDK threads: keep the latest sample only
    def on_sport_state(self, state):
        """Store a SportModeState_."""
        self._sport_state = state
        self._sport_stamp = time.monotonic()
        self._sport_count += 1

    def on_low_state(self, state):
        """Store a LowState_."""
        self._low_state = state
        self._low_stamp = time.monotonic()

    def on_cmd_vel(self, msg: Twist):
        """Take a command for the next tick."""
        if not self.gate.update(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic()):
            self.get_logger().warn('dropped a non-finite cmd_vel', throttle_duration_sec=5.0)

    def on_tick(self):
        """Send the command and publish odom from a new sport state."""
        action, (vx, vy, wz) = self.gate.tick(time.monotonic())
        if action == MOVE:
            self.sport.Move(vx, vy, wz)
        elif action == STOP:
            self.sport.StopMove()
        if self._sport_count != self._sent_count:
            self._sent_count = self._sport_count
            self.odom_pub.publish(self.odometry(self._sport_state))

    def odometry(self, state) -> Odometry:
        """Return the sport state as Odometry: pose in Unitree's frame, twist in base_link."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ODOM_FRAME
        msg.child_frame_id = BASE_FRAME
        position = msg.pose.pose.position
        position.x, position.y, position.z = (float(v) for v in state.position)
        w, x, y, z = (float(v) for v in state.imu_state.quaternion)  # Unitree order: w first
        orientation = msg.pose.pose.orientation
        orientation.w, orientation.x, orientation.y, orientation.z = w, x, y, z
        vx, vy = body_velocity(state.velocity[0], state.velocity[1], state.imu_state.rpy[2])
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx, vy
        msg.twist.twist.linear.z = float(state.velocity[2])
        msg.twist.twist.angular.z = float(state.yaw_speed)
        return msg

    def on_status(self):
        """Publish battery (while the low state is fresh) and the driver's diagnostics."""
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        low = self._low_state
        if low is not None and now - self._low_stamp <= self.state_timeout:
            battery = BatteryState()
            battery.header.stamp = stamp
            battery.voltage = float(low.power_v)
            battery.current = low.bms_state.current / 1000.0  # mA, negative while discharging
            battery.percentage = battery_fraction(low.bms_state.soc)
            battery.present = True
            self.battery_pub.publish(battery)

        status = DiagnosticStatus(name='go2_driver', hardware_id='go2')
        state = self._sport_state
        if state is None or now - self._sport_stamp > self.state_timeout:
            status.level, status.message = DiagnosticStatus.ERROR, 'no sport state from the robot'
        else:
            if state.body_height <= STANDING_HEIGHT:
                self.level_set = False  # lying: the next stand-up sets it again
            elif not self.level_set:
                self.set_speed_level()
            status.level, status.message = DiagnosticStatus.OK, 'ok'
            status.values = [KeyValue(key=key, value=str(value)) for key, value in (
                ('mode', state.mode), ('gait_type', state.gait_type),
                ('error_code', state.error_code), ('body_height', round(state.body_height, 3)),
            )]  # fmt: skip
        if low is not None:
            status.values.append(KeyValue(key='soc', value=str(low.bms_state.soc)))
        diagnostics = DiagnosticArray(status=[status])
        diagnostics.header.stamp = stamp
        self.diagnostics_pub.publish(diagnostics)

    def set_speed_level(self, quiet: bool = False) -> int:
        """Ask the sport mode for speed_level; it refuses while the robot lies (code -1)."""
        code = self.sport.SpeedLevel(self.speed_level)
        self.level_set = code == 0
        if code == 0:
            self.get_logger().info(f'speed level {self.speed_level}')
        elif not quiet:
            self.get_logger().warn(
                f'speed level {self.speed_level}: sport service code {code}',
                throttle_duration_sec=30.0,
            )
        return code

    def stand_up(self, request, response):
        """Stand, then balance stand: the mode in which Move walks, at speed_level."""
        code = self.sport.StandUp()
        if code == 0:
            time.sleep(self.stand_settle)
            code = self.sport.BalanceStand()
        if code == 0:
            self.set_speed_level()
        return self._answer(response, 'stand_up', code)

    def sit_down(self, request, response):
        """Stop and lie down."""
        self.gate.halt()
        self.sport.StopMove()
        return self._answer(response, 'sit_down', self.sport.StandDown())

    def emergency_stop(self, request, response):
        """Damp every motor."""
        self.gate.halt()
        return self._answer(response, 'emergency_stop', self.sport.Damp())

    def _answer(self, response, name: str, code: int):
        response.success = code == 0
        response.message = name if code == 0 else f'{name}: sport service code {code}'
        if code != 0:
            self.get_logger().warn(response.message)
        return response

    def close(self):
        """Stop a robot that is still moving."""
        if self.gate.moving:
            self.sport.StopMove()


def main(args=None):
    """Run the driver."""
    rclpy.init(args=args)
    node = Go2DriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
