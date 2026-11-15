"""go2_driver_node in-process against a stand-in sport service (no robot, no SDK)."""

from __future__ import annotations

import math
import time
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from go2_bringup.go2_driver_node import Go2DriverNode
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger


class FakeSport:
    """SportClient's methods, recorded; code is what the calls return."""

    def __init__(self):
        self.calls = []
        self.code = 0

    def __getattr__(self, name):
        def call(*args):
            self.calls.append((name, *args))
            return self.code

        return call

    def names(self):
        return [c[0] for c in self.calls]


def sport_state(yaw=math.pi / 2):
    return SimpleNamespace(
        position=[1.0, 2.0, 0.3], velocity=[0.0, 0.5, 0.0], yaw_speed=0.2,
        imu_state=SimpleNamespace(rpy=[0.0, 0.0, yaw], quaternion=[1.0, 0.0, 0.0, 0.0]),
        mode=1, gait_type=1, error_code=0, body_height=0.315,
    )  # fmt: skip


def low_state():
    return SimpleNamespace(power_v=29.76, bms_state=SimpleNamespace(soc=67, current=-2939))


@pytest.fixture
def rig():
    rclpy.init()
    sport = FakeSport()
    node = Go2DriverNode(sport=sport)
    node.stand_settle = 0.0
    assert sport.calls == [('SpeedLevel', 1)]  # fast, set once on connect
    sport.calls.clear()
    helper = rclpy.create_node('go2_test_helper')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    received = {'odom': [], 'battery': [], 'diagnostics': []}
    for topic, kind in (('odom', Odometry), ('battery', BatteryState),
                        ('diagnostics', DiagnosticArray)):  # fmt: skip
        helper.create_subscription(kind, topic, received[topic].append, 10)
    cmd = helper.create_publisher(Twist, 'cmd_vel', 10)

    def spin(seconds=0.0, until=None):
        end = time.monotonic() + (seconds or 3.0)
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if until is not None and until():
                return True
        return until is None

    yield SimpleNamespace(node=node, sport=sport, cmd=cmd, received=received, spin=spin)
    executor.shutdown()
    node.destroy_node()
    helper.destroy_node()
    rclpy.shutdown()


def twist(vx=0.0, vy=0.0, wz=0.0):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
    return msg


def test_cmd_vel_moves_then_watchdog_stops_once(rig):
    assert rig.spin(until=lambda: rig.cmd.get_subscription_count() > 0)
    rig.cmd.publish(twist(0.3, 0.1, 0.2))
    assert rig.spin(until=lambda: 'Move' in rig.sport.names())
    assert rig.sport.calls[0] == ('Move', 0.3, 0.1, 0.2)
    started = time.monotonic()
    assert rig.spin(until=lambda: 'StopMove' in rig.sport.names())
    assert time.monotonic() - started < 0.7  # cmd_timeout 0.5 s, then one tick
    rig.spin(0.3)
    assert rig.sport.names().count('StopMove') == 1


def test_zero_cmd_vel_sends_nothing(rig):
    assert rig.spin(until=lambda: rig.cmd.get_subscription_count() > 0)
    for _ in range(5):
        rig.cmd.publish(twist())
        rig.spin(0.05)
    assert rig.sport.calls == []


def test_odom_in_base_link(rig):
    rig.node.on_sport_state(sport_state())
    assert rig.spin(until=lambda: rig.received['odom'])
    odom = rig.received['odom'][0]
    assert (odom.header.frame_id, odom.child_frame_id) == ('odom', 'base_link')
    # facing +y and moving +y in Unitree's odometry frame: forward
    assert odom.twist.twist.linear.x == pytest.approx(0.5)
    assert odom.twist.twist.linear.y == pytest.approx(0.0, abs=1e-6)
    assert odom.twist.twist.angular.z == pytest.approx(0.2)
    assert odom.pose.pose.position.x == pytest.approx(1.0)
    rig.spin(0.2)
    assert len(rig.received['odom']) == 1  # one message per state, not per tick


def test_battery_and_diagnostics(rig):
    rig.node.on_low_state(low_state())
    rig.node.on_sport_state(sport_state())
    assert rig.spin(until=lambda: rig.received['battery'] and rig.received['diagnostics'])
    battery = rig.received['battery'][0]
    assert battery.percentage == pytest.approx(0.67)
    assert battery.voltage == pytest.approx(29.76)
    assert battery.current == pytest.approx(-2.939)
    status = rig.received['diagnostics'][0].status[0]
    assert status.level == DiagnosticStatus.OK
    assert {kv.key for kv in status.values} >= {'mode', 'error_code', 'soc'}


def test_diagnostics_error_without_robot(rig):
    assert rig.spin(until=lambda: rig.received['diagnostics'])
    assert rig.received['diagnostics'][0].status[0].level == DiagnosticStatus.ERROR
    assert rig.received['battery'] == []


def call(rig, name):
    return getattr(rig.node, name)(Trigger.Request(), Trigger.Response())


def test_services(rig):
    assert call(rig, 'stand_up').success
    assert rig.sport.calls == [('StandUp',), ('BalanceStand',), ('SpeedLevel', 1)]
    rig.sport.calls.clear()
    assert call(rig, 'sit_down').success
    assert rig.sport.names() == ['StopMove', 'StandDown']
    rig.sport.calls.clear()
    assert call(rig, 'emergency_stop').success
    assert rig.sport.names() == ['Damp']


def test_emergency_stop_halts_motion(rig):
    rig.node.gate.update(0.3, 0.0, 0.0, time.monotonic())
    rig.node.on_tick()
    call(rig, 'emergency_stop')
    rig.sport.calls.clear()
    rig.node.on_tick()
    assert rig.sport.calls == []  # no Move, and no StopMove after the Damp


def test_speed_level_is_set_whenever_the_robot_stands_up(rig):
    lying, standing = sport_state(), sport_state()
    lying.body_height = 0.08
    rig.node.on_sport_state(lying)
    rig.node.on_status()
    assert rig.sport.calls == []  # lying: nothing asked, the next stand-up sets it
    rig.sport.code = -1  # refused once (still getting up)
    rig.node.on_sport_state(standing)
    rig.node.on_status()
    rig.sport.code = 0
    rig.node.on_status()
    rig.node.on_status()  # set: not asked again while it stands
    assert rig.sport.calls == [('SpeedLevel', 1), ('SpeedLevel', 1)]
    rig.sport.calls.clear()
    rig.node.on_sport_state(lying)
    rig.node.on_status()
    rig.node.on_sport_state(standing)  # stood up by the Unitree remote
    rig.node.on_status()
    assert rig.sport.calls == [('SpeedLevel', 1)]


def test_service_failure_is_reported(rig):
    rig.sport.code = 3104
    response = call(rig, 'stand_up')
    assert not response.success
    assert '3104' in response.message
    assert rig.sport.names() == ['StandUp']  # no BalanceStand after a failed StandUp
