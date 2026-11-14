from __future__ import annotations

import math

from go2_bringup.core import (
    IDLE,
    MOVE,
    STOP,
    CommandGate,
    battery_fraction,
    body_velocity,
    clamp,
)
import pytest


def gate():
    return CommandGate(0.5, (1.0, 0.5, 1.5))


def test_clamp():
    assert clamp(2.0, 1.0) == 1.0
    assert clamp(-2.0, 1.0) == -1.0
    assert clamp(0.3, 1.0) == 0.3


@pytest.mark.parametrize(
    ('vx', 'vy', 'yaw', 'expected'),
    [
        (1.0, 0.0, 0.0, (1.0, 0.0)),
        (0.0, 1.0, math.pi / 2, (1.0, 0.0)),  # facing +y, moving +y: forward
        (1.0, 0.0, math.pi / 2, (0.0, -1.0)),  # facing +y, moving +x: to the right
        (-1.0, 0.0, math.pi, (1.0, 0.0)),
    ],
)
def test_body_velocity(vx, vy, yaw, expected):
    assert body_velocity(vx, vy, yaw) == pytest.approx(expected, abs=1e-9)


def test_battery_fraction():
    assert battery_fraction(67) == pytest.approx(0.67)
    assert battery_fraction(250) == 1.0
    assert battery_fraction(-1) == 0.0


def test_moves_while_fresh_then_stops_once():
    g = gate()
    assert g.tick(0.0) == (IDLE, (0.0, 0.0, 0.0))
    assert g.update(0.3, 0.0, 0.2, now=1.0)
    assert g.tick(1.2) == (MOVE, (0.3, 0.0, 0.2))
    assert g.tick(1.5) == (MOVE, (0.3, 0.0, 0.2))  # 0.5 s old: still fresh
    assert g.tick(1.51)[0] == STOP
    assert g.tick(1.6)[0] == IDLE


def test_zero_command_is_a_stop_not_a_move():
    g = gate()
    g.update(0.0, 0.0, 0.0, now=0.0)
    assert g.tick(0.1)[0] == IDLE  # nothing sent to a robot that is not moving
    g.update(0.2, 0.0, 0.0, now=0.2)
    assert g.tick(0.3)[0] == MOVE
    g.update(0.0, 0.0, 0.0, now=0.4)
    assert g.tick(0.45)[0] == STOP
    assert g.tick(0.5)[0] == IDLE


def test_limits_clamp_each_axis():
    g = gate()
    g.update(3.0, -2.0, 9.0, now=0.0)
    assert g.tick(0.0) == (MOVE, (1.0, -0.5, 1.5))


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_command_is_dropped(bad):
    g = gate()
    g.update(0.3, 0.0, 0.0, now=0.0)
    assert not g.update(bad, 0.0, 0.0, now=0.1)
    assert g.tick(0.2) == (MOVE, (0.3, 0.0, 0.0))  # the last good one, until it times out
    assert g.tick(0.6)[0] == STOP


def test_halt_forgets_the_command_without_a_stop():
    g = gate()
    g.update(0.3, 0.0, 0.0, now=0.0)
    g.tick(0.1)
    g.halt()
    assert g.tick(0.2)[0] == IDLE
