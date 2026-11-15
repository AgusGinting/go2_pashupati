"""What the Go2 driver decides, without ROS or the Unitree SDK (unit tested)."""

from __future__ import annotations

import math

# What the command loop sends the robot on a tick
MOVE = 'move'
STOP = 'stop'
IDLE = 'idle'


def clamp(value: float, limit: float) -> float:
    """Return value limited to [-limit, limit]."""
    return max(-limit, min(limit, value))


def body_velocity(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    """Rotate the sport-mode velocity (Unitree's odometry frame) into base_link."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


ZERO = 0.01  # m/s, rad/s: below it a command axis counts as zero (no lift)


def lift(vx: float, vy: float, wz: float, min_speed: float, min_wz: float):
    """Raise a command the Go2 would not walk to its deadband, keeping the direction.

    The walking speed (vx, vy together, so a slow sideways correction while walking forward is
    left alone) goes up to min_speed; a turn in place up to min_wz. 0 turns either off.
    """
    speed = math.hypot(vx, vy)
    if min_speed > 0.0 and ZERO < speed < min_speed:
        vx, vy = vx * min_speed / speed, vy * min_speed / speed
    if min_wz > 0.0 and speed <= ZERO < abs(wz) < min_wz:
        wz = math.copysign(min_wz, wz)
    return vx, vy, wz


def battery_fraction(soc: int) -> float:
    """Return the BMS state of charge (0-100 %) as BatteryState.percentage (0-1)."""
    return min(max(soc, 0), 100) / 100.0


class CommandGate:
    """The last cmd_vel and its age: move while it is fresh and not zero, then stop once.

    A zero command is a stop, not a Move(0, 0, 0): a robot lying down or damped after an e-stop
    gets nothing while the control mux publishes zeros.
    """

    def __init__(self, timeout: float, limits: tuple[float, float, float],
                 deadband: tuple[float, float] = (0.0, 0.0)):  # fmt: skip
        self.timeout = timeout
        self.limits = limits
        self.deadband = deadband  # (min_speed, min_wz) for lift(); zeros: off
        self._cmd = (0.0, 0.0, 0.0)
        self._stamp: float | None = None
        self.moving = False

    def update(self, vx: float, vy: float, wz: float, now: float) -> bool:
        """Take a command; a non-finite one is dropped (False) and the robot stops on timeout."""
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            return False
        vx, vy, wz = lift(vx, vy, wz, *self.deadband)
        lx, ly, lw = self.limits
        self._cmd = (clamp(vx, lx), clamp(vy, ly), clamp(wz, lw))
        self._stamp = now
        return True

    def halt(self):
        """Forget the command (after an e-stop or sitting down): nothing moves until a new one."""
        self._stamp = None
        self.moving = False

    def tick(self, now: float) -> tuple[str, tuple[float, float, float]]:
        """Return (MOVE, cmd), (STOP, zeros) once after the last motion, else (IDLE, zeros)."""
        fresh = self._stamp is not None and now - self._stamp <= self.timeout
        if fresh and any(self._cmd):
            self.moving = True
            return MOVE, self._cmd
        if self.moving:
            self.moving = False
            return STOP, (0.0, 0.0, 0.0)
        return IDLE, (0.0, 0.0, 0.0)
