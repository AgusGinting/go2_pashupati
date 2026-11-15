"""The Go2's own DDS through unitree_sdk2py: the sport service and the state topics.

The SDK runs its own Cyclone DDS (domain 0 on the robot's interface), apart from the ROS
middleware, so the Pashupati stack keeps its RMW, domain and ROS_LOCALHOST_ONLY.
"""

from __future__ import annotations

from typing import Callable

# The low-frequency copies (odom came at 40 Hz on the Go2 EDU, the full one ~300 Hz): enough
# for odom (>= 10 Hz) and cheap in Python
SPORT_STATE = 'rt/lf/sportmodestate'
LOW_STATE = 'rt/lf/lowstate'


def connect(interface: str, on_sport_state: Callable, on_low_state: Callable,
            timeout: float = 3.0):  # fmt: skip
    """Open the robot's DDS on interface; return (SportClient, subscribers to keep alive)."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

    ChannelFactoryInitialize(0, interface or None)
    subscribers = []
    for topic, kind, handler in ((SPORT_STATE, SportModeState_, on_sport_state),
                                 (LOW_STATE, LowState_, on_low_state)):  # fmt: skip
        subscriber = ChannelSubscriber(topic, kind)
        subscriber.Init(handler, 1)
        subscribers.append(subscriber)
    sport = SportClient()
    sport.SetTimeout(timeout)
    sport.Init()
    return sport, subscribers
