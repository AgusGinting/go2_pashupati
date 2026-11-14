"""Fixtures of the teleop tests."""

from __future__ import annotations

import pytest
from teleop_rig import Rig


@pytest.fixture
def rig():
    """Provide a running teleop node."""
    r = Rig()
    yield r
    r.close()


@pytest.fixture
def video_rig():
    """Provide a teleop node on a robot with WebRTC video (MediaMTX on port 8890)."""
    r = Rig(['video_port:=8890'])
    yield r
    r.close()


@pytest.fixture
def launched_rig():
    """Provide a teleop node with the remaps and topics of bridge.launch.py (robot_ns l1w)."""
    r = Rig(
        ['driver_diagnostics_topic:=/l1w/diagnostics'],
        [('battery', '/l1w/battery'), ('cmd_vel', '/l1w/cmd_vel')],
    )
    yield r
    r.close()
