"""What the driver uses of unitree_sdk2py still exists (CI installs the pinned SDK)."""

from __future__ import annotations

import pytest

sdk = pytest.importorskip('unitree_sdk2py', reason='unitree_sdk2py not installed')


def test_sport_client_calls():
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    for name in ('Init', 'SetTimeout', 'Move', 'StopMove', 'StandUp', 'StandDown',
                 'BalanceStand', 'Damp', 'SpeedLevel'):  # fmt: skip
        assert callable(getattr(SportClient, name, None)), name


def test_state_fields():
    from unitree_sdk2py.idl.default import (
        unitree_go_msg_dds__LowState_,
        unitree_go_msg_dds__SportModeState_,
    )

    sport = unitree_go_msg_dds__SportModeState_()
    assert len(sport.position) == 3
    assert len(sport.velocity) == 3
    assert len(sport.imu_state.rpy) == 3
    assert len(sport.imu_state.quaternion) == 4
    for name in ('yaw_speed', 'mode', 'gait_type', 'error_code', 'body_height'):
        assert hasattr(sport, name), name
    low = unitree_go_msg_dds__LowState_()
    for name in ('soc', 'current'):
        assert hasattr(low.bms_state, name), name
    assert hasattr(low, 'power_v')


def test_channel_api():
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

    assert callable(ChannelFactoryInitialize)
    assert callable(getattr(ChannelSubscriber, 'Init', None))
