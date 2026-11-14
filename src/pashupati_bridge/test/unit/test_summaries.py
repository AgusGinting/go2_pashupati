from __future__ import annotations

from pashupati_bridge.core.summaries import (
    handheld_health,
    inspection_summary,
    mission_text,
    people_summary,
    trip_summary,
)


def test_inspection_summary():
    status = {'enabled': True, 'state': 'running', 'model': 'm', 'interval_sec': -1,
              'inspections': 3, 'reports': 1, 'queue': 2, 'queue_max': 50,
              'last': {'category': 'trash', 'reported': True, 'latency_ms': 900, 'at': 95.0,
                       'waited_sec': 1.6, 'source': 'stop_point'}}  # fmt: skip
    summary = inspection_summary(status, False, 100.0)
    assert summary['on'] and summary['interval'] == -1.0 and summary['count'] == 3
    assert summary['last'] == {'cat': 'trash', 'reported': True, 'ms': 900, 'age': 5,
                               'desc': '', 'source': 'stop_point', 'waited': 2}  # fmt: skip
    assert inspection_summary(status, True, 100.0)['state'] == 'offline'


def test_people_boxes_only_while_fresh():
    status = {'enabled': True, 'state': 'running', 'count': 1, 'at': 99.0,
              'boxes': [[0.1, 0.2, 0.3, 0.4, 0.9], [1, 2], 'x']}  # fmt: skip
    assert people_summary(status, False, 100.0)['boxes'] == [[0.1, 0.2, 0.3, 0.4, 0.9]]
    assert people_summary(status, False, 103.0)['boxes'] == []


def test_health_for_the_handheld():
    health = {'ok': False, 'issues': [str(i) for i in range(9)], 'cpu_percent': 23.4,
              'memory_used_gb': 5.12, 'gpu_percent': None}  # fmt: skip
    out = handheld_health(health)
    assert out['ok'] is False and len(out['issues']) == 5
    assert (out['cpu'], out['ram_used'], out['gpu']) == (23.0, 5.1, None)


def test_mission_text():
    assert mission_text('PAUSED', ['operator', 'estop'], 3, 10) == 'paused: operator, estop'
    assert mission_text('FOLLOWING', [], 3, 10) == 'waypoint 4/10'
    assert mission_text('NAV_INACTIVE', [], 0, 10) == ''


def test_trip_for_the_handheld():
    out = trip_summary(1234.567, 3725.9, 0.4321, 0.38, 0.912, 1_794_550_000.7)
    assert out == {'m': 1234.6, 's': 3726, 'v': 0.43, 'avg': 0.38, 'max': 0.91,
                   'since': 1_794_550_001}  # fmt: skip
    assert trip_summary(float('nan'), -1.0, float('inf'), 0.0, 0.0, 0.0)['m'] == 0.0
