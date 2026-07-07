"""Plain pytest unit tests for safety_types. No ROS graph needed:

    python3 -m pytest test/test_safety_types.py -v
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from safety_monitor_py.safety_types import (  # noqa: E402
    Point2D,
    Severity,
    ViolationTracker,
    point_in_polygon,
)


def test_point_in_polygon_geofence():
    field = [Point2D(-20, -15), Point2D(20, -15), Point2D(20, 15), Point2D(-20, 15)]
    assert point_in_polygon(Point2D(0, 0), field)
    assert not point_in_polygon(Point2D(100, 100), field)


def test_violation_tracker_thresholds():
    tracker = ViolationTracker(warning_threshold=3, critical_threshold=8)

    assert tracker.record('engine_temp_c', True) == Severity.OK

    assert tracker.record('engine_temp_c', False) == Severity.OK  # 1st
    assert tracker.record('engine_temp_c', False) == Severity.OK  # 2nd

    sev = Severity.OK
    for _ in range(5):
        sev = tracker.record('engine_temp_c', False)  # 3rd..7th
    assert sev == Severity.WARNING

    assert tracker.record('engine_temp_c', False) == Severity.CRITICAL  # 8th

    assert tracker.record('engine_temp_c', True) == Severity.OK  # recovers
    assert tracker.record('engine_temp_c', False) == Severity.OK  # streak restarts


def test_violation_tracker_independent_names():
    tracker = ViolationTracker(warning_threshold=3, critical_threshold=8)
    for _ in range(8):
        tracker.record('ground_speed_mps', False)
    assert tracker.record('engine_temp_c', False) == Severity.OK
