"""Plain, ROS-independent types for the safety monitor.

Keeping these free of any rclpy/message dependency means you can unit test
them in isolation (see test/test_safety_types.py) without spinning up a ROS
graph.
"""
from dataclasses import dataclass
from enum import IntEnum
from typing import List


@dataclass
class Point2D:
    x: float = 0.0
    y: float = 0.0


Polygon2D = List[Point2D]


# -----------------------------------------------------------------------
# GIVEN
# -----------------------------------------------------------------------

def point_in_polygon(pt: Point2D, poly: Polygon2D) -> bool:
    """Same technique as the Q1 geometry-overlap exercise.

    Implemented here so this exercise is self-contained; if a candidate
    already solved Q1, feel free to let them paste in their own
    implementation instead.
    """
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        pi, pj = poly[i], poly[j]
        straddles = (pi.y > pt.y) != (pj.y > pt.y)
        if straddles:
            x_intersect = (pj.x - pi.x) * (pt.y - pi.y) / (pj.y - pi.y) + pi.x
            if pt.x < x_intersect:
                inside = not inside
        j = i
    return inside


@dataclass
class Range:
    min: float
    max: float

    def contains(self, value: float) -> bool:
        return self.min <= value <= self.max


class Severity(IntEnum):
    OK = 0
    WARNING = 1
    CRITICAL = 2


# -----------------------------------------------------------------------
# TODO (core) -- design and implement this class. The public interface
# below (constructor + two method signatures) is fixed, because
# safety_monitor_node.py and test/test_safety_types.py are written against
# it -- but everything inside the class body is yours to design.
# -----------------------------------------------------------------------

class ViolationTracker:
    """Tracks, per named parameter (e.g. "engine_temp_c", "geofence"), how
    many *consecutive* times in a row it has been reported out-of-range, and
    turns that streak into a Severity.

    Why debounce at all? Real sensors are noisy -- a single out-of-range
    reading is often just noise, not a real fault. Requiring several
    consecutive violations before escalating avoids nuisance E-STOPs while
    still reacting quickly to a real, sustained problem.

    Required behavior:
      - record(name, True)  [in range]     -> reset that name's streak to
        0, return Severity.OK.
      - record(name, False) [out of range] -> increment that name's streak,
        then return:
          Severity.OK        if streak < warning_threshold
          Severity.WARNING   if warning_threshold <= streak < critical_threshold
          Severity.CRITICAL  if streak >= critical_threshold
      - Each name's streak is independent of every other name's.
      - A name that has never been recorded should behave as if its streak
        is 0.

    Performance note: record() will be called once per monitored parameter
    on every timer tick (see safety_monitor_node.py) -- e.g. 5 parameters x
    10 Hz today, but think about how your choice of container/lookup would
    hold up at 50+ parameters and 100 Hz, and in Python specifically, what
    that means for per-tick object allocation / GIL contention if you were
    to move this to a multi-threaded executor.
    """

    def __init__(self, warning_threshold: int, critical_threshold: int):
        self._warning_threshold = warning_threshold
        self._critical_threshold = critical_threshold
        # TODO: add whatever state you need (e.g. a dict from name to
        # current streak count) to implement record() and
        # consecutive_violations().

    def record(self, name: str, in_range: bool) -> Severity:
        # TODO: implement the streak-tracking / thresholding behavior
        # described above.
        return Severity.OK

    def consecutive_violations(self, name: str) -> int:
        """Current consecutive-violation count for `name` (0 if never
        recorded or not currently violating)."""
        # TODO: implement.
        return 0
