"""GIVEN -- helpers to convert between geometry_msgs/Polygon and the plain
Polygon2D type used by geometry_types.py. Not part of the exercise; provided
so both nodes can share the same conversion logic.
"""
from geometry_msgs.msg import Point32, Polygon

from geometry_overlap_py.geometry_types import Point2D, Polygon2D


def from_msg(msg: Polygon) -> Polygon2D:
    return [Point2D(p.x, p.y) for p in msg.points]


def to_msg(poly: Polygon2D) -> Polygon:
    msg = Polygon()
    msg.points = [Point32(x=float(p.x), y=float(p.y), z=0.0) for p in poly]
    return msg
