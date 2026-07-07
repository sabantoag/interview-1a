"""Plain, ROS-independent geometry primitives.

Keeping these free of any rclpy/message dependency means you can unit test
them in isolation (see test/test_geometry_types.py) without spinning up a
ROS graph.

All polygons are assumed to be "simple" (non-self-intersecting). Some
functions additionally assume the polygon is convex and wound
counter-clockwise -- that assumption is called out explicitly below.

A polygon is represented as a list of Point2D. The edge from poly[i] to
poly[i+1] (wrapping around) is implicit -- do not repeat the first vertex at
the end.
"""
from dataclasses import dataclass
from typing import List


@dataclass
class Point2D:
    x: float = 0.0
    y: float = 0.0


Polygon2D = List[Point2D]


@dataclass
class BoundingBox:
    min_x: float
    min_y: float
    max_x: float
    max_y: float


# -----------------------------------------------------------------------
# GIVEN -- implemented for you as an example of the expected style/testing
# pattern. You should not need to change these.
# -----------------------------------------------------------------------

def compute_bounding_box(poly: Polygon2D) -> BoundingBox:
    """Axis-aligned bounding box of a polygon."""
    xs = [p.x for p in poly]
    ys = [p.y for p in poly]
    return BoundingBox(min(xs), min(ys), max(xs), max(ys))


def bounding_boxes_overlap(a: BoundingBox, b: BoundingBox) -> bool:
    """Do two bounding boxes overlap (touching counts as overlap)?"""
    return (
        a.min_x <= b.max_x and a.max_x >= b.min_x
        and a.min_y <= b.max_y and a.max_y >= b.min_y
    )


def segments_intersect(p1: Point2D, p2: Point2D, p3: Point2D, p4: Point2D) -> bool:
    """Do segments (p1,p2) and (p3,p4) intersect (including endpoint touches)?"""

    def cross(o: Point2D, a: Point2D, b: Point2D) -> float:
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)

    def sign(v: float) -> int:
        if v > 1e-12:
            return 1
        if v < -1e-12:
            return -1
        return 0

    d1 = sign(cross(p3, p4, p1))
    d2 = sign(cross(p3, p4, p2))
    d3 = sign(cross(p1, p2, p3))
    d4 = sign(cross(p1, p2, p4))

    if d1 != d2 and d3 != d4:
        return True

    def on_segment(a: Point2D, b: Point2D, p: Point2D) -> bool:
        return (
            min(a.x, b.x) <= p.x <= max(a.x, b.x)
            and min(a.y, b.y) <= p.y <= max(a.y, b.y)
        )

    if d1 == 0 and on_segment(p3, p4, p1):
        return True
    if d2 == 0 and on_segment(p3, p4, p2):
        return True
    if d3 == 0 and on_segment(p1, p2, p3):
        return True
    if d4 == 0 and on_segment(p1, p2, p4):
        return True
    return False


def polygon_area(poly: Polygon2D) -> float:
    """Unsigned area of a simple polygon via the shoelace formula."""
    if len(poly) < 3:
        return 0.0
    total = 0.0
    n = len(poly)
    for i in range(n):
        p1 = poly[i]
        p2 = poly[(i + 1) % n]
        total += (p1.x * p2.y) - (p2.x * p1.y)
    return abs(total) * 0.5


# -----------------------------------------------------------------------
# TODO (core) -- implement these.
# -----------------------------------------------------------------------

def point_in_polygon(pt: Point2D, poly: Polygon2D) -> bool:
    """Is `pt` inside `poly`? `poly` may be non-convex.

    Points exactly on an edge may be treated either way (not tested); just
    be consistent.

    Suggested approach: ray casting (a.k.a. even-odd rule) -- cast a ray
    from `pt` in any fixed direction (e.g. +x) and count how many polygon
    edges it crosses. Odd count => inside.
    """
    # TODO: implement ray casting (even-odd rule).
    return False


def polygons_intersect(a: Polygon2D, b: Polygon2D) -> bool:
    """Do polygons `a` and `b` overlap at all?

    Touching, one fully containing the other, or partial overlap all count
    as true. `a` and `b` may be non-convex.

    Suggested approach, cheapest checks first:
      1. bounding_boxes_overlap() quick reject
      2. any vertex of `a` inside `b`, or any vertex of `b` inside `a`
         (point_in_polygon) -- catches full containment and simple overlaps
      3. any edge of `a` intersects any edge of `b` (segments_intersect) --
         catches overlaps where no vertex of either polygon is inside the
         other (e.g. a "+" shape crossing through a square)
    """
    # TODO: bbox quick reject -> vertex-in-polygon -> edge intersection.
    return False


# -----------------------------------------------------------------------
# TODO (stretch) -- only if time allows.
# -----------------------------------------------------------------------

def clip_convex_polygon(subject: Polygon2D, clip_convex: Polygon2D) -> Polygon2D:
    """Clip `subject` against convex, CCW-wound `clip_convex`.

    Classic Sutherland-Hodgman clipping: walk each edge of clip_convex and
    keep only the part of `subject` on the "inside" of that edge. Return an
    empty list if there is no overlap.
    """
    # TODO: Sutherland-Hodgman clipping.
    return []


# -----------------------------------------------------------------------
# GIVEN -- built on top of clip_convex_polygon(), once you implement it.
# -----------------------------------------------------------------------

def polygon_intersection_area(a: Polygon2D, b: Polygon2D) -> float:
    """Area of the overlap between `a` and `b`, assuming `b` is convex/CCW."""
    clipped = clip_convex_polygon(a, b)
    return polygon_area(clipped)
