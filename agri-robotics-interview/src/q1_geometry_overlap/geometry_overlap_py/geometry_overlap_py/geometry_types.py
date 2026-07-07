"""Geometry algorithms for the Q1 exercise.

This exercise is about using a mature, real geospatial library correctly
rather than hand-rolling computational-geometry algorithms yourself. We use
Shapely (https://shapely.readthedocs.io/, requires shapely>=2.0 -- see the
top-level README for install instructions) for the actual math -- your job
is to build valid Shapely geometries from the app's plain data and call the
right library functions on them.

All polygons are assumed to be "simple" (non-self-intersecting).

Kept free of any rclpy/message dependency so you can unit test in isolation
(see test/test_geometry_types.py) without spinning up a ROS graph. This is
also the type ros_conversions.py converts geometry_msgs/Polygon to/from --
you don't need to touch that file.
"""
from dataclasses import dataclass
from typing import List

from shapely.geometry import Polygon as ShapelyPolygon
from shapely.strtree import STRtree


@dataclass
class Point2D:
    x: float = 0.0
    y: float = 0.0


# An ordered list of vertices. Do not repeat the first vertex at the end --
# Shapely's Polygon constructor handles closing the ring itself.
Polygon2D = List[Point2D]


# -----------------------------------------------------------------------
# GIVEN -- a cheap prefilter you may find useful (Shapely's own
# intersects()/intersection() already handle the general case correctly
# without this, but see the performance discussion in the README).
# -----------------------------------------------------------------------

@dataclass
class BoundingBox:
    min_x: float
    min_y: float
    max_x: float
    max_y: float


def compute_bounding_box(poly: Polygon2D) -> BoundingBox:
    xs = [p.x for p in poly]
    ys = [p.y for p in poly]
    return BoundingBox(min(xs), min(ys), max(xs), max(ys))


def bounding_boxes_overlap(a: BoundingBox, b: BoundingBox) -> bool:
    return (
        a.min_x <= b.max_x and a.max_x >= b.min_x
        and a.min_y <= b.max_y and a.max_y >= b.min_y
    )


# -----------------------------------------------------------------------
# TODO (core) -- implement these.
# -----------------------------------------------------------------------

def to_shapely_polygon(poly: Polygon2D) -> ShapelyPolygon:
    """Build a Shapely Polygon from `poly`.

    Unlike Boost.Geometry, Shapely doesn't care about winding direction and
    closes the ring for you -- but it's still worth checking `.is_valid` on
    the result (a self-intersecting or degenerate input would produce an
    invalid geometry) if you have time.
    """
    # TODO: implement.
    return ShapelyPolygon()


def polygons_intersect(a: Polygon2D, b: Polygon2D) -> bool:
    """Do polygons `a` and `b` overlap at all (touching, containment, or
    partial overlap all count as true)?

    Use ShapelyPolygon.intersects() -- don't reimplement the geometric test
    yourself.
    """
    # TODO: implement.
    return False


def polygon_intersection_area(a: Polygon2D, b: Polygon2D) -> float:
    """Area (m^2) of the overlap between `a` and `b`.

    Use ShapelyPolygon.intersection() and .area. Returns 0.0 if they don't
    overlap (an empty intersection's .area is already 0.0, so you likely
    don't need a special case).
    """
    # TODO: implement.
    return 0.0


# -----------------------------------------------------------------------
# TODO (stretch) -- only if time allows.
# -----------------------------------------------------------------------

def find_intersecting_zones(footprint: Polygon2D, zones: List[Polygon2D]) -> List[int]:
    """Return the indices (into `zones`) of every zone that `footprint`
    overlaps.

    A naive implementation calls polygons_intersect() once per zone -- O(n)
    exact geometry tests. Instead, build a shapely.strtree.STRtree over the
    zones (a broad-phase spatial index over bounding boxes) and query it
    with `footprint` first; STRtree.query() returns only the *candidates*
    whose bounding box overlaps (shapely>=2.0 returns their indices
    directly), so you should still confirm each candidate with an exact
    .intersects() check before including it in the result.
    """
    # TODO: implement.
    return []
