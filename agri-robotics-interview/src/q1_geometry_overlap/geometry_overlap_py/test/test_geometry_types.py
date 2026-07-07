"""Plain pytest unit tests for geometry_types. No ROS graph needed:

    python3 -m pytest test/test_geometry_types.py -v

Requires shapely>=2.0 (`pip install "shapely>=2.0"`) -- see the top-level
README.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_overlap_py.geometry_types import (  # noqa: E402
    Point2D,
    find_intersecting_zones,
    polygon_intersection_area,
    polygons_intersect,
)


def square(cx: float, cy: float, half_size: float):
    return [
        Point2D(cx - half_size, cy - half_size),
        Point2D(cx + half_size, cy - half_size),
        Point2D(cx + half_size, cy + half_size),
        Point2D(cx - half_size, cy + half_size),
    ]


def test_polygons_intersect_overlapping_and_far():
    a = square(0.0, 0.0, 1.0)
    b = square(1.5, 0.0, 1.0)
    c = square(10.0, 10.0, 1.0)
    d = square(0.0, 0.0, 0.25)

    assert polygons_intersect(a, b)
    assert not polygons_intersect(a, c)
    assert polygons_intersect(a, d)


def test_polygons_intersect_no_vertex_inside():
    a = square(0.0, 0.0, 1.0)
    cross_bar = [
        Point2D(-5.0, -0.1), Point2D(5.0, -0.1), Point2D(5.0, 0.1), Point2D(-5.0, 0.1),
    ]
    assert polygons_intersect(a, cross_bar)


def test_polygon_intersection_area():
    a = [Point2D(0.0, 0.0), Point2D(1.0, 0.0), Point2D(1.0, 1.0), Point2D(0.0, 1.0)]
    b = [Point2D(0.5, 0.0), Point2D(1.5, 0.0), Point2D(1.5, 1.0), Point2D(0.5, 1.0)]
    area = polygon_intersection_area(a, b)
    assert math.isclose(area, 0.5, abs_tol=1e-6)


def test_polygon_intersection_area_no_overlap():
    a = [Point2D(0.0, 0.0), Point2D(1.0, 0.0), Point2D(1.0, 1.0), Point2D(0.0, 1.0)]
    c = square(10.0, 10.0, 1.0)
    assert math.isclose(polygon_intersection_area(a, c), 0.0, abs_tol=1e-9)


def test_polygon_intersection_area_full_containment():
    a = [Point2D(0.0, 0.0), Point2D(1.0, 0.0), Point2D(1.0, 1.0), Point2D(0.0, 1.0)]
    small = square(0.5, 0.5, 0.1)  # 0.2 x 0.2, area 0.04
    area = polygon_intersection_area(a, small)
    assert math.isclose(area, 0.04, abs_tol=1e-6)


def test_find_intersecting_zones_stretch():
    footprint = square(0.0, 0.0, 1.0)
    zones = [
        square(0.5, 0.5, 0.2),    # overlaps footprint
        square(10.0, 10.0, 1.0),  # far away
        square(-0.5, -0.5, 0.2),  # overlaps footprint
        square(5.0, 0.0, 1.0),    # far away
    ]
    hits = find_intersecting_zones(footprint, zones)
    assert sorted(hits) == [0, 2]
