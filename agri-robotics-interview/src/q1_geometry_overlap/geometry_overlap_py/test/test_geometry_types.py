"""Plain pytest unit tests for geometry_types. No ROS graph needed:

    python3 -m pytest test/test_geometry_types.py -v

or, once the package is built:

    colcon test --packages-select geometry_overlap_py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_overlap_py.geometry_types import (  # noqa: E402
    Point2D,
    point_in_polygon,
    polygon_area,
    polygon_intersection_area,
    polygons_intersect,
)


def unit_square(cx: float, cy: float, half_size: float):
    return [
        Point2D(cx - half_size, cy - half_size),
        Point2D(cx + half_size, cy - half_size),
        Point2D(cx + half_size, cy + half_size),
        Point2D(cx - half_size, cy + half_size),
    ]


def test_point_in_polygon_square():
    square = unit_square(0.0, 0.0, 1.0)
    assert point_in_polygon(Point2D(0.0, 0.0), square)
    assert point_in_polygon(Point2D(0.5, 0.5), square)
    assert not point_in_polygon(Point2D(5.0, 5.0), square)
    assert not point_in_polygon(Point2D(0.0, 2.0), square)


def test_point_in_polygon_triangle():
    triangle = [Point2D(0.0, 0.0), Point2D(4.0, 0.0), Point2D(0.0, 4.0)]
    assert point_in_polygon(Point2D(1.0, 1.0), triangle)
    assert not point_in_polygon(Point2D(3.0, 3.0), triangle)


def test_polygons_intersect_overlapping_and_far():
    a = unit_square(0.0, 0.0, 1.0)
    b = unit_square(1.5, 0.0, 1.0)
    c = unit_square(10.0, 10.0, 1.0)
    d = unit_square(0.0, 0.0, 0.25)

    assert polygons_intersect(a, b)
    assert not polygons_intersect(a, c)
    assert polygons_intersect(a, d)


def test_polygons_intersect_no_vertex_inside():
    a = unit_square(0.0, 0.0, 1.0)
    cross_bar = [
        Point2D(-5.0, -0.1), Point2D(5.0, -0.1), Point2D(5.0, 0.1), Point2D(-5.0, 0.1),
    ]
    assert polygons_intersect(a, cross_bar)


def test_polygon_area():
    square = unit_square(0.0, 0.0, 1.0)  # side length 2
    assert math.isclose(polygon_area(square), 4.0, abs_tol=1e-9)


def test_polygon_intersection_area_stretch():
    a = [Point2D(0.0, 0.0), Point2D(1.0, 0.0), Point2D(1.0, 1.0), Point2D(0.0, 1.0)]
    b = [Point2D(0.5, 0.0), Point2D(1.5, 0.0), Point2D(1.5, 1.0), Point2D(0.5, 1.0)]
    area = polygon_intersection_area(a, b)
    assert math.isclose(area, 0.5, abs_tol=1e-6)
