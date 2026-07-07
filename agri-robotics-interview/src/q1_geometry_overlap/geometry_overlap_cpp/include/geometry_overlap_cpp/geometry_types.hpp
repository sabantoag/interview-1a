#ifndef GEOMETRY_OVERLAP_CPP__GEOMETRY_TYPES_HPP_
#define GEOMETRY_OVERLAP_CPP__GEOMETRY_TYPES_HPP_

#include <vector>

// Plain, ROS-independent geometry primitives. Keeping these free of any
// rclcpp/message dependency means you can unit test them in isolation
// (see test/test_geometry_types.cpp) without spinning up a ROS graph.
//
// All polygons are assumed to be "simple" (non-self-intersecting). Some
// functions additionally assume the polygon is convex and wound
// counter-clockwise -- that assumption is called out explicitly below.
namespace geometry_overlap_cpp
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

// A polygon is just an ordered list of vertices. The edge from vertices[i]
// to vertices[i+1] (wrapping around) is implicit -- do not repeat the first
// vertex at the end.
using Polygon2D = std::vector<Point2D>;

struct BoundingBox
{
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

// ---------------------------------------------------------------------------
// GIVEN -- implemented for you as an example of the expected style/testing
// pattern. You should not need to change these.
// ---------------------------------------------------------------------------

// Axis-aligned bounding box of a polygon.
BoundingBox compute_bounding_box(const Polygon2D & poly);

// Cheap O(1) reject: do two bounding boxes overlap (touching counts as
// overlap)?
bool bounding_boxes_overlap(const BoundingBox & a, const BoundingBox & b);

// Do line segments (p1,p2) and (p3,p4) intersect (including endpoint
// touches)? Standard orientation/cross-product test.
bool segments_intersect(
  const Point2D & p1, const Point2D & p2,
  const Point2D & p3, const Point2D & p4);

// Unsigned area of a simple polygon via the shoelace formula.
double polygon_area(const Polygon2D & poly);

// ---------------------------------------------------------------------------
// TODO (core) -- implement these.
// ---------------------------------------------------------------------------

// Is `pt` inside `poly`? `poly` may be non-convex. Points exactly on an edge
// may be treated either way (not tested); just be consistent.
//
// Suggested approach: ray casting (a.k.a. even-odd rule) -- cast a ray from
// `pt` in any fixed direction (e.g. +x) and count how many polygon edges it
// crosses. Odd count => inside.
bool point_in_polygon(const Point2D & pt, const Polygon2D & poly);

// Do polygons `a` and `b` overlap at all (touching, one fully containing the
// other, or partial overlap all count as true)? `a` and `b` may be
// non-convex.
//
// Suggested approach, cheapest checks first:
//   1. bounding_boxes_overlap() quick reject
//   2. any vertex of `a` inside `b`, or any vertex of `b` inside `a`
//      (point_in_polygon) -- catches full containment and simple overlaps
//   3. any edge of `a` intersects any edge of `b` (segments_intersect) --
//      catches overlaps where no vertex of either polygon is inside the
//      other (e.g. a "+" shape crossing through a square)
bool polygons_intersect(const Polygon2D & a, const Polygon2D & b);

// ---------------------------------------------------------------------------
// TODO (stretch) -- only if time allows.
// ---------------------------------------------------------------------------

// Clip `subject` (any simple polygon) against `clip_convex` (assumed convex
// and wound counter-clockwise), returning the intersection polygon.
// Classic Sutherland-Hodgman clipping: walk each edge of clip_convex and
// keep only the part of `subject` on the "inside" of that edge.
// Return an empty Polygon2D if there is no overlap.
Polygon2D clip_convex_polygon(const Polygon2D & subject, const Polygon2D & clip_convex);

// ---------------------------------------------------------------------------
// GIVEN -- built on top of clip_convex_polygon(), once you implement it.
// ---------------------------------------------------------------------------

// Area of the overlap between `a` and `b`, assuming `b` is convex/CCW (see
// clip_convex_polygon). Returns 0.0 if they don't overlap.
double polygon_intersection_area(const Polygon2D & a, const Polygon2D & b);

}  // namespace geometry_overlap_cpp

#endif  // GEOMETRY_OVERLAP_CPP__GEOMETRY_TYPES_HPP_
