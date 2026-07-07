#ifndef GEOMETRY_OVERLAP_CPP__GEOMETRY_TYPES_HPP_
#define GEOMETRY_OVERLAP_CPP__GEOMETRY_TYPES_HPP_

#include <cstddef>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

// This exercise is about using a mature, real geospatial library correctly
// rather than hand-rolling computational-geometry algorithms yourself. We
// use Boost.Geometry (header-only, ships with libboost-dev) for the actual
// math -- your job is to build valid Boost.Geometry types from the app's
// plain data and call the right library functions on them.
//
// All polygons are assumed to be "simple" (non-self-intersecting).
namespace geometry_overlap_cpp
{

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Boost.Geometry types we standardize on for this exercise.
using BoostPoint = bg::model::d2::point_xy<double>;
// Default template args: clockwise winding, closed rings (first point
// repeated at the end). Both matter -- see to_boost_polygon() below.
using BoostPolygon = bg::model::polygon<BoostPoint>;
using BoostMultiPolygon = bg::model::multi_polygon<BoostPolygon>;
using BoostBox = bg::model::box<BoostPoint>;

// Plain, ROS-independent app types. Kept free of any rclcpp/message
// dependency so you can unit test in isolation (see
// test/test_geometry_types.cpp) without spinning up a ROS graph. This is
// also the type ros_conversions.hpp converts geometry_msgs::msg::Polygon
// to/from -- you don't need to touch that file.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

// An ordered list of vertices. Do not repeat the first vertex at the end --
// that's an internal detail of BoostPolygon, not of this type.
using Polygon2D = std::vector<Point2D>;

// ---------------------------------------------------------------------------
// GIVEN -- implemented for you as an example of the expected style. A cheap
// O(1) prefilter you may find useful (Boost.Geometry's own intersects/
// intersection calls already handle the general case correctly without
// this, but see the performance discussion in the README).
// ---------------------------------------------------------------------------

struct BoundingBox
{
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

BoundingBox compute_bounding_box(const Polygon2D & poly);
bool bounding_boxes_overlap(const BoundingBox & a, const BoundingBox & b);

// ---------------------------------------------------------------------------
// TODO (core) -- implement these.
// ---------------------------------------------------------------------------

// Build a valid BoostPolygon from `poly`. Two things Boost.Geometry expects
// that our plain Polygon2D doesn't guarantee:
//   1. The ring must be *closed* -- the first point must also appear as the
//      last point.
//   2. The ring must be wound in the polygon type's expected orientation
//      (clockwise, by default, for BoostPolygon).
// You don't need to figure out the winding direction yourself --
// boost::geometry::correct() will fix orientation (and can fix an unclosed
// ring too) once the points are loaded into the polygon.
BoostPolygon to_boost_polygon(const Polygon2D & poly);

// Do polygons `a` and `b` overlap at all (touching, containment, or partial
// overlap all count as true)? Use boost::geometry::intersects() -- don't
// reimplement the geometric test yourself.
bool polygons_intersect(const Polygon2D & a, const Polygon2D & b);

// Area (m^2) of the overlap between `a` and `b`. Use
// boost::geometry::intersection() (note: for two polygons, the result is a
// *multi*-polygon -- non-convex inputs can overlap in more than one
// disjoint piece) and boost::geometry::area(). Return 0.0 if they don't
// overlap.
double polygon_intersection_area(const Polygon2D & a, const Polygon2D & b);

// ---------------------------------------------------------------------------
// TODO (stretch) -- only if time allows.
// ---------------------------------------------------------------------------

// Given a footprint and a list of candidate zones, return the indices (into
// `zones`) of every zone that `footprint` overlaps. A naive implementation
// calls polygons_intersect() once per zone -- O(n) exact geometry tests.
// Instead, build a boost::geometry::index::rtree over the zones' bounding
// boxes (bg::envelope()) as a broad-phase filter, query it with
// `footprint`'s bounding box, and only run the exact (and more expensive)
// polygons_intersect() check against the candidates the R-tree returns.
std::vector<size_t> find_intersecting_zones(
  const Polygon2D & footprint, const std::vector<Polygon2D> & zones);

}  // namespace geometry_overlap_cpp

#endif  // GEOMETRY_OVERLAP_CPP__GEOMETRY_TYPES_HPP_
