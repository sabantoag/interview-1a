#include "geometry_overlap_cpp/geometry_types.hpp"

#include <algorithm>
#include <limits>

namespace geometry_overlap_cpp
{

// ---------------------------------------------------------------------------
// GIVEN
// ---------------------------------------------------------------------------

BoundingBox compute_bounding_box(const Polygon2D & poly)
{
  BoundingBox bbox{
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest()};

  for (const auto & p : poly) {
    bbox.min_x = std::min(bbox.min_x, p.x);
    bbox.min_y = std::min(bbox.min_y, p.y);
    bbox.max_x = std::max(bbox.max_x, p.x);
    bbox.max_y = std::max(bbox.max_y, p.y);
  }
  return bbox;
}

bool bounding_boxes_overlap(const BoundingBox & a, const BoundingBox & b)
{
  return a.min_x <= b.max_x && a.max_x >= b.min_x &&
         a.min_y <= b.max_y && a.max_y >= b.min_y;
}

// ---------------------------------------------------------------------------
// TODO (core) -- candidate implements these using Boost.Geometry.
// ---------------------------------------------------------------------------

BoostPolygon to_boost_polygon(const Polygon2D & poly)
{
  (void)poly;
  // TODO: load `poly`'s points into a BoostPolygon's outer ring, close the
  // ring, and call boost::geometry::correct() on it.
  return BoostPolygon{};
}

bool polygons_intersect(const Polygon2D & a, const Polygon2D & b)
{
  (void)a;
  (void)b;
  // TODO: convert both to BoostPolygon and call boost::geometry::intersects().
  return false;
}

double polygon_intersection_area(const Polygon2D & a, const Polygon2D & b)
{
  (void)a;
  (void)b;
  // TODO: convert both to BoostPolygon, call boost::geometry::intersection()
  // into a BoostMultiPolygon, and return boost::geometry::area() of that.
  return 0.0;
}

// ---------------------------------------------------------------------------
// TODO (stretch)
// ---------------------------------------------------------------------------

std::vector<size_t> find_intersecting_zones(
  const Polygon2D & footprint, const std::vector<Polygon2D> & zones)
{
  (void)footprint;
  (void)zones;
  // TODO: build an rtree over each zone's bounding box (bg::envelope()),
  // query it with footprint's bounding box, then confirm each candidate
  // with an exact polygons_intersect() check.
  return {};
}

}  // namespace geometry_overlap_cpp
