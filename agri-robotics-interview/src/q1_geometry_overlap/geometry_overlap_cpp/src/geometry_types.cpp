#include "geometry_overlap_cpp/geometry_types.hpp"

#include <algorithm>
#include <cmath>
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

bool segments_intersect(
  const Point2D & p1, const Point2D & p2,
  const Point2D & p3, const Point2D & p4)
{
  auto cross = [](const Point2D & o, const Point2D & a, const Point2D & b) {
      return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };
  auto sign = [](double v) {return (v > 1e-12) - (v < -1e-12);};

  int d1 = sign(cross(p3, p4, p1));
  int d2 = sign(cross(p3, p4, p2));
  int d3 = sign(cross(p1, p2, p3));
  int d4 = sign(cross(p1, p2, p4));

  if (d1 != d2 && d3 != d4) {
    return true;
  }

  auto on_segment = [](const Point2D & a, const Point2D & b, const Point2D & p) {
      return std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) &&
             std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
    };

  if (d1 == 0 && on_segment(p3, p4, p1)) {return true;}
  if (d2 == 0 && on_segment(p3, p4, p2)) {return true;}
  if (d3 == 0 && on_segment(p1, p2, p3)) {return true;}
  if (d4 == 0 && on_segment(p1, p2, p4)) {return true;}
  return false;
}

double polygon_area(const Polygon2D & poly)
{
  if (poly.size() < 3) {
    return 0.0;
  }
  double sum = 0.0;
  for (size_t i = 0; i < poly.size(); ++i) {
    const auto & p1 = poly[i];
    const auto & p2 = poly[(i + 1) % poly.size()];
    sum += (p1.x * p2.y) - (p2.x * p1.y);
  }
  return std::abs(sum) * 0.5;
}

// ---------------------------------------------------------------------------
// TODO (core) -- candidate implements these.
// ---------------------------------------------------------------------------

bool point_in_polygon(const Point2D & pt, const Polygon2D & poly)
{
  (void)pt;
  (void)poly;
  // TODO: implement ray casting (even-odd rule).
  return false;
}

bool polygons_intersect(const Polygon2D & a, const Polygon2D & b)
{
  (void)a;
  (void)b;
  // TODO: bbox quick reject -> vertex-in-polygon -> edge intersection.
  return false;
}

// ---------------------------------------------------------------------------
// TODO (stretch)
// ---------------------------------------------------------------------------

Polygon2D clip_convex_polygon(const Polygon2D & subject, const Polygon2D & clip_convex)
{
  (void)subject;
  (void)clip_convex;
  // TODO: Sutherland-Hodgman clipping.
  return {};
}

// ---------------------------------------------------------------------------
// GIVEN -- depends on clip_convex_polygon()
// ---------------------------------------------------------------------------

double polygon_intersection_area(const Polygon2D & a, const Polygon2D & b)
{
  const Polygon2D clipped = clip_convex_polygon(a, b);
  return polygon_area(clipped);
}

}  // namespace geometry_overlap_cpp
