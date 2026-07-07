// Plain, framework-free unit test for geometry_types. No ROS graph needed --
// build and run directly:
//   colcon build --packages-select geometry_overlap_cpp
//   ./install/geometry_overlap_cpp/lib/geometry_overlap_cpp/test_geometry_types
#include <cmath>
#include <cstdio>
#include <algorithm>

#include "geometry_overlap_cpp/geometry_types.hpp"

using geometry_overlap_cpp::Point2D;
using geometry_overlap_cpp::Polygon2D;

namespace
{
int g_failures = 0;

void check(bool cond, const char * description)
{
  if (cond) {
    std::printf("  [PASS] %s\n", description);
  } else {
    std::printf("  [FAIL] %s\n", description);
    ++g_failures;
  }
}

Polygon2D square(double cx, double cy, double half_size)
{
  return {
    {cx - half_size, cy - half_size},
    {cx + half_size, cy - half_size},
    {cx + half_size, cy + half_size},
    {cx - half_size, cy + half_size},
  };
}
}  // namespace

int main()
{
  using namespace geometry_overlap_cpp;

  std::printf("to_boost_polygon / polygons_intersect:\n");
  {
    Polygon2D a = square(0.0, 0.0, 1.0);   // [-1,1] x [-1,1]
    Polygon2D b = square(1.5, 0.0, 1.0);   // [0.5, 2.5] x [-1, 1] -- overlaps a
    Polygon2D c = square(10.0, 10.0, 1.0);  // far away -- no overlap
    Polygon2D d = square(0.0, 0.0, 0.25);  // fully inside a

    check(polygons_intersect(a, b), "overlapping squares intersect");
    check(!polygons_intersect(a, c), "far-apart squares do not intersect");
    check(polygons_intersect(a, d), "fully-contained square counts as intersecting");

    Polygon2D cross_bar = {{-5.0, -0.1}, {5.0, -0.1}, {5.0, 0.1}, {-5.0, 0.1}};
    check(
      polygons_intersect(a, cross_bar),
      "thin bar crossing through square with no vertex inside still intersects");
  }

  std::printf("polygon_intersection_area:\n");
  {
    // Two unit squares (side 1) offset by 0.5 in x -> overlap is a 0.5 x 1.0
    // rectangle, area 0.5.
    Polygon2D a = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    Polygon2D b = {{0.5, 0.0}, {1.5, 0.0}, {1.5, 1.0}, {0.5, 1.0}};
    double area = polygon_intersection_area(a, b);
    check(std::abs(area - 0.5) < 1e-6, "overlap area of offset unit squares is 0.5");

    Polygon2D c = square(10.0, 10.0, 1.0);
    check(
      std::abs(polygon_intersection_area(a, c)) < 1e-9,
      "non-overlapping polygons have 0 intersection area");

    // Fully-contained square: overlap area == the smaller square's area.
    Polygon2D small = square(0.5, 0.5, 0.1);  // 0.2 x 0.2, area 0.04
    double contained_area = polygon_intersection_area(a, small);
    check(
      std::abs(contained_area - 0.04) < 1e-6,
      "overlap area of a fully-contained square equals its own area");
  }

  std::printf("find_intersecting_zones (stretch):\n");
  {
    Polygon2D footprint = square(0.0, 0.0, 1.0);  // [-1,1] x [-1,1]
    std::vector<Polygon2D> zones = {
      square(0.5, 0.5, 0.2),    // overlaps footprint
      square(10.0, 10.0, 1.0),  // far away
      square(-0.5, -0.5, 0.2),  // overlaps footprint
      square(5.0, 0.0, 1.0),    // far away
    };

    std::vector<size_t> hits = find_intersecting_zones(footprint, zones);
    check(hits.size() == 2, "exactly 2 of 4 zones intersect the footprint");
    const bool has0 = std::find(hits.begin(), hits.end(), 0u) != hits.end();
    const bool has2 = std::find(hits.begin(), hits.end(), 2u) != hits.end();
    check(has0 && has2, "the correct zone indices (0 and 2) were returned");
  }

  if (g_failures == 0) {
    std::printf("\nAll tests passed.\n");
    return 0;
  }
  std::printf("\n%d test(s) failed.\n", g_failures);
  return 1;
}
