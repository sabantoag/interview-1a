// Plain, framework-free unit test for geometry_types. No ROS graph needed --
// build and run directly:
//   colcon build --packages-select geometry_overlap_cpp
//   ./install/geometry_overlap_cpp/lib/geometry_overlap_cpp/test_geometry_types
#include <cstdio>
#include <cmath>

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

Polygon2D unit_square(double cx, double cy, double half_size)
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

  std::printf("point_in_polygon:\n");
  {
    Polygon2D square = unit_square(0.0, 0.0, 1.0);  // corners at +-1
    check(point_in_polygon({0.0, 0.0}, square), "center of square is inside");
    check(point_in_polygon({0.5, 0.5}, square), "point inside square");
    check(!point_in_polygon({5.0, 5.0}, square), "point far outside square");
    check(!point_in_polygon({0.0, 2.0}, square), "point above square");

    Polygon2D triangle = {{0.0, 0.0}, {4.0, 0.0}, {0.0, 4.0}};
    check(point_in_polygon({1.0, 1.0}, triangle), "point inside triangle");
    check(!point_in_polygon({3.0, 3.0}, triangle), "point outside triangle (past hypotenuse)");
  }

  std::printf("polygons_intersect:\n");
  {
    Polygon2D a = unit_square(0.0, 0.0, 1.0);   // [-1,1] x [-1,1]
    Polygon2D b = unit_square(1.5, 0.0, 1.0);   // [0.5, 2.5] x [-1, 1] -- overlaps a
    Polygon2D c = unit_square(10.0, 10.0, 1.0);  // far away -- no overlap
    Polygon2D d = unit_square(0.0, 0.0, 0.25);  // fully inside a

    check(polygons_intersect(a, b), "overlapping squares intersect");
    check(!polygons_intersect(a, c), "far-apart squares do not intersect");
    check(polygons_intersect(a, d), "fully-contained square counts as intersecting");

    Polygon2D cross_bar = {{-5.0, -0.1}, {5.0, -0.1}, {5.0, 0.1}, {-5.0, 0.1}};
    check(
      polygons_intersect(a, cross_bar),
      "thin bar crossing through square with no vertex inside still intersects");
  }

  std::printf("polygon_area:\n");
  {
    Polygon2D square = unit_square(0.0, 0.0, 1.0);  // side length 2
    check(std::abs(polygon_area(square) - 4.0) < 1e-9, "area of 2x2 square is 4");
  }

  std::printf("polygon_intersection_area (stretch):\n");
  {
    // Two unit squares (side 1) offset by 0.5 in x -> overlap is a 0.5 x 1.0
    // rectangle, area 0.5.
    Polygon2D a = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    Polygon2D b = {{0.5, 0.0}, {1.5, 0.0}, {1.5, 1.0}, {0.5, 1.0}};
    double area = polygon_intersection_area(a, b);
    check(std::abs(area - 0.5) < 1e-6, "overlap area of offset unit squares is 0.5");
  }

  if (g_failures == 0) {
    std::printf("\nAll tests passed.\n");
    return 0;
  }
  std::printf("\n%d test(s) failed.\n", g_failures);
  return 1;
}
