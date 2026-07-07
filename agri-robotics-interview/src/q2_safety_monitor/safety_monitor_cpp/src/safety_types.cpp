#include "safety_monitor_cpp/safety_types.hpp"

namespace safety_monitor_cpp
{

// ---------------------------------------------------------------------------
// GIVEN
// ---------------------------------------------------------------------------

bool point_in_polygon(const Point2D & pt, const Polygon2D & poly)
{
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto & pi = poly[i];
    const auto & pj = poly[j];
    const bool straddles = (pi.y > pt.y) != (pj.y > pt.y);
    if (straddles) {
      const double x_intersect = (pj.x - pi.x) * (pt.y - pi.y) / (pj.y - pi.y) + pi.x;
      if (pt.x < x_intersect) {
        inside = !inside;
      }
    }
  }
  return inside;
}

// ---------------------------------------------------------------------------
// TODO (core) -- candidate implements ViolationTracker.
// ---------------------------------------------------------------------------

ViolationTracker::ViolationTracker(int warning_threshold, int critical_threshold)
: warning_threshold_(warning_threshold), critical_threshold_(critical_threshold)
{
  // TODO: initialize whatever member state you added in the header.
}

Severity ViolationTracker::record(const std::string & name, bool in_range)
{
  (void)name;
  (void)in_range;
  // TODO: implement the streak-tracking / thresholding behavior described
  // in the header.
  return Severity::OK;
}

int ViolationTracker::consecutive_violations(const std::string & name) const
{
  (void)name;
  // TODO: implement.
  return 0;
}

}  // namespace safety_monitor_cpp
