#ifndef SAFETY_MONITOR_CPP__SAFETY_TYPES_HPP_
#define SAFETY_MONITOR_CPP__SAFETY_TYPES_HPP_

#include <string>
#include <vector>

// Plain, ROS-independent types for the safety monitor. Keeping these free of
// any rclcpp/message dependency means you can unit test them in isolation
// (see test/test_safety_types.cpp) without spinning up a ROS graph.
namespace safety_monitor_cpp
{

// ---------------------------------------------------------------------------
// GIVEN
// ---------------------------------------------------------------------------

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

using Polygon2D = std::vector<Point2D>;

// Same technique as the Q1 geometry-overlap exercise. Implemented here so
// this exercise is self-contained; if a candidate already solved Q1, feel
// free to let them paste in their own implementation instead.
bool point_in_polygon(const Point2D & pt, const Polygon2D & poly);

// An inclusive safe operating range for one telemetry value.
struct Range
{
  double min{0.0};
  double max{0.0};

  bool contains(double value) const {return value >= min && value <= max;}
};

enum class Severity
{
  OK = 0,
  WARNING = 1,
  CRITICAL = 2,
};

// ---------------------------------------------------------------------------
// TODO (core) -- design and implement this class. The public interface
// below (constructor + two method signatures) is fixed, because
// safety_monitor_node.cpp and test/test_safety_types.cpp are written against
// it -- but the private section is entirely yours to design.
// ---------------------------------------------------------------------------

// Tracks, per named parameter (e.g. "engine_temp_c", "geofence"), how many
// *consecutive* times in a row it has been reported out-of-range, and turns
// that streak into a Severity.
//
// Why debounce at all? Real sensors are noisy -- a single out-of-range
// reading is often just noise, not a real fault. Requiring several
// consecutive violations before escalating avoids nuisance E-STOPs while
// still reacting quickly to a real, sustained problem.
//
// Required behavior:
//   - record(name, true)  [in range]     -> reset that name's streak to 0,
//     return Severity::OK.
//   - record(name, false) [out of range] -> increment that name's streak,
//     then return:
//       Severity::OK        if streak < warning_threshold
//       Severity::WARNING   if warning_threshold <= streak < critical_threshold
//       Severity::CRITICAL  if streak >= critical_threshold
//   - Each name's streak is independent of every other name's.
//   - A name that has never been recorded should behave as if its streak is 0.
//
// Performance note: record() will be called once per monitored parameter on
// every timer tick (see safety_monitor_node.cpp) -- e.g. 5 parameters x 10 Hz
// today, but think about how your choice of container/lookup would hold up
// at 50+ parameters and 100 Hz.
class ViolationTracker
{
public:
  ViolationTracker(int warning_threshold, int critical_threshold);

  Severity record(const std::string & name, bool in_range);

  // Convenience for tests/introspection -- current consecutive-violation
  // count for `name` (0 if never recorded or not currently violating).
  int consecutive_violations(const std::string & name) const;

private:
  int warning_threshold_;
  int critical_threshold_;

  // TODO: add whatever member state you need (e.g. a map from name to
  // current streak count) to implement record() and consecutive_violations().
};

}  // namespace safety_monitor_cpp

#endif  // SAFETY_MONITOR_CPP__SAFETY_TYPES_HPP_
