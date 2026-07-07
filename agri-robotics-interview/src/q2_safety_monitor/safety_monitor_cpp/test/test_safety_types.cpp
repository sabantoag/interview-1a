// Plain, framework-free unit test for safety_types. No ROS graph needed:
//   colcon build --packages-select safety_monitor_cpp
//   ./install/safety_monitor_cpp/lib/safety_monitor_cpp/test_safety_types
#include <cstdio>

#include "safety_monitor_cpp/safety_types.hpp"

using safety_monitor_cpp::Point2D;
using safety_monitor_cpp::Polygon2D;
using safety_monitor_cpp::Severity;
using safety_monitor_cpp::ViolationTracker;
using safety_monitor_cpp::point_in_polygon;

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

[[maybe_unused]] const char * to_str(Severity s)
{
  switch (s) {
    case Severity::OK: return "OK";
    case Severity::WARNING: return "WARNING";
    case Severity::CRITICAL: return "CRITICAL";
  }
  return "?";
}
}  // namespace

int main()
{
  std::printf("point_in_polygon (geofence building block):\n");
  {
    Polygon2D field = {{-20, -15}, {20, -15}, {20, 15}, {-20, 15}};
    check(point_in_polygon({0, 0}, field), "origin is inside the field");
    check(!point_in_polygon({100, 100}, field), "far point is outside the field");
  }

  std::printf("ViolationTracker (warning_threshold=3, critical_threshold=8):\n");
  {
    ViolationTracker tracker(3, 8);

    // In range from the start -> always OK.
    check(tracker.record("engine_temp_c", true) == Severity::OK, "in-range reading is OK");

    // 1st..2nd consecutive violation -> still OK (below warning_threshold).
    check(
      tracker.record("engine_temp_c", false) == Severity::OK,
      "1st consecutive violation is still OK");
    check(
      tracker.record("engine_temp_c", false) == Severity::OK,
      "2nd consecutive violation is still OK");

    // 3rd..7th -> WARNING.
    Severity sev = Severity::OK;
    for (int i = 0; i < 5; ++i) {
      sev = tracker.record("engine_temp_c", false);
    }
    check(sev == Severity::WARNING, "3rd-7th consecutive violation is WARNING");

    // 8th+ -> CRITICAL.
    sev = tracker.record("engine_temp_c", false);
    check(sev == Severity::CRITICAL, "8th consecutive violation is CRITICAL");

    // Recovering resets the streak.
    check(
      tracker.record("engine_temp_c", true) == Severity::OK,
      "an in-range reading resets to OK");
    check(
      tracker.record("engine_temp_c", false) == Severity::OK,
      "streak restarts from 1 after recovery");

    // Independent parameters don't share state.
    ViolationTracker tracker2(3, 8);
    for (int i = 0; i < 8; ++i) {
      tracker2.record("ground_speed_mps", false);
    }
    check(
      tracker2.record("engine_temp_c", false) == Severity::OK,
      "a different parameter's streak is independent");
  }

  if (g_failures == 0) {
    std::printf("\nAll tests passed.\n");
    return 0;
  }
  std::printf("\n%d test(s) failed.\n", g_failures);
  return 1;
}
