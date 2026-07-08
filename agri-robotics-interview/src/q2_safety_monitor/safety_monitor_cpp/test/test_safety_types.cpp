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

const std::string kEngineTempC = "engine_temp_c";
const std::string kGroundSpeedMps = "ground_speed_mps";

void check(bool cond, const std::string& description)
{
  if (cond) {
    std::printf("  [PASS] %s\n", description.c_str());
  } else {
    std::printf("  [FAIL] %s\n", description.c_str());
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

void test_sensor_increasing_violations(const std::string& sensor1, const std::string& sensor2)
{
  std::printf("test_sensor_increasing_violations (warning_threshold=3, critical_threshold=8):\n");
  ViolationTracker tracker(3, 8);

  // In range from the start -> always OK.
  check(tracker.record(sensor1, true) == Severity::OK, "in-range reading is OK");

  // 1st..2nd consecutive violation -> still OK (below warning_threshold).
  check(
    tracker.record(sensor1, false) == Severity::OK,
    "1st consecutive violation is still OK (" + sensor1 + ")");
  check(
    tracker.record(sensor1, false) == Severity::OK,
    "2nd consecutive violation is still OK (" + sensor1 + ")");

  // 3rd..7th -> WARNING.
  Severity sev = Severity::OK;
  for (int i = 0; i < 5; ++i) {
    sev = tracker.record(sensor1, false);
  }
  check(sev == Severity::WARNING, "3rd-7th consecutive violation is WARNING (" + sensor1 + ")");

  // 8th+ -> CRITICAL.
  sev = tracker.record(sensor1, false);
  check(sev == Severity::CRITICAL, "8th consecutive violation is CRITICAL (" + sensor1 + ")");

  // Recovering resets the streak.
  check(
    tracker.record(sensor1, true) == Severity::OK,
    "an in-range reading resets to OK (" + sensor1 + ")");
  check(
    tracker.record(sensor1, false) == Severity::OK,
    "streak restarts from 1 after recovery (" + sensor1 + ")");

  // Independent parameters don't share state.
  for (int i = 0; i < 8; ++i) {
    tracker.record(sensor2, false);
  }
  check(
    tracker.record(sensor1, false) == Severity::OK,
    "a different parameter's streak is independent (" + sensor1 + ")");
}

void test_sensor_fluctuation(const std::string& sensor)
{
  std::printf("test_sensor_fluctuation (warning_threshold=3, critical_threshold=8):\n");

  ViolationTracker tracker(3, 8);

  // In range from the start -> always OK.
  check(tracker.record(sensor, true) == Severity::OK, "in-range reading is OK");

  for (int ii = 0; ii < 5; ii++) {
    // 1st..2nd consecutive violation -> still OK (below warning_threshold).
    check(
      tracker.record(sensor, false) == Severity::OK,
      "1st consecutive violation is still OK (" + sensor + ")");
    check(
      tracker.record(sensor, false) == Severity::OK,
      "2nd consecutive violation is still OK (" + sensor + ")");

    check(tracker.record(sensor, true) == Severity::OK,
      "streak restarts after recovery (" + sensor + ")");
  }

  // 1st..2nd consecutive violation -> still OK (below warning_threshold).
  check(
    tracker.record(sensor, false) == Severity::OK,
    "1st consecutive violation is still OK (" + sensor + ")");
  check(
    tracker.record(sensor, false) == Severity::OK,
    "2nd consecutive violation is still OK (" + sensor + ")");

  // 3rd..7th -> WARNING.
  Severity sev = Severity::OK;
  for (int i = 0; i < 5; ++i) {
    sev = tracker.record(sensor, false);
  }
  check(sev == Severity::WARNING, "3rd-7th consecutive violation is WARNING (" + sensor + ")");

  // 8th+ -> CRITICAL.
  sev = tracker.record(sensor, false);
  check(sev == Severity::CRITICAL, "8th consecutive violation is CRITICAL (" + sensor + ")");
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

  test_sensor_increasing_violations(kEngineTempC, kGroundSpeedMps);
  test_sensor_fluctuation(kEngineTempC);

  if (g_failures == 0) {
    std::printf("\nAll tests passed.\n");
    return 0;
  }
  std::printf("\n%d test(s) failed.\n", g_failures);
  return 1;
}
