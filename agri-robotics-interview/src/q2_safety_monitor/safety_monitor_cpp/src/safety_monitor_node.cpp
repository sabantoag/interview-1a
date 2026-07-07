// Candidate file -- most of the ROS wiring below is GIVEN. Your job is the
// evaluate() method near the bottom of this class, which is called on a
// timer at kEvalRateHz and is where you actually *use* the ViolationTracker
// class you implemented in safety_types.hpp/.cpp.
//
// Subscribes:
//   /tractor/hydraulic_pressure_psi, /tractor/engine_temp_c,
//   /tractor/ground_speed_mps, /tractor/pto_rpm, /tractor/roll_deg
//     (all std_msgs/Float64)
//   /tractor/position (geometry_msgs/Point)
//   /field/boundary   (geometry_msgs/Polygon, latched)
//
// Publishes:
//   /diagnostics    (diagnostic_msgs/DiagnosticArray) -- one DiagnosticStatus
//     per monitored parameter plus one for "geofence", at kEvalRateHz.
//   /safety/estop   (std_msgs/Bool) -- true iff the overall status is
//     CRITICAL.
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "safety_monitor_cpp/safety_types.hpp"

using namespace std::chrono_literals;
using safety_monitor_cpp::Point2D;
using safety_monitor_cpp::Polygon2D;
using safety_monitor_cpp::Range;
using safety_monitor_cpp::Severity;
using safety_monitor_cpp::ViolationTracker;

namespace
{
constexpr double kEvalRateHz = 10.0;
// Consecutive out-of-range evaluate() calls before escalating. At 10 Hz:
// WARNING after 0.3s, CRITICAL after 0.8s. Tune to your control loop and
// sensor noise in a real deployment -- these are chosen to make the demo's
// fault schedule (see telemetry_publisher_node.cpp) easy to observe.
constexpr int kWarningThreshold = 3;
constexpr int kCriticalThreshold = 8;
}  // namespace

class SafetyMonitorNode : public rclcpp::Node
{
public:
  SafetyMonitorNode()
  : Node("safety_monitor_node"),
    tracker_(kWarningThreshold, kCriticalThreshold)
  {
    // Safe operating ranges. In production you'd likely expose these as ROS
    // parameters (declare_parameter/get_parameter) instead of hardcoding
    // them -- a nice bonus if you have time left over.
    limits_["hydraulic_pressure_psi"] = Range{1800.0, 3000.0};
    limits_["engine_temp_c"] = Range{-10.0, 105.0};
    limits_["ground_speed_mps"] = Range{0.0, 4.5};
    limits_["pto_rpm"] = Range{0.0, 1050.0};
    limits_["roll_deg"] = Range{-15.0, 15.0};

    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/estop", rclcpp::QoS(10));

    subscribe_float("/tractor/hydraulic_pressure_psi", "hydraulic_pressure_psi");
    subscribe_float("/tractor/engine_temp_c", "engine_temp_c");
    subscribe_float("/tractor/ground_speed_mps", "ground_speed_mps");
    subscribe_float("/tractor/pto_rpm", "pto_rpm");
    subscribe_float("/tractor/roll_deg", "roll_deg");

    position_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/tractor/position", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Point::SharedPtr msg) {
        latest_position_ = Point2D{msg->x, msg->y};
        have_position_ = true;
      });

    boundary_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
      "/field/boundary", latched_qos,
      [this](geometry_msgs::msg::Polygon::SharedPtr msg) {
        field_boundary_.clear();
        field_boundary_.reserve(msg->points.size());
        for (const auto & p : msg->points) {
          field_boundary_.push_back(Point2D{p.x, p.y});
        }
      });

    const auto period = std::chrono::duration<double>(1.0 / kEvalRateHz);
    eval_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SafetyMonitorNode::evaluate, this));

    RCLCPP_INFO(get_logger(), "safety_monitor_node started");
  }

private:
  void subscribe_float(const std::string & topic, const std::string & name)
  {
    float_subs_.push_back(
      create_subscription<std_msgs::msg::Float64>(
        topic, rclcpp::QoS(10),
        [this, name](std_msgs::msg::Float64::SharedPtr msg) {
          latest_values_[name] = msg->data;
        }));
  }

  // GIVEN convenience -- build one DiagnosticStatus entry.
  diagnostic_msgs::msg::DiagnosticStatus make_status(
    const std::string & name, Severity severity, const std::string & message) const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "tractor: " + name;
    status.message = message;
    switch (severity) {
      case Severity::OK:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        break;
      case Severity::WARNING:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        break;
      case Severity::CRITICAL:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        break;
    }
    return status;
  }

  // -------------------------------------------------------------------
  // TODO (core) -- implement this method.
  //
  // For each entry in limits_:
  //   - look up the latest value in latest_values_ (skip if we haven't
  //     received one yet)
  //   - check it against the Range with Range::contains()
  //   - call tracker_.record(name, in_range) to get a debounced Severity
  //   - build a DiagnosticStatus (make_status() above) describing the
  //     value, its bounds, and the severity, and add it to the array
  //
  // Also check the geofence:
  //   - if we have both a position and a field boundary, use
  //     safety_monitor_cpp::point_in_polygon() to see if the tractor is
  //     inside the field
  //   - call tracker_.record("geofence", inside) to get a debounced Severity
  //   - add a DiagnosticStatus for it too (name it "geofence")
  //
  // Then:
  //   - publish a DiagnosticArray (header.stamp = now(), status = your
  //     vector of statuses) to diagnostics_pub_
  //   - compute the overall severity as the worst of all the individual
  //     severities, and publish std_msgs::msg::Bool(true) to estop_pub_ iff
  //     the overall severity is CRITICAL
  //
  // Performance to keep in mind while you write this (see also the Q2
  // README's discussion questions): this runs kEvalRateHz times per second
  // for the life of the node. Avoid anything here that allocates or scales
  // worse than O(number of monitored parameters).
  // -------------------------------------------------------------------
  void evaluate()
  {
    // TODO: implement (see above).
  }

  std::unordered_map<std::string, Range> limits_;
  std::unordered_map<std::string, double> latest_values_;
  ViolationTracker tracker_;

  Point2D latest_position_;
  bool have_position_{false};
  Polygon2D field_boundary_;

  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> float_subs_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr boundary_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::TimerBase::SharedPtr eval_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
