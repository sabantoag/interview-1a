// GIVEN -- candidates do not need to edit this file.
//
// Simulates a tractor's telemetry stream at 20 Hz plus a repeating,
// deterministic 30-second fault schedule so the monitor's behavior is easy
// to reason about while testing:
//   t=7.5s  (0.5s) : engine_temp_c spikes above its max -- a *transient*
//                    blip. With reasonable debounce thresholds this should
//                    surface as WARNING only, never CRITICAL/E-STOP.
//   t=20s   (1.5s) : ground_speed_mps stays pinned above its max -- long
//                    enough to become a real, sustained CRITICAL/E-STOP.
//   t=25s   (1.5s) : tractor position drives outside the field boundary --
//                    a sustained geofence CRITICAL/E-STOP.
// The cycle then repeats. All other values sit at a nominal setpoint plus
// small random jitter, safely inside range.
//
// Published topics:
//   /tractor/hydraulic_pressure_psi (std_msgs/Float64)
//   /tractor/engine_temp_c          (std_msgs/Float64)
//   /tractor/ground_speed_mps       (std_msgs/Float64)
//   /tractor/pto_rpm                (std_msgs/Float64)
//   /tractor/roll_deg               (std_msgs/Float64)
//   /tractor/position               (geometry_msgs/Point)
//   /field/boundary                 (geometry_msgs/Polygon, latched)
#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr int kCycleTicks = 600;            // 30s at 20 Hz
constexpr int kEngineFaultStart = 150;      // t=7.5s
constexpr int kEngineFaultTicks = 10;       // 0.5s
constexpr int kSpeedFaultStart = 400;       // t=20s
constexpr int kSpeedFaultTicks = 30;        // 1.5s
constexpr int kGeofenceFaultStart = 500;    // t=25s
constexpr int kGeofenceFaultTicks = 30;     // 1.5s
}  // namespace

class TelemetryPublisherNode : public rclcpp::Node
{
public:
  TelemetryPublisherNode()
  : Node("telemetry_publisher_node"), tick_(0), rng_(42), jitter_(-1.0, 1.0)
  {
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();

    pressure_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/tractor/hydraulic_pressure_psi", 10);
    engine_temp_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/tractor/engine_temp_c", 10);
    ground_speed_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/tractor/ground_speed_mps", 10);
    pto_rpm_pub_ = create_publisher<std_msgs::msg::Float64>("/tractor/pto_rpm", 10);
    roll_pub_ = create_publisher<std_msgs::msg::Float64>("/tractor/roll_deg", 10);
    position_pub_ = create_publisher<geometry_msgs::msg::Point>("/tractor/position", 10);
    boundary_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/field/boundary", latched_qos);

    // Same 40m x 30m field used in the Q1 exercise, for consistency.
    geometry_msgs::msg::Polygon boundary;
    for (auto [x, y] : std::vector<std::pair<double, double>>{
        {-20, -15}, {20, -15}, {20, 15}, {-20, 15}})
    {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(x);
      pt.y = static_cast<float>(y);
      boundary.points.push_back(pt);
    }
    boundary_pub_->publish(boundary);

    timer_ = create_wall_timer(50ms, std::bind(&TelemetryPublisherNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "telemetry_publisher_node started (30s repeating fault schedule)");
  }

private:
  double j(double amplitude) {return jitter_(rng_) * amplitude;}

  void on_timer()
  {
    const int t = tick_ % kCycleTicks;

    double engine_temp = 85.0 + j(2.0);
    if (t >= kEngineFaultStart && t < kEngineFaultStart + kEngineFaultTicks) {
      engine_temp = 112.0;  // transient -- above max of 105
      RCLCPP_INFO_ONCE(get_logger(), "injecting transient engine_temp fault");
    }

    double ground_speed = 2.0 + j(0.2);
    if (t >= kSpeedFaultStart && t < kSpeedFaultStart + kSpeedFaultTicks) {
      ground_speed = 6.0;  // sustained -- above max of 4.5
    }

    geometry_msgs::msg::Point position;
    position.x = j(0.3);
    position.y = j(0.3);
    if (t >= kGeofenceFaultStart && t < kGeofenceFaultStart + kGeofenceFaultTicks) {
      position.x = 100.0;  // sustained -- outside the field boundary
      position.y = 100.0;
    }

    publish(pressure_pub_, 2400.0 + j(30.0));
    publish(engine_temp_pub_, engine_temp);
    publish(ground_speed_pub_, ground_speed);
    publish(pto_rpm_pub_, 540.0 + j(10.0));
    publish(roll_pub_, j(1.5));
    position_pub_->publish(position);

    ++tick_;
  }

  void publish(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub, double value)
  {
    std_msgs::msg::Float64 msg;
    msg.data = value;
    pub->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pressure_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr engine_temp_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ground_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pto_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr boundary_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int tick_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> jitter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
