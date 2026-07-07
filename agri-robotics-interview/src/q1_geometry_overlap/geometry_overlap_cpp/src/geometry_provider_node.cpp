// GIVEN -- candidates do not need to edit this file.
//
// Stands in for "an internal node that returns geometries." Publishes:
//   /field/boundary            (geometry_msgs/Polygon, latched)
//   /field/restricted_zone     (geometry_msgs/Polygon, latched)
//   /tractor/implement_footprint (geometry_msgs/Polygon, ~2 Hz)
//
// The implement footprint walks a fixed loop of waypoints so behavior is
// deterministic: it starts outside the field, drives through it, clips the
// restricted zone briefly, and exits again.
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_overlap_cpp/geometry_types.hpp"
#include "geometry_overlap_cpp/ros_conversions.hpp"

using namespace std::chrono_literals;
using geometry_overlap_cpp::Point2D;
using geometry_overlap_cpp::Polygon2D;

namespace
{
Polygon2D make_square(double cx, double cy, double half_size)
{
  return {
    {cx - half_size, cy - half_size},
    {cx + half_size, cy - half_size},
    {cx + half_size, cy + half_size},
    {cx - half_size, cy + half_size},
  };
}
}  // namespace

class GeometryProviderNode : public rclcpp::Node
{
public:
  GeometryProviderNode()
  : Node("geometry_provider_node"), waypoint_index_(0)
  {
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();

    boundary_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/field/boundary", latched_qos);
    restricted_zone_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/field/restricted_zone", latched_qos);
    footprint_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
      "/tractor/implement_footprint", rclcpp::QoS(10));

    // 40m x 30m field centered on the origin.
    field_boundary_ = {{-20, -15}, {20, -15}, {20, 15}, {-20, 15}};
    // A small no-spray buffer (e.g. around a waterway) inside the field.
    restricted_zone_ = {{8, 3}, {14, 3}, {14, 8}, {8, 8}};

    // Deterministic path for the tractor implement's center point. The
    // implement footprint is a 3m x 3m square centered on this point.
    waypoints_ = {
      {-25.0, 0.0},
      {-15.0, 0.0},
      {-5.0, 0.0},
      {5.0, 0.0},
      {9.0, 5.5},    // clips into the restricted zone
      {11.0, 5.5},   // fully inside the restricted zone
      {13.0, 5.5},   // clipping out again
      {15.0, 0.0},
      {25.0, 0.0},   // exits the field boundary
    };

    boundary_pub_->publish(geometry_overlap_cpp::to_msg(field_boundary_));
    restricted_zone_pub_->publish(geometry_overlap_cpp::to_msg(restricted_zone_));

    timer_ = create_wall_timer(500ms, std::bind(&GeometryProviderNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "geometry_provider_node started");
  }

private:
  void on_timer()
  {
    const auto & wp = waypoints_[waypoint_index_];
    Polygon2D footprint = make_square(wp.first, wp.second, 1.5);
    footprint_pub_->publish(geometry_overlap_cpp::to_msg(footprint));
    RCLCPP_INFO(
      get_logger(), "implement footprint centered at (%.1f, %.1f)", wp.first, wp.second);
    waypoint_index_ = (waypoint_index_ + 1) % waypoints_.size();
  }

  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr boundary_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr restricted_zone_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr footprint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Polygon2D field_boundary_;
  Polygon2D restricted_zone_;
  std::vector<std::pair<double, double>> waypoints_;
  size_t waypoint_index_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeometryProviderNode>());
  rclcpp::shutdown();
  return 0;
}
