// GIVEN -- this node's wiring is provided. Your work for this exercise is in
// geometry_types.cpp/.hpp; this file just subscribes to the geometries
// published by geometry_provider_node and calls into your implementation.
//
// Publishes:
//   /safety/restricted_zone_violation (std_msgs/Bool)
//     true iff the current implement footprint overlaps the restricted zone.
//   /field/coverage_overlap_area_m2 (std_msgs/Float64) [depends on the
//     stretch goal, clip_convex_polygon()]
//     overlap area (m^2) between the implement footprint and the field
//     boundary. NOTE: field boundary here is convex, satisfying the
//     assumption in clip_convex_polygon().
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "geometry_overlap_cpp/geometry_types.hpp"
#include "geometry_overlap_cpp/ros_conversions.hpp"

using geometry_overlap_cpp::Polygon2D;
using std::placeholders::_1;

class OverlapCheckerNode : public rclcpp::Node
{
public:
  OverlapCheckerNode()
  : Node("overlap_checker_node")
  {
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();

    violation_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/safety/restricted_zone_violation", rclcpp::QoS(10));
    coverage_area_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/field/coverage_overlap_area_m2", rclcpp::QoS(10));

    boundary_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
      "/field/boundary", latched_qos,
      [this](geometry_msgs::msg::Polygon::SharedPtr msg) {
        field_boundary_ = geometry_overlap_cpp::from_msg(*msg);
        RCLCPP_INFO(get_logger(), "received field boundary (%zu pts)", msg->points.size());
      });

    restricted_zone_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
      "/field/restricted_zone", latched_qos,
      [this](geometry_msgs::msg::Polygon::SharedPtr msg) {
        restricted_zone_ = geometry_overlap_cpp::from_msg(*msg);
        RCLCPP_INFO(get_logger(), "received restricted zone (%zu pts)", msg->points.size());
      });

    footprint_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
      "/tractor/implement_footprint", rclcpp::QoS(10),
      std::bind(&OverlapCheckerNode::on_footprint, this, _1));

    RCLCPP_INFO(get_logger(), "overlap_checker_node started");
  }

private:
  void on_footprint(geometry_msgs::msg::Polygon::SharedPtr msg)
  {
    const Polygon2D footprint = geometry_overlap_cpp::from_msg(*msg);

    if (!restricted_zone_.empty()) {
      const bool violates = geometry_overlap_cpp::polygons_intersect(footprint, restricted_zone_);
      std_msgs::msg::Bool violation_msg;
      violation_msg.data = violates;
      violation_pub_->publish(violation_msg);
      if (violates) {
        RCLCPP_WARN(get_logger(), "implement footprint is inside the restricted zone!");
      }
    }

    if (!field_boundary_.empty()) {
      const double area = geometry_overlap_cpp::polygon_intersection_area(
        footprint, field_boundary_);
      std_msgs::msg::Float64 area_msg;
      area_msg.data = area;
      coverage_area_pub_->publish(area_msg);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr boundary_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr restricted_zone_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr violation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr coverage_area_pub_;

  Polygon2D field_boundary_;
  Polygon2D restricted_zone_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OverlapCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
