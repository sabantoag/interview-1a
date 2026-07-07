#ifndef GEOMETRY_OVERLAP_CPP__ROS_CONVERSIONS_HPP_
#define GEOMETRY_OVERLAP_CPP__ROS_CONVERSIONS_HPP_

// GIVEN -- small helpers to convert between geometry_msgs/Polygon and the
// plain geometry_overlap_cpp::Polygon2D type used by the algorithms in
// geometry_types.hpp. Not part of the exercise; provided so both nodes can
// share the same conversion logic.

#include <geometry_msgs/msg/polygon.hpp>

#include "geometry_overlap_cpp/geometry_types.hpp"

namespace geometry_overlap_cpp
{

inline Polygon2D from_msg(const geometry_msgs::msg::Polygon & msg)
{
  Polygon2D poly;
  poly.reserve(msg.points.size());
  for (const auto & p : msg.points) {
    poly.push_back(Point2D{static_cast<double>(p.x), static_cast<double>(p.y)});
  }
  return poly;
}

inline geometry_msgs::msg::Polygon to_msg(const Polygon2D & poly)
{
  geometry_msgs::msg::Polygon msg;
  msg.points.reserve(poly.size());
  for (const auto & p : poly) {
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(p.x);
    pt.y = static_cast<float>(p.y);
    pt.z = 0.0f;
    msg.points.push_back(pt);
  }
  return msg;
}

}  // namespace geometry_overlap_cpp

#endif  // GEOMETRY_OVERLAP_CPP__ROS_CONVERSIONS_HPP_
