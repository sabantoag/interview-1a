"""GIVEN -- candidates do not need to edit this file.

Stands in for "an internal node that returns geometries." Publishes:
  /field/boundary               (geometry_msgs/Polygon, latched)
  /field/restricted_zone        (geometry_msgs/Polygon, latched)
  /tractor/implement_footprint  (geometry_msgs/Polygon, ~2 Hz)

The implement footprint walks a fixed loop of waypoints so behavior is
deterministic: it starts outside the field, drives through it, clips the
restricted zone briefly, and exits again.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Polygon

from geometry_overlap_py.geometry_types import Point2D
from geometry_overlap_py.ros_conversions import to_msg


def make_square(cx: float, cy: float, half_size: float):
    return [
        Point2D(cx - half_size, cy - half_size),
        Point2D(cx + half_size, cy - half_size),
        Point2D(cx + half_size, cy + half_size),
        Point2D(cx - half_size, cy + half_size),
    ]


class GeometryProviderNode(Node):

    def __init__(self):
        super().__init__('geometry_provider_node')

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._boundary_pub = self.create_publisher(Polygon, '/field/boundary', latched_qos)
        self._restricted_zone_pub = self.create_publisher(
            Polygon, '/field/restricted_zone', latched_qos)
        self._footprint_pub = self.create_publisher(
            Polygon, '/tractor/implement_footprint', 10)

        # 40m x 30m field centered on the origin.
        field_boundary = [
            Point2D(-20, -15), Point2D(20, -15), Point2D(20, 15), Point2D(-20, 15),
        ]
        # A small no-spray buffer (e.g. around a waterway) inside the field.
        restricted_zone = [
            Point2D(8, 3), Point2D(14, 3), Point2D(14, 8), Point2D(8, 8),
        ]

        # Deterministic path for the tractor implement's center point. The
        # implement footprint is a 3m x 3m square centered on this point.
        self._waypoints = [
            (-25.0, 0.0),
            (-15.0, 0.0),
            (-5.0, 0.0),
            (5.0, 0.0),
            (9.0, 5.5),    # clips into the restricted zone
            (11.0, 5.5),   # fully inside the restricted zone
            (13.0, 5.5),   # clipping out again
            (15.0, 0.0),
            (25.0, 0.0),   # exits the field boundary
        ]
        self._waypoint_index = 0

        self._boundary_pub.publish(to_msg(field_boundary))
        self._restricted_zone_pub.publish(to_msg(restricted_zone))

        self._timer = self.create_timer(0.5, self._on_timer)
        self.get_logger().info('geometry_provider_node started')

    def _on_timer(self):
        cx, cy = self._waypoints[self._waypoint_index]
        footprint = make_square(cx, cy, 1.5)
        self._footprint_pub.publish(to_msg(footprint))
        self.get_logger().info(f'implement footprint centered at ({cx:.1f}, {cy:.1f})')
        self._waypoint_index = (self._waypoint_index + 1) % len(self._waypoints)


def main(args=None):
    rclpy.init(args=args)
    node = GeometryProviderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
