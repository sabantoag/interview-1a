"""GIVEN -- this node's wiring is provided. Your work for this exercise is in
geometry_types.py; this file just subscribes to the geometries published by
geometry_provider_node and calls into your implementation.

Publishes:
  /safety/restricted_zone_violation (std_msgs/Bool)
    true iff the current implement footprint overlaps the restricted zone.
  /field/coverage_overlap_area_m2 (std_msgs/Float64)
    overlap area (m^2) between the implement footprint and the field
    boundary, via geometry_overlap_py.geometry_types.polygon_intersection_area().
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Polygon
from std_msgs.msg import Bool, Float64

from geometry_overlap_py.geometry_types import polygon_intersection_area, polygons_intersect
from geometry_overlap_py.ros_conversions import from_msg


class OverlapCheckerNode(Node):

    def __init__(self):
        super().__init__('overlap_checker_node')

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._violation_pub = self.create_publisher(
            Bool, '/safety/restricted_zone_violation', 10)
        self._coverage_area_pub = self.create_publisher(
            Float64, '/field/coverage_overlap_area_m2', 10)

        self._field_boundary = []
        self._restricted_zone = []

        self.create_subscription(
            Polygon, '/field/boundary', self._on_boundary, latched_qos)
        self.create_subscription(
            Polygon, '/field/restricted_zone', self._on_restricted_zone, latched_qos)
        self.create_subscription(
            Polygon, '/tractor/implement_footprint', self._on_footprint, 10)

        self.get_logger().info('overlap_checker_node started')

    def _on_boundary(self, msg: Polygon):
        self._field_boundary = from_msg(msg)
        self.get_logger().info(f'received field boundary ({len(msg.points)} pts)')

    def _on_restricted_zone(self, msg: Polygon):
        self._restricted_zone = from_msg(msg)
        self.get_logger().info(f'received restricted zone ({len(msg.points)} pts)')

    def _on_footprint(self, msg: Polygon):
        footprint = from_msg(msg)

        if self._restricted_zone:
            violates = polygons_intersect(footprint, self._restricted_zone)
            self._violation_pub.publish(Bool(data=violates))
            if violates:
                self.get_logger().warning('implement footprint is inside the restricted zone!')

        if self._field_boundary:
            area = polygon_intersection_area(footprint, self._field_boundary)
            self._coverage_area_pub.publish(Float64(data=area))


def main(args=None):
    rclpy.init(args=args)
    node = OverlapCheckerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
