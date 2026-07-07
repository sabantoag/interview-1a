"""Candidate file -- most of the ROS wiring below is GIVEN. Your job is the
evaluate() method near the bottom of this class, which is called on a timer
at EVAL_RATE_HZ and is where you actually *use* the ViolationTracker class
you implemented in safety_types.py.

Subscribes:
  /tractor/hydraulic_pressure_psi, /tractor/engine_temp_c,
  /tractor/ground_speed_mps, /tractor/pto_rpm, /tractor/roll_deg
    (all std_msgs/Float64)
  /tractor/position (geometry_msgs/Point)
  /field/boundary   (geometry_msgs/Polygon, latched)

Publishes:
  /diagnostics   (diagnostic_msgs/DiagnosticArray) -- one DiagnosticStatus
    per monitored parameter plus one for "geofence", at EVAL_RATE_HZ.
  /safety/estop  (std_msgs/Bool) -- true iff the overall status is CRITICAL.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Point, Polygon
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from safety_monitor_py.safety_types import (
    Point2D,
    Range,
    Severity,
    ViolationTracker,
    point_in_polygon,
)

EVAL_RATE_HZ = 10.0
# Consecutive out-of-range evaluate() calls before escalating. At 10 Hz:
# WARNING after 0.3s, CRITICAL after 0.8s. Tune to your control loop and
# sensor noise in a real deployment -- these are chosen to make the demo's
# fault schedule (see telemetry_publisher_node.py) easy to observe.
WARNING_THRESHOLD = 3
CRITICAL_THRESHOLD = 8

_SEVERITY_TO_LEVEL = {
    Severity.OK: DiagnosticStatus.OK,
    Severity.WARNING: DiagnosticStatus.WARN,
    Severity.CRITICAL: DiagnosticStatus.ERROR,
}


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor_node')

        # Safe operating ranges. In production you'd likely expose these as
        # ROS parameters (declare_parameter/get_parameter) instead of
        # hardcoding them -- a nice bonus if you have time left over.
        self._limits = {
            'hydraulic_pressure_psi': Range(1800.0, 3000.0),
            'engine_temp_c': Range(-10.0, 105.0),
            'ground_speed_mps': Range(0.0, 4.5),
            'pto_rpm': Range(0.0, 1050.0),
            'roll_deg': Range(-15.0, 15.0),
        }
        self._latest_values = {}
        self._tracker = ViolationTracker(WARNING_THRESHOLD, CRITICAL_THRESHOLD)

        self._latest_position = None
        self._field_boundary = []

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._estop_pub = self.create_publisher(Bool, '/safety/estop', 10)

        for name in self._limits:
            self._subscribe_float(name)

        self.create_subscription(Point, '/tractor/position', self._on_position, 10)
        self.create_subscription(
            Polygon, '/field/boundary', self._on_boundary, latched_qos)

        self.create_timer(1.0 / EVAL_RATE_HZ, self.evaluate)

        self.get_logger().info('safety_monitor_node started')

    def _subscribe_float(self, name: str):
        def callback(msg, name=name):
            self._latest_values[name] = msg.data

        self.create_subscription(Float64, f'/tractor/{name}', callback, 10)

    def _on_position(self, msg: Point):
        self._latest_position = Point2D(msg.x, msg.y)

    def _on_boundary(self, msg: Polygon):
        self._field_boundary = [Point2D(p.x, p.y) for p in msg.points]

    # GIVEN convenience -- build one DiagnosticStatus entry.
    def _make_status(self, name: str, severity: Severity, message: str) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f'tractor: {name}'
        status.message = message
        status.level = _SEVERITY_TO_LEVEL[severity]
        return status

    # -----------------------------------------------------------------
    # TODO (core) -- implement this method.
    #
    # For each (name, limit) in self._limits.items():
    #   - look up the latest value in self._latest_values (skip if we
    #     haven't received one yet)
    #   - check it against the Range with limit.contains()
    #   - call self._tracker.record(name, in_range) to get a debounced
    #     Severity
    #   - build a DiagnosticStatus (self._make_status() above) describing
    #     the value, its bounds, and the severity, and add it to the list
    #
    # Also check the geofence:
    #   - if we have both a position and a field boundary, use
    #     point_in_polygon() to see if the tractor is inside the field
    #   - call self._tracker.record("geofence", inside) to get a debounced
    #     Severity
    #   - add a DiagnosticStatus for it too (name it "geofence")
    #
    # Then:
    #   - publish a DiagnosticArray (header.stamp = self.get_clock().now()
    #     .to_msg(), status = your list of statuses) to self._diagnostics_pub
    #   - compute the overall severity as the worst of all the individual
    #     severities, and publish Bool(data=True) to self._estop_pub iff the
    #     overall severity is CRITICAL (publish Bool(data=False) otherwise)
    #
    # Performance to keep in mind while you write this (see also the Q2
    # README's discussion questions): this runs EVAL_RATE_HZ times per
    # second for the life of the node, and Python callbacks all run on the
    # same thread by default (single-threaded executor) -- so this method
    # competing for time with 5+ subscription callbacks matters. Avoid
    # anything here that's O(n^2) or does needless re-allocation every tick.
    # -----------------------------------------------------------------
    def evaluate(self):
        # TODO: implement (see above).
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
