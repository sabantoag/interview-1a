"""GIVEN -- candidates do not need to edit this file.

Simulates a tractor's telemetry stream at 20 Hz plus a repeating,
deterministic 30-second fault schedule so the monitor's behavior is easy to
reason about while testing:
  t=7.5s  (0.5s) : engine_temp_c spikes above its max -- a *transient* blip.
                   With reasonable debounce thresholds this should surface
                   as WARNING only, never CRITICAL/E-STOP.
  t=20s   (1.5s) : ground_speed_mps stays pinned above its max -- long
                   enough to become a real, sustained CRITICAL/E-STOP.
  t=25s   (1.5s) : tractor position drives outside the field boundary -- a
                   sustained geofence CRITICAL/E-STOP.
The cycle then repeats. All other values sit at a nominal setpoint plus
small random jitter, safely inside range.

Published topics:
  /tractor/hydraulic_pressure_psi (std_msgs/Float64)
  /tractor/engine_temp_c          (std_msgs/Float64)
  /tractor/ground_speed_mps       (std_msgs/Float64)
  /tractor/pto_rpm                (std_msgs/Float64)
  /tractor/roll_deg               (std_msgs/Float64)
  /tractor/position                (geometry_msgs/Point)
  /field/boundary                  (geometry_msgs/Polygon, latched)
"""
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Point32, Polygon

CYCLE_TICKS = 600           # 30s at 20 Hz
ENGINE_FAULT_START = 150    # t=7.5s
ENGINE_FAULT_TICKS = 10     # 0.5s
SPEED_FAULT_START = 400     # t=20s
SPEED_FAULT_TICKS = 30      # 1.5s
GEOFENCE_FAULT_START = 500  # t=25s
GEOFENCE_FAULT_TICKS = 30   # 1.5s


class TelemetryPublisherNode(Node):

    def __init__(self):
        super().__init__('telemetry_publisher_node')

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._pubs = {
            'hydraulic_pressure_psi': self.create_publisher(
                Float64, '/tractor/hydraulic_pressure_psi', 10),
            'engine_temp_c': self.create_publisher(
                Float64, '/tractor/engine_temp_c', 10),
            'ground_speed_mps': self.create_publisher(
                Float64, '/tractor/ground_speed_mps', 10),
            'pto_rpm': self.create_publisher(Float64, '/tractor/pto_rpm', 10),
            'roll_deg': self.create_publisher(Float64, '/tractor/roll_deg', 10),
        }
        self._position_pub = self.create_publisher(Point, '/tractor/position', 10)
        self._boundary_pub = self.create_publisher(Polygon, '/field/boundary', latched_qos)

        # Same 40m x 30m field used in the Q1 exercise, for consistency.
        boundary = Polygon()
        for x, y in [(-20, -15), (20, -15), (20, 15), (-20, 15)]:
            boundary.points.append(Point32(x=float(x), y=float(y), z=0.0))
        self._boundary_pub.publish(boundary)

        self._tick = 0
        self._rng = random.Random(42)
        self._logged_transient_fault = False

        self._timer = self.create_timer(0.05, self._on_timer)
        self.get_logger().info(
            'telemetry_publisher_node started (30s repeating fault schedule)')

    def _jitter(self, amplitude: float) -> float:
        return self._rng.uniform(-amplitude, amplitude)

    def _on_timer(self):
        t = self._tick % CYCLE_TICKS

        engine_temp = 85.0 + self._jitter(2.0)
        if ENGINE_FAULT_START <= t < ENGINE_FAULT_START + ENGINE_FAULT_TICKS:
            engine_temp = 112.0  # transient -- above max of 105
            if not self._logged_transient_fault:
                self.get_logger().info('injecting transient engine_temp fault')
                self._logged_transient_fault = True

        ground_speed = 2.0 + self._jitter(0.2)
        if SPEED_FAULT_START <= t < SPEED_FAULT_START + SPEED_FAULT_TICKS:
            ground_speed = 6.0  # sustained -- above max of 4.5

        position = Point()
        position.x = self._jitter(0.3)
        position.y = self._jitter(0.3)
        if GEOFENCE_FAULT_START <= t < GEOFENCE_FAULT_START + GEOFENCE_FAULT_TICKS:
            position.x = 100.0  # sustained -- outside the field boundary
            position.y = 100.0

        self._pubs['hydraulic_pressure_psi'].publish(Float64(data=2400.0 + self._jitter(30.0)))
        self._pubs['engine_temp_c'].publish(Float64(data=engine_temp))
        self._pubs['ground_speed_mps'].publish(Float64(data=ground_speed))
        self._pubs['pto_rpm'].publish(Float64(data=540.0 + self._jitter(10.0)))
        self._pubs['roll_deg'].publish(Float64(data=self._jitter(1.5)))
        self._position_pub.publish(position)

        self._tick += 1


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
