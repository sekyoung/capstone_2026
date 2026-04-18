"""
sweep_node.py — simulated tracking source for testing.

Publishes a smooth circular sweep of the TCP target and a matching
elbow hint so the IK node always has something to track.  Replace (or
supplement) this node with a real tracking source later — as long as it
publishes the same topics / message types, the rest of the pipeline is
unchanged.

Publications
------------
  /target_hand_pose    geometry_msgs/PoseStamped   — TCP target (world frame)
  /target_elbow_point  geometry_msgs/PointStamped  — elbow hint (world frame)

The sweep is generated directly in the robot's world frame
(x-forward, y-left, z-up).  Positions are chosen to stay well within
the arm's reachable workspace.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped


class SweepNode(Node):
    def __init__(self):
        super().__init__('sweep_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('radius', 0.08)    # metres, sweep circle radius
        self.declare_parameter('height', 0.18)    # metres, fixed z height
        self.declare_parameter('speed', 1.0)      # angular speed multiplier

        rate   = self.get_parameter('rate_hz').get_parameter_value().double_value
        self._r = self.get_parameter('radius').get_parameter_value().double_value
        self._h = self.get_parameter('height').get_parameter_value().double_value
        self._w = self.get_parameter('speed').get_parameter_value().double_value

        # ── Publishers ────────────────────────────────────────────────
        self._pub_hand  = self.create_publisher(PoseStamped,  '/target_hand_pose',    10)
        self._pub_elbow = self.create_publisher(PointStamped, '/target_elbow_point',  10)

        # ── Timer ─────────────────────────────────────────────────────
        self._t = 0.0
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'Sweep node started  r={self._r:.3f} m  h={self._h:.3f} m  '
            f'speed={self._w:.1f}x  rate={rate:.0f} Hz'
        )

    def _tick(self) -> None:
        # ── Sweep position (world / robot base frame) ─────────────────
        # Circle in the X-Y plane at fixed height Z.
        # X-forward, Y-left convention (REP 103 / Pinocchio standard).
        t = self._t
        x =  self._r * math.cos(t * self._w) + 0.12
        y =  self._r * math.sin(t * self._w)
        z =  self._h

        # Elbow hint: halfway between base and hand, slightly lower
        ex = x * 0.5
        ey = y * 0.5
        ez = z * 0.5

        stamp = self.get_clock().now().to_msg()

        # ── Hand pose ─────────────────────────────────────────────────
        hand = PoseStamped()
        hand.header.stamp    = stamp
        hand.header.frame_id = 'world'
        hand.pose.position.x = x
        hand.pose.position.y = y
        hand.pose.position.z = z
        # Identity orientation — IK node applies its own ee_rot_offset
        hand.pose.orientation.w = 1.0
        hand.pose.orientation.x = 0.0
        hand.pose.orientation.y = 0.0
        hand.pose.orientation.z = 0.0

        # ── Elbow hint ────────────────────────────────────────────────
        elbow = PointStamped()
        elbow.header.stamp    = stamp
        elbow.header.frame_id = 'world'
        elbow.point.x = ex
        elbow.point.y = ey
        elbow.point.z = ez

        self._pub_hand.publish(hand)
        self._pub_elbow.publish(elbow)

        self._t += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = SweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
