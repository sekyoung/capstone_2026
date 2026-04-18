"""
ik_node.py — ROS 2 node that runs QP inverse kinematics.

Subscriptions
-------------
  /target_hand_pose   geometry_msgs/PoseStamped   — desired TCP pose (world frame)
  /target_elbow_point geometry_msgs/PointStamped  — elbow hint (world frame, optional)

Publications
------------
  /joint_states       sensor_msgs/JointState       — solved joint angles (radians)

The node runs a timer at ik_rate_hz (default 100 Hz).  On each tick it
solves one QP-IK step using the most recently received target and
publishes updated joint states.  When no target has been received yet,
the arm stays at its rest configuration.

RViz2 and the meshcat_bridge both subscribe to /joint_states.
"""
import time

import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import JointState

from robot_arm_ros2.config import IKConfig
from robot_arm_ros2.ik_solver import IKSolver


def _pose_to_se3(msg: PoseStamped):
    """Convert PoseStamped → (position np.ndarray, rotation 3x3 np.ndarray)."""
    p = msg.pose.position
    o = msg.pose.orientation
    pos = np.array([p.x, p.y, p.z])
    q = pin.Quaternion(o.w, o.x, o.y, o.z)
    q.normalize()
    return pos, q.toRotationMatrix()


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('ik_rate_hz', 100.0)

        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        ik_rate   = self.get_parameter('ik_rate_hz').get_parameter_value().double_value

        if not urdf_path:
            self.get_logger().fatal('urdf_path parameter is empty — cannot start IK node.')
            raise RuntimeError('urdf_path not set')

        # ── IK solver ─────────────────────────────────────────────────
        cfg = IKConfig(ik_rate_hz=ik_rate)
        self._solver = IKSolver(urdf_path, cfg)
        self._cfg = cfg

        self.get_logger().info(
            f'IK solver ready  urdf={urdf_path}  '
            f'max_reach={self._solver.max_reach:.3f} m  rate={ik_rate:.0f} Hz'
        )

        # ── Subscribers ───────────────────────────────────────────────
        self._sub_hand = self.create_subscription(
            PoseStamped, '/target_hand_pose', self._cb_hand, 10)
        self._sub_elbow = self.create_subscription(
            PointStamped, '/target_elbow_point', self._cb_elbow, 10)

        # ── Publisher ─────────────────────────────────────────────────
        self._pub_js = self.create_publisher(JointState, '/joint_states', 10)

        # ── Timer ─────────────────────────────────────────────────────
        self._last_time = self.get_clock().now()
        self._timer = self.create_timer(1.0 / ik_rate, self._tick)

    # ── Callbacks ─────────────────────────────────────────────────────

    def _cb_hand(self, msg: PoseStamped) -> None:
        pos, rot = _pose_to_se3(msg)
        self._solver.apply_target(pos, rot)

    def _cb_elbow(self, msg: PointStamped) -> None:
        p = msg.point
        self._solver.apply_elbow_hint(np.array([p.x, p.y, p.z]))

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        q = self._solver.step(dt)
        self._publish_joint_states(q)

    # ── Helpers ───────────────────────────────────────────────────────

    def _publish_joint_states(self, q: np.ndarray) -> None:
        named = self._solver.named_joint_positions()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name     = list(named.keys())
        msg.position = list(named.values())
        self._pub_js.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
