import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from robot_arm_ros2.config import IKConfig
from robot_arm_ros2.ik_solver import IKSolver


def _pose_to_se3(msg: PoseStamped):
    p = msg.pose.position
    o = msg.pose.orientation
    pos = np.array([p.x, p.y, p.z])
    q = pin.Quaternion(o.w, o.x, o.y, o.z)
    q.normalize()
    return pos, q.toRotationMatrix()


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.declare_parameter('urdf_path', '')
        self.declare_parameter('ik_rate_hz', 100.0)

        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        ik_rate   = self.get_parameter('ik_rate_hz').get_parameter_value().double_value

        if not urdf_path:
            self.get_logger().fatal('urdf_path parameter is empty')
            raise RuntimeError('urdf_path not set')

        cfg = IKConfig(ik_rate_hz=ik_rate)
        self._solver = IKSolver(urdf_path, cfg)

        self.get_logger().info(
            f'IK ready  urdf={urdf_path}  '
            f'max_reach={self._solver.max_reach:.3f} m  rate={ik_rate:.0f} Hz'
        )

        self._sub = self.create_subscription(
            PoseStamped, '/target_hand_pose', self._cb_hand, 10)
        self._pub = self.create_publisher(JointState, '/joint_states', 10)

        self._last_time = self.get_clock().now()
        self._timer = self.create_timer(1.0 / ik_rate, self._tick)

    def _cb_hand(self, msg: PoseStamped) -> None:
        pos, rot = _pose_to_se3(msg)
        self._solver.apply_target(pos, rot)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        q = self._solver.step(dt)

        named = self._solver.named_joint_positions()
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name     = list(named.keys())
        msg.position = list(named.values())
        self._pub.publish(msg)


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
