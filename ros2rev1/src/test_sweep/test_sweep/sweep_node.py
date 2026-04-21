import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class SweepNode(Node):
    def __init__(self):
        super().__init__('sweep_node')

        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('radius',  0.08)
        self.declare_parameter('height',  0.18)
        self.declare_parameter('speed',   1.0)

        rate    = self.get_parameter('rate_hz').get_parameter_value().double_value
        self._r = self.get_parameter('radius').get_parameter_value().double_value
        self._h = self.get_parameter('height').get_parameter_value().double_value
        self._w = self.get_parameter('speed').get_parameter_value().double_value

        self._pub = self.create_publisher(PoseStamped, '/target_hand_pose', 10)

        self._t  = 0.0
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'Sweep node started  r={self._r:.3f} m  h={self._h:.3f} m  '
            f'speed={self._w:.1f}x  rate={rate:.0f} Hz'
        )

    def _tick(self) -> None:
        t = self._t
        x = self._r * math.cos(t * self._w) + 0.12
        y = self._r * math.sin(t * self._w)
        z = self._h

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        self._pub.publish(msg)
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
