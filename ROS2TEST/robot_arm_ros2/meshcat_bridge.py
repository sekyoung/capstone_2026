"""
meshcat_bridge.py — ROS 2 node that mirrors joint states into a Meshcat
web viewer.

Subscriptions
-------------
  /joint_states   sensor_msgs/JointState

On startup it opens a Meshcat server and prints the URL.  Open that URL
in any browser to see the arm moving in 3-D.

The bridge is a pure *subscriber* — it never publishes anything.
Hardware nodes (future) should subscribe to /joint_states the same way;
no changes to this bridge are needed when hardware is added.
"""
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import meshcat
    import meshcat.geometry as g
    _MESHCAT_OK = True
except ImportError:
    _MESHCAT_OK = False


class MeshcatBridge(Node):
    def __init__(self):
        super().__init__('meshcat_bridge')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value

        if not urdf_path:
            self.get_logger().fatal('urdf_path parameter is empty — cannot start meshcat_bridge.')
            raise RuntimeError('urdf_path not set')

        # ── Load robot model ──────────────────────────────────────────
        self._robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            package_dirs=[],
            root_joint=None,
        )
        self._model = self._robot.model

        # ── Build joint-name → q-index map ────────────────────────────
        # Used to convert incoming JointState names to the Pinocchio q vector.
        self._name_to_qidx: dict = {}
        for i in range(1, self._model.njoints):
            if self._model.nqs[i] > 0:
                self._name_to_qidx[self._model.names[i]] = self._model.idx_qs[i]

        # ── Meshcat visualizer ────────────────────────────────────────
        self._viz: MeshcatVisualizer | None = None
        if _MESHCAT_OK:
            self._init_meshcat()
        else:
            self.get_logger().warn('meshcat not installed — bridge is running without visualization.')

        # ── Subscriber ────────────────────────────────────────────────
        self._sub = self.create_subscription(
            JointState, '/joint_states', self._cb_joint_states, 10)

        self.get_logger().info('meshcat_bridge ready — subscribed to /joint_states')

    # ── Meshcat setup ─────────────────────────────────────────────────

    def _init_meshcat(self) -> None:
        try:
            self._viz = MeshcatVisualizer(
                self._robot.model,
                self._robot.collision_model,
                self._robot.visual_model,
            )
            # open=False so it doesn't auto-launch a browser (works in headless env)
            self._viz.initViewer(open=False)
            self._viz.loadViewerModel()

            # Show rest pose on startup
            self._viz.display(self._robot.q0)

            # ── Decorative markers ─────────────────────────────────────
            # TCP axis triad
            self._viz.viewer['tcp_axis'].set_object(g.triad(scale=0.08))

            url = self._viz.viewer.url()
            self.get_logger().info(f'Meshcat running at  {url}')
            self.get_logger().info('Open that URL in a browser to see the arm.')
        except Exception as exc:
            self.get_logger().warn(f'Meshcat init failed: {exc}  — continuing without visualization.')
            self._viz = None

    # ── Joint state callback ──────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState) -> None:
        if self._viz is None:
            return

        # Build q from named joint positions
        q = pin.neutral(self._model)
        for name, pos in zip(msg.name, msg.position):
            idx = self._name_to_qidx.get(name)
            if idx is not None:
                q[idx] = float(pos)

        self._viz.display(q)

        # Update TCP marker
        pin.forwardKinematics(self._model, self._robot.data, q)
        pin.updateFramePlacements(self._model, self._robot.data)
        tcp_id = self._model.getFrameId('link_5')
        if tcp_id < len(self._robot.data.oMf):
            T = self._robot.data.oMf[tcp_id].homogeneous
            self._viz.viewer['tcp_axis'].set_transform(T)


def main(args=None):
    rclpy.init(args=args)
    node = MeshcatBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
