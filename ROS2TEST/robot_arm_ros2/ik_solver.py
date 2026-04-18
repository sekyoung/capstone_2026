"""
ik_solver.py — pure IK logic, no ROS 2 / no comm.

IKSolver wraps Pinocchio + Pink.  Feed it target SE3 poses via
apply_target() and apply_elbow_hint(), then call step(dt) to get the
new joint configuration vector q (radians, Pinocchio ordering).
"""
import math

import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask, PostureTask


class IKSolver:
    def __init__(self, urdf_path: str, cfg):
        """
        urdf_path : absolute path to arm.urdf
        cfg       : IKConfig instance
        """
        self._cfg = cfg
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            package_dirs=[],
            root_joint=None,
        )

        # ── Reach limit ───────────────────────────────────────────────
        ee_id = self.robot.model.getFrameId(cfg.end_effector_link)
        self._max_reach = self._compute_reach(ee_id) * cfg.safe_reach_fraction

        # ── Initial configuration ─────────────────────────────────────
        self._q = self.robot.q0.copy()
        self._configuration = pink.Configuration(
            self.robot.model, self.robot.data, self._q
        )

        # Compute rest-pose FK to seed the initial targets
        pin.forwardKinematics(self.robot.model, self.robot.data, self._q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        tcp_id    = self.robot.model.getFrameId(cfg.end_effector_link)
        elbow_id  = self.robot.model.getFrameId(cfg.elbow_joint_name)

        self._target_pos  = self.robot.data.oMf[tcp_id].translation.copy()
        self._target_rot  = self.robot.data.oMf[tcp_id].rotation.copy()
        self._elbow_pos   = self.robot.data.oMf[elbow_id].translation.copy()

        # ── Pink tasks ────────────────────────────────────────────────
        self._hand_task = FrameTask(
            cfg.end_effector_link,
            position_cost=cfg.position_cost,
            orientation_cost=cfg.orientation_cost,
        )
        self._elbow_task = FrameTask(
            cfg.elbow_joint_name,
            position_cost=cfg.elbow_position_cost,
            orientation_cost=0.0,
        )
        self._posture_task = PostureTask(cost=cfg.posture_cost)
        self._posture_task.set_target(self._q)

        self._tasks = [self._hand_task, self._elbow_task, self._posture_task]

        # Small rotation offset applied to the end-effector orientation target
        # (aligns the IK's notion of "down" with the physical end-effector)
        self._ee_rot_offset = pin.utils.rpyToMatrix(-math.pi / 2, 0.0, 0.0)

    # ── Public API ────────────────────────────────────────────────────

    def apply_target(self, pos: np.ndarray, rot: np.ndarray) -> None:
        """
        Update the hand target.
        pos : (3,) position in world/base frame
        rot : (3,3) rotation matrix in world/base frame
        """
        self._target_pos = np.asarray(pos, dtype=np.float64)
        self._target_rot = np.asarray(rot, dtype=np.float64) @ self._ee_rot_offset

    def apply_elbow_hint(self, pos: np.ndarray) -> None:
        """Update the elbow position hint (position only)."""
        self._elbow_pos = np.asarray(pos, dtype=np.float64)

    def step(self, dt: float) -> np.ndarray:
        """
        Run one QP-IK step.
        Returns the new joint configuration (radians, Pinocchio ordering).
        """
        cfg = self._cfg
        dt = max(dt, cfg.min_dt)

        # Clamp target to safe reach sphere
        dist = np.linalg.norm(self._target_pos)
        if dist > self._max_reach:
            self._target_pos = self._target_pos * (self._max_reach / dist)

        self._hand_task.set_target(pin.SE3(self._target_rot, self._target_pos))
        self._elbow_task.set_target(pin.SE3(np.eye(3), self._elbow_pos))

        velocity = pink.solve_ik(
            self._configuration, self._tasks, dt,
            solver=cfg.ik_solver,
            damping=cfg.ik_damping,
        )
        q_new = pin.integrate(self.robot.model, self._configuration.q, velocity * dt)
        q_new = np.clip(
            q_new,
            self.robot.model.lowerPositionLimit,
            self.robot.model.upperPositionLimit,
        )

        # Dead-zone: skip micro-updates
        if np.linalg.norm(q_new - self._configuration.q) > cfg.deadzone_threshold:
            self._configuration = pink.Configuration(
                self.robot.model, self.robot.data, q_new
            )
            self._q = q_new

        return self._q

    # ── Properties ────────────────────────────────────────────────────

    @property
    def q(self) -> np.ndarray:
        return self._q

    @property
    def target_pos(self) -> np.ndarray:
        return self._target_pos

    @property
    def target_rot(self) -> np.ndarray:
        return self._target_rot

    @property
    def elbow_pos(self) -> np.ndarray:
        return self._elbow_pos

    @property
    def max_reach(self) -> float:
        return self._max_reach

    # ── Helpers ───────────────────────────────────────────────────────

    def _compute_reach(self, ee_frame_id: int) -> float:
        model = self.robot.model
        reach = np.linalg.norm(model.frames[ee_frame_id].placement.translation)
        jid = model.frames[ee_frame_id].parentJoint
        while jid > 0:
            reach += np.linalg.norm(model.jointPlacements[jid].translation)
            jid = model.parents[jid]
        return reach

    def named_joint_positions(self) -> dict:
        """Return {joint_name: angle_rad} for all active (non-fixed) joints."""
        model = self.robot.model
        result = {}
        for i in range(1, model.njoints):
            if model.nqs[i] > 0:
                result[model.names[i]] = float(self._q[model.idx_qs[i]])
        return result
