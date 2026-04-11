"""
ik_process.py — inverse kinematics process.

Receives hand/elbow tracking from Unity (via SHM or UDP),
solves QP-based IK using Pinocchio + Pink,
publishes joint angles back to Unity and to the motor command buffer.

Usage:
    python ik_process.py                  # defaults from config.py
    python ik_process.py --comm UDP       # override comm mode
    python ik_process.py --urdf my.urdf   # override URDF path
"""
import argparse
import json
import math
import struct
import time
from typing import Optional

import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask, PostureTask

from robot_arm.config import CommConfig, IKConfig
from robot_arm.comm import make_ik_comm


# ── Coordinate conversion ─────────────────────────────────────────────────────

def unity_to_pinocchio(pos, quat_wxyz, scale: float = 1.0) -> pin.SE3:
    """
    Convert Unity (left-handed, Y-up) position + quaternion to a Pinocchio SE3.

    Unity  → Pinocchio axis mapping:
      X_unity → -Y_pin
      Y_unity →  Z_pin
      Z_unity →  X_pin
    """
    p = np.array(pos, dtype=np.float64) * scale
    robot_pos = np.array([p[2], -p[0], p[1]])

    w, x, y, z = quat_wxyz
    q = pin.Quaternion(w, z, -x, y)
    q.normalize()
    return pin.SE3(q.toRotationMatrix(), robot_pos)


def parse_tracking_shm(raw: bytes) -> Optional[dict]:
    """Unpack the 44-byte SHM tracking packet."""
    if len(raw) < 44:
        return None
    vals = struct.unpack("f" * 11, raw[:44])
    return {
        "type": "tracking",
        "hand_pos":     list(vals[0:3]),
        "hand_rot":     list(vals[3:7]),
        "elbow_pos":    list(vals[7:10]),
        "dynamic_scale": vals[10],
    }


def parse_tracking_udp(raw: bytes) -> Optional[dict]:
    """Decode JSON UDP tracking packet."""
    try:
        return json.loads(raw.decode("utf-8"))
    except Exception:
        return None


# ── IK solver state ───────────────────────────────────────────────────────────

class IKSolver:
    def __init__(self, cfg: IKConfig):
        self._cfg = cfg
        self.robot = pin.RobotWrapper.BuildFromURDF(cfg.urdf_path)

        # Reach limit
        ee_id = self.robot.model.getFrameId(cfg.end_effector_link)
        self._max_reach = self._compute_reach(ee_id) * cfg.safe_reach_fraction

        # Initial config
        self._q = self.robot.q0.copy()
        self._configuration = pink.Configuration(
            self.robot.model, self.robot.data, self._q
        )

        # Forward kinematics at rest
        pin.forwardKinematics(self.robot.model, self.robot.data, self._q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        tcp_id = self.robot.model.getFrameId(cfg.end_effector_link)
        elbow_id = self.robot.model.getFrameId(cfg.elbow_joint_name)

        self._target_pos = self.robot.data.oMf[tcp_id].translation.copy()
        self._target_rot = self.robot.data.oMf[tcp_id].rotation.copy()
        self._elbow_pos = self.robot.data.oMf[elbow_id].translation.copy()

        # Tasks
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

        # Rotation offset applied to end-effector target
        self._ee_rot_offset = pin.utils.rpyToMatrix(-math.pi / 2, 0.0, 0.0)

        self.dynamic_scale: float = 1.0

    def _compute_reach(self, ee_frame_id: int) -> float:
        model = self.robot.model
        reach = np.linalg.norm(model.frames[ee_frame_id].placement.translation)
        jid = model.frames[ee_frame_id].parentJoint
        while jid > 0:
            reach += np.linalg.norm(model.jointPlacements[jid].translation)
            jid = model.parents[jid]
        return reach

    def apply_tracking(self, data: dict, comm_mode: str) -> None:
        """Update internal targets from a parsed tracking packet."""
        if data.get("type") == "calibration":
            arm_length = data.get("arm_length", 1.0)
            # Recompute scale so robot reach maps to human arm length
            self.dynamic_scale = (self._max_reach / self._cfg.safe_reach_fraction) \
                                  / arm_length * 1.1
            return

        if data.get("type") != "tracking":
            return

        if comm_mode == "SHM":
            self.dynamic_scale = data.get("dynamic_scale", self.dynamic_scale)

        hand_se3 = unity_to_pinocchio(
            data["hand_pos"], data["hand_rot"], scale=self.dynamic_scale
        )
        self._target_pos = hand_se3.translation
        self._target_rot = hand_se3.rotation @ self._ee_rot_offset

        elbow_se3 = unity_to_pinocchio(
            data["elbow_pos"], (1, 0, 0, 0), scale=self.dynamic_scale
        )
        self._elbow_pos = elbow_se3.translation

    def step(self, dt: float) -> np.ndarray:
        """Run one QP-IK step. Returns new joint configuration (radians)."""
        cfg = self._cfg

        # Clamp target to safe reach sphere
        dist = np.linalg.norm(self._target_pos)
        if dist > self._max_reach:
            self._target_pos = self._target_pos * (self._max_reach / dist)

        self._hand_task.set_target(pin.SE3(self._target_rot, self._target_pos))
        self._elbow_task.set_target(pin.SE3(np.eye(3), self._elbow_pos))

        velocity = pink.solve_ik(
            self._configuration, self._tasks, dt,
            solver=cfg.ik_solver, damping=cfg.ik_damping
        )
        q_new = pin.integrate(self.robot.model, self._configuration.q, velocity * dt)
        q_new = np.clip(
            q_new,
            self.robot.model.lowerPositionLimit,
            self.robot.model.upperPositionLimit,
        )

        # Dead-zone: skip if change is negligible
        if np.linalg.norm(q_new - self._configuration.q) > cfg.deadzone_threshold:
            self._configuration = pink.Configuration(
                self.robot.model, self.robot.data, q_new
            )
            self._q = q_new

        return self._q

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


# ── Visualizer wrapper ────────────────────────────────────────────────────────

def make_visualizer(robot):
    """Create and return a Meshcat visualizer, or None if meshcat is unavailable."""
    try:
        import meshcat
        import meshcat.geometry as g
        from pinocchio.visualize import MeshcatVisualizer

        viz = MeshcatVisualizer(
            robot.model, robot.collision_model, robot.visual_model
        )
        viz.initViewer(open=True)
        viz.loadViewerModel()

        viz.viewer["target_hand"].set_object(
            g.Sphere(0.04), g.MeshBasicMaterial(color=0xFF0000)
        )
        viz.viewer["target_hand_axis"].set_object(g.triad(scale=0.15))
        viz.viewer["target_elbow"].set_object(
            g.Sphere(0.04), g.MeshBasicMaterial(color=0x00FF00)
        )
        viz.viewer["robot_tcp_axis"].set_object(g.triad(scale=0.2))
        return viz
    except ImportError:
        print("[WARN] meshcat not available — running without visualization")
        return None


def update_visualizer(viz, robot, solver: IKSolver) -> None:
    """Push current IK state to the Meshcat viewer."""
    if viz is None:
        return

    q = solver.q
    viz.display(q)

    T_hand = np.eye(4)
    T_hand[:3, :3] = solver.target_rot
    T_hand[:3, 3] = solver.target_pos
    viz.viewer["target_hand"].set_transform(T_hand)
    viz.viewer["target_hand_axis"].set_transform(T_hand)

    T_elbow = np.eye(4)
    T_elbow[:3, 3] = solver.elbow_pos
    viz.viewer["target_elbow"].set_transform(T_elbow)

    pin.forwardKinematics(robot.model, robot.data, q)
    pin.updateFramePlacements(robot.model, robot.data)
    tcp_id = robot.model.getFrameId(solver._cfg.end_effector_link)
    viz.viewer["robot_tcp_axis"].set_transform(robot.data.oMf[tcp_id].homogeneous)


# ── Main loop ─────────────────────────────────────────────────────────────────

def run(comm_cfg: CommConfig, ik_cfg: IKConfig) -> None:
    solver = IKSolver(ik_cfg)
    comm = make_ik_comm(comm_cfg, ik_cfg)
    viz = make_visualizer(solver.robot)

    print(f"[IK] Started — comm={comm_cfg.mode}, urdf={ik_cfg.urdf_path}")
    print(f"[IK] Max reach: {solver._max_reach:.4f} m")

    # Determine parser based on comm mode
    parse = parse_tracking_shm if comm_cfg.mode == "SHM" else parse_tracking_udp

    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = max(now - last_time, ik_cfg.min_dt)
            last_time = now

            # 1. Receive tracking
            raw = comm.receive()
            if raw is not None:
                data = parse(raw)
                if data:
                    solver.apply_tracking(data, comm_cfg.mode)

            # 2. Solve IK
            q_new = solver.step(dt)

            # 3. Visualize
            update_visualizer(viz, solver.robot, solver)

            # 4. Publish
            packed = struct.pack(f"{len(q_new)}f", *q_new.tolist())
            comm.send(packed)

    except KeyboardInterrupt:
        print("\n[IK] Shutting down.")
    finally:
        comm.close()


# ── CLI entry point ───────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(description="Robot arm IK process")
    p.add_argument("--comm", choices=["SHM", "UDP"], default=None,
                   help="Override communication mode")
    p.add_argument("--urdf", default=None, help="Override URDF path")
    p.add_argument("--ip", default=None, help="Override IP address")
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()

    comm_cfg = CommConfig()
    ik_cfg = IKConfig()

    if args.comm:
        comm_cfg.mode = args.comm
    if args.ip:
        comm_cfg.ip = args.ip
    if args.urdf:
        ik_cfg.urdf_path = args.urdf

    run(comm_cfg, ik_cfg)
