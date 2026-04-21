import numpy as np
import pinocchio as pin


class IKSolver:
    def __init__(self, urdf_path: str, cfg):
        self._cfg = cfg
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path, package_dirs=[], root_joint=None,
        )
        self._model = self.robot.model
        self._data  = self.robot.data
        self._q     = self.robot.q0.copy()

        self._ee_id = self._model.getFrameId(cfg.end_effector_link)

        # Seed target from rest-pose FK
        pin.computeJointJacobians(self._model, self._data, self._q)
        pin.updateFramePlacements(self._model, self._data)
        T0 = self._data.oMf[self._ee_id]
        self._target_pos = T0.translation.copy()
        self._target_rot = T0.rotation.copy()

        self._max_reach = self._compute_reach() * cfg.safe_reach_fraction

    # ── Public API ────────────────────────────────────────────────────

    def apply_target(self, pos: np.ndarray, rot: np.ndarray) -> None:
        self._target_pos = np.asarray(pos, dtype=np.float64)
        self._target_rot = np.asarray(rot, dtype=np.float64)

    def step(self, dt: float) -> np.ndarray:
        dt = max(dt, self._cfg.min_dt)

        # Clamp to reachable sphere
        dist = np.linalg.norm(self._target_pos)
        if dist > self._max_reach:
            self._target_pos = self._target_pos * (self._max_reach / dist)

        pin.computeJointJacobians(self._model, self._data, self._q)
        pin.updateFramePlacements(self._model, self._data)

        T_cur = self._data.oMf[self._ee_id]
        T_des = pin.SE3(self._target_rot, self._target_pos)

        # 6-D error in local end-effector frame
        err = pin.log6(T_cur.inverse() * T_des).vector

        # Jacobian (local frame)
        J = pin.getFrameJacobian(
            self._model, self._data, self._ee_id, pin.LOCAL
        )

        # Damped least-squares velocity
        lam = self._cfg.ik_damping
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), self._cfg.ik_gain * err)

        q_new = pin.integrate(self._model, self._q, dq * dt)
        q_new = np.clip(
            q_new,
            self._model.lowerPositionLimit,
            self._model.upperPositionLimit,
        )

        if np.linalg.norm(q_new - self._q) > self._cfg.deadzone_threshold:
            self._q = q_new

        return self._q

    def named_joint_positions(self) -> dict:
        result = {}
        for i in range(1, self._model.njoints):
            if self._model.nqs[i] > 0:
                result[self._model.names[i]] = float(self._q[self._model.idx_qs[i]])
        return result

    @property
    def q(self) -> np.ndarray:
        return self._q

    @property
    def max_reach(self) -> float:
        return self._max_reach

    # ── Helpers ───────────────────────────────────────────────────────

    def _compute_reach(self) -> float:
        model = self._model
        reach = np.linalg.norm(model.frames[self._ee_id].placement.translation)
        jid = model.frames[self._ee_id].parentJoint
        while jid > 0:
            reach += np.linalg.norm(model.jointPlacements[jid].translation)
            jid = model.parents[jid]
        return reach
