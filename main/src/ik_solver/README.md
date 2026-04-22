# IK Solver (AI BUILD need to be edited...)

A Jacobian-based iterative inverse kinematics solver built on [Pinocchio](https://github.com/stack-of-tasks/pinocchio). Given a desired end-effector pose (position + orientation), it outputs joint angles that bring the robot there.

---

## How it works

### 1. Rigid-body kinematics (SE3)

Each link pose is an element of the Special Euclidean group **SE(3)**:

$$
T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \in SE(3), \quad R \in SO(3),\; p \in \mathbb{R}^3
$$

where $R$ is a $3\times3$ rotation matrix and $p$ is the origin of the frame expressed in the world frame.

---

### 2. Forward kinematics

Given joint configuration $q \in \mathbb{R}^n$, FK computes the current end-effector transform:

$$
T_{\text{cur}}(q) = T_1(q_1)\; T_2(q_2) \cdots T_n(q_n)
$$

This is computed by `pin.updateFramePlacements`.

---

### 3. Pose error in SE(3)

The solver works in the **local end-effector frame**. The error between the current transform $T_{\text{cur}}$ and the desired transform $T_{\text{des}}$ is computed via the SE(3) logarithm map:

$$
e = \log_{SE(3)}\!\left(T_{\text{cur}}^{-1}\, T_{\text{des}}\right) \in \mathbb{R}^6
$$

This gives a 6-D twist $e = [v^\top,\; \omega^\top]^\top$ where $v \in \mathbb{R}^3$ is translational error and $\omega \in \mathbb{R}^3$ is rotational error (axis-angle). Using the inverse avoids gimbal singularities and respects the non-Euclidean geometry of $SO(3)$.

---

### 4. Jacobian

The frame Jacobian $J(q)$ maps joint velocities $\dot{q}$ to end-effector twists in the local frame:

$$
J(q) \in \mathbb{R}^{6 \times n}, \qquad \dot{x} = J(q)\,\dot{q}
$$

$$
J = \begin{bmatrix}
J_{v,1} & J_{v,2} & \cdots & J_{v,n} \\
J_{\omega,1} & J_{\omega,2} & \cdots & J_{\omega,n}
\end{bmatrix}
$$

where each column $[J_{v,i}^\top,\; J_{\omega,i}^\top]^\top$ is the linear and angular velocity contribution of joint $i$.

---

### 5. Damped least-squares (Levenberg–Marquardt) IK

A naive pseudoinverse $J^+$ amplifies noise near singularities where $J$ loses rank. Instead the solver uses the **damped least-squares** solution:

$$
\dot{q} = J^\top \left(J J^\top + \lambda^2 I_6\right)^{-1} k_p\, e
$$

| Symbol | Config key | Meaning |
|--------|-----------|---------|
| $\lambda$ | `ik_damping` | Damping factor — higher = smoother near singularities |
| $k_p$ | `ik_gain` | Proportional gain — higher = faster convergence |
| $e$ | — | 6-D SE(3) error |

The $6\times6$ system $(J J^\top + \lambda^2 I)$ is always well-conditioned, so the solve is numerically stable even at kinematic singularities.

---

### 6. Joint integration

The velocity $\dot{q}$ is integrated on the **configuration manifold** (not naively in $\mathbb{R}^n$) over timestep $dt$:

$$
q_{\text{new}} = q \oplus (\dot{q}\,\Delta t)
$$

`pin.integrate` handles this correctly for joints with non-Euclidean configuration spaces (e.g. spherical joints). The result is then clamped to the URDF joint limits:

$$
q_{\text{new}} = \text{clip}\!\left(q_{\text{new}},\; q_{\min},\; q_{\max}\right)
$$

A **deadzone** suppresses micro-updates when the change is negligible:

$$
\text{update only if}\;\|q_{\text{new}} - q\| > \delta_{\text{deadzone}}
$$

---

### 7. Reachability clamping

Before solving, the target position is clamped to a sphere of radius $r_{\max}$:

$$
p_{\text{target}} \leftarrow p_{\text{target}} \cdot \frac{r_{\max}}{\|p_{\text{target}}\|} \quad \text{if } \|p_{\text{target}}\| > r_{\max}
$$

$r_{\max}$ is computed once at construction by summing link lengths along the kinematic chain (from the base to the end-effector frame), then scaling by `safe_reach_fraction < 1` to keep targets within a physically reachable region:

$$
r_{\text{reach}} = \|t_{\text{ee}}\| + \sum_{j=1}^{n} \|t_j\|, \qquad r_{\max} = \alpha \cdot r_{\text{reach}}
$$

where $t_j$ is the joint placement translation in the parent frame and $\alpha$ is `safe_reach_fraction`.

---

## Control flow per step

```
step(dt)
  │
  ├─ clamp target to reachable sphere
  ├─ FK → T_cur  (pin.updateFramePlacements)
  ├─ e  = log_SE3(T_cur⁻¹ · T_des)       # 6-D error
  ├─ J  = frame Jacobian (local frame)
  ├─ dq = Jᵀ (JJᵀ + λ²I)⁻¹ kp e         # damped LS
  ├─ q_new = integrate(q, dq·dt)          # manifold integration
  ├─ q_new = clip(q_new, q_min, q_max)   # joint limits
  └─ update q if ‖Δq‖ > deadzone
```

---

## Configuration parameters

| Parameter | Description |
|-----------|-------------|
| `end_effector_link` | Name of the end-effector frame in the URDF |
| `ik_gain` ($k_p$) | Proportional gain on the error |
| `ik_damping` ($\lambda$) | Damping factor for singularity robustness |
| `min_dt` | Minimum timestep to avoid division by near-zero |
| `safe_reach_fraction` ($\alpha$) | Fraction of full reach used as the safety sphere |
| `deadzone_threshold` ($\delta$) | Minimum $\|\Delta q\|$ to accept an update |

---

## Dependencies

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) — rigid-body kinematics and SE(3) math
- [NumPy](https://numpy.org) — linear algebra
