#!/usr/bin/env python3
"""Analytic 3-DOF kinematics for the new_arm, in the URDF joint frame.

Pure Python (numpy only) so it runs identically inside Isaac Sim, under ROS, or
standalone. Angles are RADIANS in the URDF joint convention (centred at 0);
positions are METRES in the base (`arm_base_link`) frame.

The arm: base yaw (after_base_full_joint, axis Z) -> shoulder pitch (link1_joint,
axis Y) -> elbow pitch (link2_joint, axis Y) -> passive wrist (end_effector_joint,
axis Y, no motor). The planning/IK target is the WRIST PIVOT = the origin of
`end_effector_link`, whose position is independent of the passive joint (rotating it
only spins that frame in place — the same reason MoveIt plans 3-DOF).

Geometry is taken verbatim from new_arm_description/urdf/new_arm.urdf. A faithful
port of the manufacturer firmware FK/IK (_espmax.cpp, degrees + mm) is included as
`fk_firmware`/`ik_firmware` for reference and real-arm cross-checks.
"""

import math

import numpy as np

# --- URDF joint origins (xyz, metres) and rotation axes, verbatim from the URDF ---
# T_parent_child = Trans(origin) @ Rot(axis, q)   (all rpy are zero in this URDF)
T1 = np.array([0.00043239, -0.00184063, 0.08396122])  # base_link  -> after_base_full (axis Z)
T2 = np.array([0.00814224, -0.00043239, 0.00161718])  # after_base -> link1          (axis Y)
T3 = np.array([0.001,      -0.00856,    0.12839612])  # link1      -> link2          (axis Y)
T4 = np.array([0.12531939,  0.01099,   -0.05778452])  # link2      -> end_effector   (axis Y, passive)

ACTUATED_JOINTS = ["after_base_full_joint", "link1_joint", "link2_joint"]
PASSIVE_JOINT = "end_effector_joint"

# Passive wrist coupling (from new_arm_driver/config/calibration.yaml):
#   theta4 = passive_offset + c_sh*theta_shoulder + c_el*theta_elbow, wrapped to (-pi, pi]
# NOTE: the original firmware offset was 2*pi - pi/2 (=> home wrist -pi/2). The "- pi/2"
# term was intentionally dropped here so the wrist is LEVEL (0) at home; this is a
# deliberate sim/twin change (see import_urdf.py) and means the twin no longer matches
# the un-recalibrated real ESPMax firmware until the hardware calibration is updated.
PASSIVE_OFFSET = 2.0 * math.pi  # was 2*pi - pi/2; -pi/2 removed => home EE = 0
PASSIVE_C_SH = -1.0
PASSIVE_C_EL = -1.0

JOINT_LIMITS = {
    "after_base_full_joint": (-3.14, 3.14),
    "link1_joint": (-3.14, 3.14),
    "link2_joint": (-3.14, 3.14),
    "end_effector_joint": (-3.14, 3.14),
}


# ----------------------------------------------------------------------------- #
# Rotation helpers
# ----------------------------------------------------------------------------- #
def _rz(q):
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(q):
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def wrap_pi(a):
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# ----------------------------------------------------------------------------- #
# Forward kinematics (exact, by transform composition)
# ----------------------------------------------------------------------------- #
def forward_kinematics(q):
    """Wrist-pivot position (x, y, z) in the base frame for actuated joints q.

    q = [q_base, q_shoulder, q_elbow] (radians). The passive joint does not move
    the pivot, so it is not needed here.
    """
    q1, q2, q3 = float(q[0]), float(q[1]), float(q[2])
    R1 = _rz(q1)
    R2 = R1 @ _ry(q2)
    R3 = R2 @ _ry(q3)
    return T1 + R1 @ T2 + R2 @ T3 + R3 @ T4


def fk_full_chain(q_actuated, q_passive=None):
    """Return positions of every joint origin: [base, j1, j2, j3, tip] (5 points)."""
    q1, q2, q3 = (float(v) for v in q_actuated)
    R1 = _rz(q1)
    R2 = R1 @ _ry(q2)
    R3 = R2 @ _ry(q3)
    p0 = np.zeros(3)
    p1 = T1
    p2 = T1 + R1 @ T2
    p3 = T1 + R1 @ T2 + R2 @ T3
    p4 = T1 + R1 @ T2 + R2 @ T3 + R3 @ T4
    return np.array([p0, p1, p2, p3, p4])


def passive_wrist(q_shoulder, q_elbow):
    """Coupled passive wrist angle (rad), wrapped to (-pi, pi] — matches arm_bridge."""
    v = PASSIVE_OFFSET + PASSIVE_C_SH * q_shoulder + PASSIVE_C_EL * q_elbow
    return wrap_pi(v)


# ----------------------------------------------------------------------------- #
# Inverse kinematics (analytic 3-DOF, position-only)
# ----------------------------------------------------------------------------- #
# Planar 2-link model recovered from the URDF link origins. Each pitch joint rotates
# about +Y; after the base yaw the arm moves in a (radial, z) plane. A link vector
# v=(vx,vy,vz) in its parent frame, rotated by Ry(q), lands at planar angle (q + phi)
# measured FROM +z TOWARD +radial, where phi = atan2(vx, vz), keeping length
# |(vx,vz)|. The tiny vy components (sub-cm) are dropped here (residual floor), then
# a damped-Newton polish drives the position residual to ~0.
_A = math.hypot(T3[0], T3[2])      # shoulder->elbow planar length (~0.1284 m)
_B = math.hypot(T4[0], T4[2])      # elbow->wrist  planar length (~0.1380 m)
_PHI_A = math.atan2(T3[0], T3[2])  # intrinsic planar angle of link a (~0.008 rad)
_PHI_B = math.atan2(T4[0], T4[2])  # intrinsic planar angle of link b (~2.00 rad)
_SHOULDER = T1 + T2                # shoulder pivot in base frame
_SH_Z = _SHOULDER[2]


def _jacobian(q, eps=1e-6):
    """Numerical 3x3 position Jacobian d(tip)/d(q) via central differences."""
    J = np.zeros((3, 3))
    for i in range(3):
        dq = np.zeros(3)
        dq[i] = eps
        J[:, i] = (forward_kinematics(q + dq) - forward_kinematics(q - dq)) / (2.0 * eps)
    return J


def _newton_polish(q, target, iters=8, lam=1e-4, tol=1e-9):
    """Damped least-squares refinement of q so FK(q) == target (position)."""
    q = q.astype(float).copy()
    for _ in range(iters):
        err = target - forward_kinematics(q)
        if float(err @ err) < tol * tol:
            break
        J = _jacobian(q)
        dq = np.linalg.solve(J.T @ J + lam * np.eye(3), J.T @ err)
        q = q + dq
    return np.array([wrap_pi(v) for v in q])


def inverse_kinematics(target_xyz, elbow_up=True):
    """Analytic IK: wrist-pivot target (x, y, z) metres -> [q_base, q_shoulder, q_elbow].

    Returns (q, ok). `ok` is False (with a best-effort, clamped q) if the target is
    out of reach. Position-only; the passive wrist follows by coupling. `elbow_up`
    selects one of the two elbow branches.
    """
    x, y, z = (float(v) for v in target_xyz)

    # 1) base yaw aligns the arm plane with the target azimuth.
    q1 = math.atan2(y, x)

    # 2) target in the (radial, z) plane, relative to the shoulder pivot.
    sh_r = _SHOULDER[0] * math.cos(q1) + _SHOULDER[1] * math.sin(q1)
    X = math.hypot(x, y) - sh_r     # radial of target from shoulder
    Y = z - _SH_Z                   # height of target from shoulder

    d2 = X * X + Y * Y
    d = math.sqrt(d2)
    reach = _A + _B
    ok = True
    if d > reach:                   # clamp to the boundary if unreachable
        s = (reach - 1e-9) / d
        X, Y, d, d2 = X * s, Y * s, reach - 1e-9, (reach - 1e-9) ** 2
        ok = False
    elif d < abs(_A - _B):
        s = (abs(_A - _B) + 1e-9) / max(d, 1e-12)
        X, Y, d, d2 = X * s, Y * s, abs(_A - _B) + 1e-9, (abs(_A - _B) + 1e-9) ** 2
        ok = False

    # 3) standard 2-link IK, angles measured from +z (Y axis) toward +radial (X).
    cos_delta = (d2 - _A * _A - _B * _B) / (2.0 * _A * _B)
    cos_delta = max(-1.0, min(1.0, cos_delta))
    delta = math.acos(cos_delta)
    if not elbow_up:
        delta = -delta
    alpha_a = math.atan2(X, Y) - math.atan2(_B * math.sin(delta), _A + _B * math.cos(delta))

    # 4) map planar angles back to URDF joint angles.
    q2 = alpha_a - _PHI_A
    q3 = delta + _PHI_A - _PHI_B

    q = np.array([wrap_pi(q1), wrap_pi(q2), wrap_pi(q3)])

    # 5) polish: FK is exact and cheap, so refine the analytic seed to ~0 residual.
    q = _newton_polish(q, np.array([x, y, z], dtype=float))
    return q, ok


# ----------------------------------------------------------------------------- #
# Faithful firmware port (degrees + millimetres) — reference / real-arm cross-check
# ----------------------------------------------------------------------------- #
L0, L1, L2, L3, L4 = 84.4, 8.14, 128.4, 138.0, 16.8  # mm, from _espmax.h


def fk_firmware(joints_deg):
    """Port of _espmax.cpp forward(): joint angles (deg) -> position (mm)."""
    a1 = math.radians(joints_deg[0]) + math.radians(150.0)
    a2 = math.radians(joints_deg[1])
    a3 = math.radians(joints_deg[2])
    if a1 > 2.0 * math.pi:
        a1 -= 2.0 * math.pi
    beta = a2 - a3
    side = math.sqrt(L2 * L2 + L3 * L3 - 2.0 * L2 * L3 * math.cos(beta))
    cos_g = ((side * side + L2 * L2) - L3 * L3) / (2.0 * side * L2)
    cos_g = min(1.0, cos_g)
    gamma = math.acos(cos_g)
    alpha = (math.pi - a2) - gamma
    z = side * math.sin(alpha)
    r = math.sqrt(max(0.0, side * side - z * z))
    z += L0
    r += L1 + L4
    x = r * math.cos(a1)
    y = r * math.sin(a1)
    return np.array([-x, y, z])


def ik_firmware(pos_mm):
    """Port of _espmax.cpp inverse(): position (mm) -> joint angles (deg)."""
    x = -pos_mm[0]
    y = pos_mm[1]
    z = pos_mm[2]
    if x == 0.0:
        theta1 = math.pi / 2.0 if y >= 0.0 else math.pi / 2.0 * 3.0
    elif y == 0.0:
        theta1 = 0.0 if x > 0.0 else math.pi
    elif x < 0.0:
        theta1 = math.atan(y / x) + math.pi
    else:
        theta1 = math.atan(y / x) + 2.0 * math.pi
    r = math.hypot(x, y) - L1 - L4
    z = z - L0
    beta = math.acos(max(-1.0, min(1.0, (L2 * L2 + L3 * L3 - (r * r + z * z)) / (2.0 * L2 * L3))))
    gamma = math.acos(max(-1.0, min(1.0,
              (L2 * L2 + (r * r + z * z - L3 * L3)) / (2.0 * L2 * math.sqrt(r * r + z * z)))))
    alpha = math.atan2(z, r)
    theta2 = math.pi - (alpha + gamma)
    theta3 = math.pi - (alpha + beta + gamma)
    deg = math.degrees(theta1)
    if deg <= 30.0:
        deg += 360.0
    return np.array([deg - 150.0, math.degrees(theta2), math.degrees(theta3)])


if __name__ == "__main__":
    for q in ([0, 0, 0], [0, 0.4, -0.6], [0.5, 0.3, -0.4]):
        p = forward_kinematics(q)
        print(f"q={q} -> tip={np.round(p, 4)}  passive={passive_wrist(q[1], q[2]):.3f}")
