#!/usr/bin/env python3
"""Batched (torch) forward kinematics for the new_arm — for IsaacLab RL.

A GPU-friendly port of new_arm_isaac.kinematics.forward_kinematics: given a tensor
of actuated joint angles (N, 3) it returns the wrist-pivot positions (N, 3) in the
base frame. Same URDF geometry, so it agrees with the numpy/firmware versions.

Also provides the passive-wrist coupling so the env can command the leveled wrist.
Dependency-free except torch (no IsaacLab import) so it can be unit-tested.
"""

import math

import torch

# URDF joint origins (metres), same as kinematics.py.
_T1 = (0.00043239, -0.00184063, 0.08396122)
_T2 = (0.00814224, -0.00043239, 0.00161718)
_T3 = (0.001, -0.00856, 0.12839612)
_T4 = (0.12531939, 0.01099, -0.05778452)

# Matches kinematics.PASSIVE_OFFSET: the firmware "- pi/2" term is dropped so the
# wrist is level (0) at home. Keep in sync with kinematics.py.
PASSIVE_OFFSET = 2.0 * math.pi  # was 2*pi - pi/2; -pi/2 removed => home EE = 0
PASSIVE_C_SH = -1.0
PASSIVE_C_EL = -1.0


def _rz(q):
    """(N,) angle -> (N,3,3) rotation about Z."""
    c, s = torch.cos(q), torch.sin(q)
    z, o = torch.zeros_like(q), torch.ones_like(q)
    return torch.stack([
        torch.stack([c, -s, z], dim=-1),
        torch.stack([s, c, z], dim=-1),
        torch.stack([z, z, o], dim=-1),
    ], dim=-2)


def _ry(q):
    """(N,) angle -> (N,3,3) rotation about Y."""
    c, s = torch.cos(q), torch.sin(q)
    z, o = torch.zeros_like(q), torch.ones_like(q)
    return torch.stack([
        torch.stack([c, z, s], dim=-1),
        torch.stack([z, o, z], dim=-1),
        torch.stack([-s, z, c], dim=-1),
    ], dim=-2)


def forward_kinematics(q, device=None):
    """q: (N, 3) actuated joint angles (rad) -> tip (wrist pivot) positions (N, 3)."""
    if device is None:
        device = q.device
    n = q.shape[0]

    def vec(t):
        return torch.tensor(t, dtype=q.dtype, device=device).expand(n, 3)

    def mv(r, v):
        return torch.bmm(r, v.unsqueeze(-1)).squeeze(-1)

    r1 = _rz(q[:, 0])
    r2 = torch.bmm(r1, _ry(q[:, 1]))
    r3 = torch.bmm(r2, _ry(q[:, 2]))
    return vec(_T1) + mv(r1, vec(_T2)) + mv(r2, vec(_T3)) + mv(r3, vec(_T4))


def passive_wrist(q_shoulder, q_elbow):
    """Coupled passive wrist (rad), wrapped to (-pi, pi]. Tensors in, tensor out."""
    v = PASSIVE_OFFSET + PASSIVE_C_SH * q_shoulder + PASSIVE_C_EL * q_elbow
    return torch.atan2(torch.sin(v), torch.cos(v))


# Workspace bounds for sampling reach targets (metres, base frame). The planar reach
# is A+B ~ 0.266 m; keep r below that and away from the base axis.
WORKSPACE = {
    "z": (0.02, 0.34),
    "r_min": 0.05,
    "r_max": 0.26,
}
