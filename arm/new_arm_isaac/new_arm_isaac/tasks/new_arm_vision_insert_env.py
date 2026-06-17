"""NewArm-VisionInsert — end-to-end RL insertion from WRIST-CAMERA observations (no privileged target).

Subclasses NewArm-Insert (identical plug-tip tracking, socket sampling, and ground-truth reward),
but the POLICY OBSERVATION replaces the privileged base-frame pos_error with what the wrist camera
actually measures: the two socket holes projected into the camera image as normalized bearings
(nx, ny) in [-1, 1] plus a visibility flag, followed by proprioception (q, dq, prev_action).

This is exactly the information vision_align extracts on the real arm (hole pixel positions), so the
trained policy is real-deployable and its final alignment is driven by the camera measurement. When
the socket is outside the camera FOV the bearings are zeroed with visible=0, so the policy must learn
to move the arm to bring the socket into view (active perception). The REWARD still uses ground truth
(privileged, training-only) — this is the standard "observable actor, privileged reward" setup.

Projection: the wrist camera is a child of end_effector_link at config.CAMERA_OFFSET with local
orientation RotateY(CAMERA_PITCH) * RotateZ(CAMERA_ROLL) (matches usd_camera.bake). A USD camera
looks down its own -Z, so a point's depth is -z_cam; nx = (x_cam/depth)/tan(HFOV/2),
ny = (y_cam/depth)/tan(VFOV/2). Camera-frame X/Y already include the roll, so nx/ny match the rolled
(portrait) image the real camera produces.
"""

from __future__ import annotations

import math

import torch

from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply, quat_conjugate, quat_mul

from new_arm_isaac import config
from new_arm_isaac.tasks.new_arm_insert_env import NewArmInsertEnv, NewArmInsertEnvCfg


def _cam_local_quat():
    """Constant camera-in-EE orientation quaternion (wxyz) = RotateY(pitch) * RotateZ(roll)."""
    p = math.radians(config.CAMERA_PITCH_DEG) / 2.0
    r = math.radians(config.CAMERA_ROLL_DEG) / 2.0
    w1, x1, y1, z1 = math.cos(p), 0.0, math.sin(p), 0.0       # RotateY(pitch)
    w2, x2, y2, z2 = math.cos(r), 0.0, 0.0, math.sin(r)       # RotateZ(roll)
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


@configclass
class NewArmVisionInsertEnvCfg(NewArmInsertEnvCfg):
    # ACTOR obs (camera): holeL(nx,ny,vis) + holeR(nx,ny,vis) + q(3) + dq(3) + prev_action(3)
    observation_space = 15
    # CRITIC obs (privileged, asymmetric AC): GT pos_error(3) + q(3) + dq(3) + prev_action(3).
    # The critic sees the true state for accurate value estimates (stabilises PPO under the actor's
    # partial/noisy camera observability); the actor stays camera-only -> still real-deployable.
    state_space = 12
    cam_pixel_noise = 0.012   # gaussian sigma on the normalized bearings (~perception error)


class NewArmVisionInsertEnv(NewArmInsertEnv):
    cfg: NewArmVisionInsertEnvCfg

    def _init_handles(self):
        super()._init_handles()
        if getattr(self, "_vis_ready", False):
            return
        self._vis_ready = True
        clq = _cam_local_quat()
        self._cam_lq = torch.tensor(clq, device=self.device).unsqueeze(0).expand(self.num_envs, 4).contiguous()
        self._cam_off = torch.tensor(
            [float(v) for v in config.CAMERA_OFFSET_XYZ], device=self.device
        ).unsqueeze(0).expand(self.num_envs, 3).contiguous()
        f = config.CAMERA_FOCAL_LENGTH
        self._tan_h = config.CAMERA_HORIZONTAL_APERTURE / (2.0 * f)   # tan(HFOV/2)
        self._tan_v = config.CAMERA_VERTICAL_APERTURE / (2.0 * f)     # tan(VFOV/2)
        self._near = float(config.CAMERA_CLIPPING_RANGE[0])
        self._hole_half = 0.5 * config.SOCKET_HOLE_SPACING

    def _hole_world(self):
        """World positions of the two socket holes (centre +/- spacing/2 along the socket's lateral)."""
        local = self.target_pos_w - self.scene.env_origins
        az = torch.atan2(local[:, 1], local[:, 0])
        lat = torch.stack([-torch.sin(az), torch.cos(az), torch.zeros_like(az)], dim=-1)
        return self.target_pos_w + self._hole_half * lat, self.target_pos_w - self._hole_half * lat

    def _project(self, P):
        """World point P (N,3) -> normalized image bearing (nx, ny) and visibility (all N,)."""
        ee_pos = self.robot.data.body_pos_w[:, self.tip_body_id[0], :]
        ee_quat = self.robot.data.body_quat_w[:, self.tip_body_id[0], :]
        cam_pos = ee_pos + quat_apply(ee_quat, self._cam_off)
        cam_quat = quat_mul(ee_quat, self._cam_lq)
        pc = quat_apply(quat_conjugate(cam_quat), P - cam_pos)   # point in camera frame
        depth = -pc[:, 2]                                        # USD camera looks down -Z
        d = torch.clamp(depth, min=1e-4)
        nx = (pc[:, 0] / d) / self._tan_h
        ny = (pc[:, 1] / d) / self._tan_v
        vis = (depth > self._near) & (nx.abs() <= 1.0) & (ny.abs() <= 1.0)
        return nx, ny, vis

    def _feat(self, nx, ny, vis):
        v = vis.float()
        n = self.cfg.cam_pixel_noise
        nx = (nx + n * torch.randn_like(nx)) * v   # zero the bearing when not visible
        ny = (ny + n * torch.randn_like(ny)) * v
        return torch.stack([nx, ny, v], dim=-1)

    def _get_observations(self) -> dict:
        self._init_handles()
        self._visualize_markers()
        q = self.robot.data.joint_pos[:, self.arm_joint_ids]
        dq = self.robot.data.joint_vel[:, self.arm_joint_ids]
        # ACTOR: wrist-camera hole bearings + proprioception (what the real arm measures)
        hL, hR = self._hole_world()
        fL = self._feat(*self._project(hL))
        fR = self._feat(*self._project(hR))
        actor = torch.cat([fL, fR, q, dq, self.prev_actions], dim=-1)
        # CRITIC: privileged ground-truth state (training-only) for asymmetric actor-critic
        pos_err = self.target_pos_w - self._tip_pos_w()
        critic = torch.cat([pos_err, q, dq, self.prev_actions], dim=-1)
        return {"policy": actor, "critic": critic}
