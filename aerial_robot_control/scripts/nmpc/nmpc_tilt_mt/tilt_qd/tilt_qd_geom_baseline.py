#!/usr/bin/env python
# -*- encoding: ascii -*-
"""
Classic baseline controller for tiltable quadrotors:
a geometric (PD) controller computes the desired 6D wrench from position, velocity,
attitude and angular velocity, then a pseudo-inverse allocation converts the wrench
into thrust and servo angle commands. This is the most common control method for
tiltable multirotors and serves as the control group for the NMPC methods.
"""
import numpy as np
import transformations as tf

from .qd_reference_generator import QDNMPCReferenceGenerator
from ..archive import phys_param_beetle_art as phys_art


class GeomControllerTiltQd:
    """
    Geometric controller + pseudo-inverse allocation for a tiltable quadrotor.

    The controller runs at a fixed rate (T_samp) and outputs the same 8-dim command
    as the NMPC controllers: [ft1c, ft2c, ft3c, ft4c, a1c, a2c, a3c, a4c].
    The wrench-to-actuator mapping reuses the allocation matrix of
    QDNMPCReferenceGenerator to stay identical to the NMPC reference generation.
    """

    def __init__(
        self,
        phys=phys_art,
        kp=np.array([8.0, 8.0, 8.0]),  # position gain [1/s^2]
        kv=np.array([5.0, 5.0, 5.0]),  # velocity gain [1/s]
        kr=np.array([25.0, 25.0, 25.0]),  # attitude gain [1/s^2]
        kw=np.array([10.0, 10.0, 10.0]),  # angular velocity gain [1/s]
        ts_ctrl=0.01,
        thrust_min=0.0,
        thrust_max=23.0,
        a_min=-3.15,
        a_max=3.15,
    ):
        self.phys = phys
        self.mass = phys.mass
        self.gravity = phys.gravity
        self.inertia = np.diag([phys.Ixx, phys.Iyy, phys.Izz])

        self.kp = kp
        self.kv = kv
        self.kr = kr
        self.kw = kw

        self.thrust_min = thrust_min
        self.thrust_max = thrust_max
        self.a_min = a_min
        self.a_max = a_max

        # Same key as the NMPC classes so that sim_nmpc.py can read the control rate
        self.params = {"T_samp": ts_ctrl}

        # Flags read by sim_nmpc.py and the Visualizer. The baseline has no actuator
        # or disturbance model, so all model flags are off.
        self.tilt = True
        self.include_servo_model = False
        self.include_servo_derivative = False
        self.include_thrust_model = False
        self.include_cog_dist_model = False

        # State/input dimensions of the controller (p, v, q, w / ft x4 + a x4)
        self.nx = 13
        self.nu = 8

        # Reuse the allocation matrix of the NMPC reference generator. The nmpc object
        # is only needed for compute_trajectory(), which the baseline never calls.
        self._ref_gen = QDNMPCReferenceGenerator(
            None,
            phys.p1_b,
            phys.p2_b,
            phys.p3_b,
            phys.p4_b,
            phys.dr1,
            phys.dr2,
            phys.dr3,
            phys.dr4,
            phys.kq_d_kt,
            phys.mass,
            phys.gravity,
        )
        self.alloc_mat_pinv = self._ref_gen.get_alloc_mat_pinv()

        self.a_cmd_prev = np.zeros(4)

    @staticmethod
    def _vee(mtx):
        """Map a skew-symmetric matrix to a 3D vector."""
        return np.array([mtx[2, 1], mtx[0, 2], mtx[1, 0]])

    def compute_control(self, x, target_xyz, target_rpy) -> np.ndarray:
        """
        Compute thrust and servo angle commands from the current state and target pose.

        :param x: Current state, at least [p(3), v(3), q_wxyz(4), w(3)]
        :param target_xyz: Target position
        :param target_rpy: Target orientation (roll, pitch, yaw)
        :return u_cmd: Command [ft1c, ft2c, ft3c, ft4c, a1c, a2c, a3c, a4c]
        """
        p = x[0:3]
        v = x[3:6]
        q = x[6:10] / np.linalg.norm(x[6:10])  # qw, qx, qy, qz
        w = x[10:13]

        rot_wb = tf.quaternion_matrix(q)[:3, :3]

        # Position loop: desired force in World frame with gravity compensation
        p_ref = np.asarray(target_xyz).flatten()
        f_des_w = self.mass * (self.kp * (p_ref - p) - self.kv * v + np.array([0.0, 0.0, self.gravity]))
        f_des_b = rot_wb.T @ f_des_w

        # Attitude loop: geometric control on SO(3)
        roll, pitch, yaw = np.asarray(target_rpy).flatten()
        rot_wb_ref = tf.euler_matrix(roll, pitch, yaw, axes="sxyz")[:3, :3]
        e_rot = 0.5 * self._vee(rot_wb_ref.T @ rot_wb - rot_wb.T @ rot_wb_ref)
        e_w = w  # w_ref = 0
        tau_des_b = self.inertia @ (-self.kr * e_rot - self.kw * e_w) + np.cross(w, self.inertia @ w)

        # Pseudo-inverse allocation, same as QDNMPCReferenceGenerator.compute_trajectory
        target_wrench = np.concatenate((f_des_b, tau_des_b)).reshape(6, 1)
        target_force = self.alloc_mat_pinv @ target_wrench

        ft_cmd = np.array(
            [
                np.sqrt(target_force[0, 0] ** 2 + target_force[1, 0] ** 2),
                np.sqrt(target_force[2, 0] ** 2 + target_force[3, 0] ** 2),
                np.sqrt(target_force[4, 0] ** 2 + target_force[5, 0] ** 2),
                np.sqrt(target_force[6, 0] ** 2 + target_force[7, 0] ** 2),
            ]
        )

        a_cmd = np.array(
            [
                np.arctan2(target_force[0, 0], target_force[1, 0]),
                np.arctan2(target_force[2, 0], target_force[3, 0]),
                np.arctan2(target_force[4, 0], target_force[5, 0]),
                np.arctan2(target_force[6, 0], target_force[7, 0]),
            ]
        )

        # Keep servo angles continuous before clipping so that the tracking of the
        # previous command is not corrupted by saturation
        a_cmd = QDNMPCReferenceGenerator._ensure_servo_angles_continuity(a_cmd, self.a_cmd_prev)
        self.a_cmd_prev = a_cmd

        ft_cmd = np.clip(ft_cmd, self.thrust_min, self.thrust_max)
        a_cmd = np.clip(a_cmd, self.a_min, self.a_max)

        return np.concatenate((ft_cmd, a_cmd))
