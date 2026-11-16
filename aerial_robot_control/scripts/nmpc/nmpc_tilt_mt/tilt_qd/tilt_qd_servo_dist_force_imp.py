#!/usr/bin/env python
# -*- encoding: ascii -*-
# This version is the debug version that include the EE conversion.
# For running, please use the tilt_qd_servo_dist_cog_force_imp.py
import numpy as np
import casadi as ca
from .qd_nmpc_base import QDNMPCBase
from .fake_sensor import FakeSensor
from . import phys_param_beetle_omni as phys_omni


class NMPCTiltQdServoForceImpedance(QDNMPCBase):
    """Tiltable quadrotor NMPC with force impedance and attitude tracking."""

    def __init__(self, build: bool = True, phys=phys_omni, use_ee_acceleration: bool = True):
        self.model_name = "tilt_qd_servo_dist_force_imp_mdl"
        self.phys = phys

        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = False
        self.include_thrust_model = False
        self.include_cog_dist_model = True
        self.include_cog_dist_parameter = True
        # Keep the impedance parameter layout expected by QDNMPCBase. Only the
        # translational part is used in the cost below.
        self.include_impedance = True
        self.use_ee_acceleration = use_ee_acceleration

        self.read_params("controller", "nmpc", "beetle_omni", "BeetleNMPCFullServoForceImp.yaml")

        super().__init__(build)

        self.fake_sensor = FakeSensor(self.include_servo_model, self.include_thrust_model, self.include_cog_dist_model)

    def get_cost_function(self, lin_acc_w=None, ang_acc_b=None):
        # fmt: off
        q_wt_w, q_wt_x, q_wt_y, q_wt_z = self._quaternion_multiply(
            self.qw, self.qx, self.qy, self.qz,
            self.ee_q[0], self.ee_q[1], self.ee_q[2], self.ee_q[3]
        )

        qe_w, qe_x, qe_y, qe_z = self._quaternion_multiply(
            self.qwr, -self.qxr, -self.qyr, -self.qzr,
            q_wt_w, q_wt_x, q_wt_y, q_wt_z
        )

        rot_wb = self._get_rot_wb_ca(self.qw, self.qx, self.qy, self.qz)
        skew_w = self._get_skew_symmetric_matrix(self.w)
        skew_ang_acc = self._get_skew_symmetric_matrix(ang_acc_b)
        if self.use_ee_acceleration:
            lin_acc_ee_w = lin_acc_w + rot_wb @ (
                skew_ang_acc @ self.ee_p + skew_w @ skew_w @ self.ee_p
            )
        else:
            lin_acc_ee_w = lin_acc_w

        rot_bt = self._get_rot_wb_ca(self.ee_q[0], self.ee_q[1], self.ee_q[2], self.ee_q[3])
        rot_tb = rot_bt.T

        # Force disturbance participates in the translational impedance cost.
        # Torque disturbance remains outside the compliant residual: it has zero
        # direct cost weight, while still entering the rotational dynamics for
        # disturbance-aware attitude control.
        state_y = ca.vertcat(
            self.p + rot_wb @ self.ee_p,
            self.v + rot_wb @ skew_w @ self.ee_p,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            rot_tb @ self.w,
            self.a_s,
            ca.times(lin_acc_ee_w, self.mp) - self.fds_w,
            self.tau_ds_b,
        )

        # Terminal cost must not depend on the control-dependent acceleration.
        # Keep the same residual dimension and enforce the state-only part of
        # the impedance relation at the final shooting node.
        state_y_e = ca.vertcat(
            self.p + rot_wb @ self.ee_p,
            self.v + rot_wb @ skew_w @ self.ee_p,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            rot_tb @ self.w,
            self.a_s,
            -self.fds_w,
            self.tau_ds_b,
        )

        control_y = ca.vertcat(
            self.ft_c,
            self.a_c - self.a_s  # a_c_ref must be zero!
        )

        return state_y, state_y_e, control_y
        # fmt: on

    def get_weights(self):
        # Start with ordinary attitude tracking weights. Translational tracking
        # weights are replaced by the force-impedance block below.
        Q = np.diag(
            [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                self.params["Qq_xy"],
                self.params["Qq_xy"],
                self.params["Qq_z"],
                self.params["Qw_xy"],
                self.params["Qw_xy"],
                self.params["Qw_z"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        )

        pM_imp = np.eye(3)
        pD_imp = np.diag([self.params["Qv_xy"], self.params["Qv_xy"], self.params["Qv_z"]])
        pK_imp = np.diag([self.params["Qp_xy"], self.params["Qp_xy"], self.params["Qp_z"]])

        enlarge_factor = self.params["enlarge_factor"]
        p_weight = np.concatenate([pK_imp, pD_imp, pM_imp])
        p_weight_mtx = np.dot(p_weight * enlarge_factor, p_weight.T * enlarge_factor)
        Q[0:6, 0:6] = p_weight_mtx[0:6, 0:6]
        Q[17:20, 17:20] = p_weight_mtx[6:9, 6:9]
        Q[0:6, 17:20] = p_weight_mtx[0:6, 6:9]
        Q[17:20, 0:6] = p_weight_mtx[6:9, 0:6]

        print("Q: \n", Q)

        R = np.diag(
            [
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rac_d"],
                self.params["Rac_d"],
                self.params["Rac_d"],
                self.params["Rac_d"],
            ]
        )
        print("R: \n", R)

        return Q, R

    def get_reference(self, target_xyz, target_qwxyz, ft_ref, a_ref):
        ocp = self.get_ocp()
        nn = ocp.solver_options.N_horizon
        nx = ocp.dims.nx
        nu = ocp.dims.nu

        xr = np.zeros([nn + 1, nx])
        xr[:, 0] = target_xyz[0]
        xr[:, 1] = target_xyz[1]
        xr[:, 2] = target_xyz[2]
        xr[:, 6] = target_qwxyz[0]
        xr[:, 7] = target_qwxyz[1]
        xr[:, 8] = target_qwxyz[2]
        xr[:, 9] = target_qwxyz[3]
        xr[:, 13] = a_ref[0]
        xr[:, 14] = a_ref[1]
        xr[:, 15] = a_ref[2]
        xr[:, 16] = a_ref[3]

        ur = np.zeros([nn, nu])
        ur[:, 0] = ft_ref[0]
        ur[:, 1] = ft_ref[1]
        ur[:, 2] = ft_ref[2]
        ur[:, 3] = ft_ref[3]

        return xr, ur


if __name__ == "__main__":
    print("Please run the gen_nmpc_code.py in the nmpc folder to generate the code for this controller.")
