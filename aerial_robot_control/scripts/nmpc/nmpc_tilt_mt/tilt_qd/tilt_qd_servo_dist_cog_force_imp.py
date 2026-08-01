#!/usr/bin/env python
# -*- encoding: ascii -*-
# This version is the real running version of tilt_qd_servo_dist_force_imp.py by eliminate the EE conversion.
import numpy as np
import casadi as ca

from .fake_sensor import FakeSensor
from .qd_nmpc_base import QDNMPCBase
from . import phys_param_beetle_omni as phys_omni


class NMPCTiltQdServoCoGForceImpedance(QDNMPCBase):
    """CoG-centric force-impedance NMPC with ordinary attitude tracking."""

    def __init__(self, build: bool = True, phys=phys_omni):
        self.model_name = "tilt_qd_servo_dist_cog_force_imp_mdl"
        self.phys = phys

        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = False
        self.include_thrust_model = False
        self.include_cog_dist_model = True
        self.include_cog_dist_parameter = True
        self.include_impedance = True

        self.read_params("controller", "nmpc", "beetle_omni", "BeetleNMPCFullServoForceImp.yaml")

        super().__init__(build)

        self.fake_sensor = FakeSensor(self.include_servo_model, self.include_thrust_model, self.include_cog_dist_model)

    def get_cost_function(self, lin_acc_w=None, ang_acc_b=None):
        # fmt: off
        _, qe_x, qe_y, qe_z = self._quaternion_multiply(
            self.qwr, -self.qxr, -self.qyr, -self.qzr,
            self.qw, self.qx, self.qy, self.qz
        )

        state_y = ca.vertcat(
            self.p,
            self.v,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            self.w,
            self.a_s,
            ca.times(lin_acc_w, self.mp) - self.fds_w,
            self.tau_ds_b,
        )

        # The terminal cost must remain independent of the control-dependent
        # acceleration while retaining the same residual dimension.
        state_y_e = ca.vertcat(
            self.p,
            self.v,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            self.w,
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
        # Translational tracking is represented by the force-impedance block.
        # Attitude and angular velocity retain ordinary tracking weights.
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
        xr[:, 0:3] = target_xyz
        xr[:, 6:10] = np.asarray(target_qwxyz).reshape(4)
        xr[:, 13:17] = a_ref

        ur = np.zeros([nn, nu])
        ur[:, 0:4] = ft_ref
        return xr, ur


if __name__ == "__main__":
    print("Please run the gen_nmpc_code.py in the nmpc folder to generate the code for this controller.")
