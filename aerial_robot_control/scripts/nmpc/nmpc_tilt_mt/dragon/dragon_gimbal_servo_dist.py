#!/usr/bin/env python3

import os

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcpSolver

from ..tilt_qd.qd_nmpc_base import QDNMPCBase
from . import phys_param_dragon as phys_dragon
from .dragon_reference_generator import DragonNMPCReferenceGenerator


class DragonGimbalNMPCBase(QDNMPCBase):
    num_modules = 4
    num_gimbals = 8

    def __init__(
        self,
        model_name,
        include_thrust_model,
        include_cog_dist_model,
        build=True,
        phys=phys_dragon,
    ):
        self.model_name = model_name
        self.phys = phys
        self.num_rotors = self.num_modules
        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = False
        self.include_thrust_model = include_thrust_model
        self.include_cog_dist_model = include_cog_dist_model
        self.include_cog_dist_parameter = True
        self.include_impedance = False
        self.read_params("controller", "nmpc", "dragon", "DragonNMPCGimbalServoDist.yaml")
        super().__init__(build)

    @staticmethod
    def _rotation_x(angle):
        return ca.vertcat(
            ca.horzcat(1, 0, 0),
            ca.horzcat(0, ca.cos(angle), -ca.sin(angle)),
            ca.horzcat(0, ca.sin(angle), ca.cos(angle)),
        )

    @staticmethod
    def _rotation_y(angle):
        return ca.vertcat(
            ca.horzcat(ca.cos(angle), 0, ca.sin(angle)),
            ca.horzcat(0, 1, 0),
            ca.horzcat(-ca.sin(angle), 0, ca.cos(angle)),
        )

    def create_acados_model(self):
        self.p = ca.SX.sym("p", 3)
        self.v = ca.SX.sym("v", 3)
        self.q = ca.SX.sym("q", 4)
        self.w = ca.SX.sym("w", 3)
        self.qw, self.qx, self.qy, self.qz = [self.q[index] for index in range(4)]

        self.a_s = ca.SX.sym("a_s", self.num_gimbals)
        state = ca.vertcat(self.p, self.v, self.q, self.w, self.a_s)

        if self.include_thrust_model:
            self.ft_s = ca.SX.sym("ft_s", self.num_modules)
            state = ca.vertcat(state, self.ft_s)

        if self.include_cog_dist_model:
            self.fds_w = ca.SX.sym("fds_w", 3)
            self.tau_ds_b = ca.SX.sym("tau_ds_b", 3)
            state = ca.vertcat(state, self.fds_w, self.tau_ds_b)
        else:
            self.fds_w = ca.SX.zeros(3)
            self.tau_ds_b = ca.SX.zeros(3)

        self.ft_c = ca.SX.sym("ft_c", self.num_modules)
        self.a_c = ca.SX.sym("a_c", self.num_gimbals)
        controls = ca.vertcat(self.ft_c, self.a_c)

        q_ref = ca.SX.sym("q_ref", 4)
        self.qwr, self.qxr, self.qyr, self.qzr = [q_ref[index] for index in range(4)]

        mass = ca.SX.sym("mass")
        gravity = ca.SX.sym("gravity")
        Ixx = ca.SX.sym("Ixx")
        Iyy = ca.SX.sym("Iyy")
        Izz = ca.SX.sym("Izz")
        Ixy = ca.SX.sym("Ixy")
        Ixz = ca.SX.sym("Ixz")
        Iyz = ca.SX.sym("Iyz")
        inertia_parameters = ca.vertcat(Ixx, Iyy, Izz, Ixy, Ixz, Iyz)

        positions = []
        link_quaternions = []
        geometry_parameters = ca.SX.zeros(0, 1)
        for index in range(self.num_modules):
            position = ca.SX.sym(f"p{index + 1}_b", 3)
            quaternion = ca.SX.sym(f"q_bl{index + 1}", 4)
            positions.append(position)
            link_quaternions.append(quaternion)
            geometry_parameters = ca.vertcat(geometry_parameters, position, quaternion)

        t_rotor = ca.SX.sym("t_rotor")
        t_servo = ca.SX.sym("t_servo")
        self.fdp_w = ca.SX.sym("fdp_w", 3)
        self.tau_dp_b = ca.SX.sym("tau_dp_b", 3)
        physical_parameters = ca.vertcat(
            mass,
            gravity,
            inertia_parameters,
            geometry_parameters,
            t_rotor,
            t_servo,
        )
        parameters = ca.vertcat(q_ref, physical_parameters, self.fdp_w, self.tau_dp_b)

        rotation_wb = self._get_rot_wb_ca(self.qw, self.qx, self.qy, self.qz)
        force_body = ca.SX.zeros(3)
        torque_body = ca.SX.zeros(3)
        for index in range(self.num_modules):
            q_link = link_quaternions[index]
            rotation_bl = self._get_rot_wb_ca(q_link[0], q_link[1], q_link[2], q_link[3])
            roll = self.a_s[2 * index]
            pitch = self.a_s[2 * index + 1]
            thrust = self.ft_s[index] if self.include_thrust_model else self.ft_c[index]
            direction_body = rotation_bl @ self._rotation_x(roll) @ self._rotation_y(pitch) @ ca.vertcat(0, 0, 1)
            module_force = direction_body * thrust
            force_body += module_force
            torque_body += ca.cross(positions[index], module_force)

        inertia = ca.vertcat(
            ca.horzcat(Ixx, Ixy, Ixz),
            ca.horzcat(Ixy, Iyy, Iyz),
            ca.horzcat(Ixz, Iyz, Izz),
        )
        inertia_inv = ca.inv(inertia)
        gravity_world = ca.vertcat(0, 0, -gravity)

        state_dot = ca.vertcat(
            self.v,
            (rotation_wb @ force_body + self.fds_w + self.fdp_w) / mass + gravity_world,
            (-self.w[0] * self.q[1] - self.w[1] * self.q[2] - self.w[2] * self.q[3]) / 2,
            (self.w[0] * self.q[0] + self.w[2] * self.q[2] - self.w[1] * self.q[3]) / 2,
            (self.w[1] * self.q[0] - self.w[2] * self.q[1] + self.w[0] * self.q[3]) / 2,
            (self.w[2] * self.q[0] + self.w[1] * self.q[1] - self.w[0] * self.q[2]) / 2,
            inertia_inv @ (-ca.cross(self.w, inertia @ self.w) + torque_body + self.tau_ds_b + self.tau_dp_b),
            (self.a_c - self.a_s) / t_servo,
        )
        if self.include_thrust_model:
            state_dot = ca.vertcat(state_dot, (self.ft_c - self.ft_s) / t_rotor)
        if self.include_cog_dist_model:
            state_dot = ca.vertcat(state_dot, ca.SX.zeros(6))

        dynamics = ca.Function(
            "f",
            [state, controls],
            [state_dot],
            ["state", "control_input"],
            ["state_dot"],
            {"allow_free": True},
        )
        x_dot = ca.SX.sym("x_dot", state.size1())

        qe_w, qe_x, qe_y, qe_z = self._quaternion_multiply(
            self.qwr,
            -self.qxr,
            -self.qyr,
            -self.qzr,
            self.qw,
            self.qx,
            self.qy,
            self.qz,
        )
        state_cost = ca.vertcat(
            self.p,
            self.v,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            self.w,
            self.a_s,
        )
        if self.include_thrust_model:
            state_cost = ca.vertcat(state_cost, self.ft_s)
        if self.include_cog_dist_model:
            state_cost = ca.vertcat(state_cost, self.fds_w, self.tau_ds_b)

        model = AcadosModel()
        model.name = self.model_name
        model.x = state
        model.xdot = x_dot
        model.u = controls
        model.p = parameters
        model.f_expl_expr = dynamics(state, controls)
        model.f_impl_expr = x_dot - dynamics(state, controls)
        model.cost_y_expr = ca.vertcat(state_cost, self.ft_c, self.a_c - self.a_s)
        model.cost_y_expr_e = state_cost
        model.con_h_expr = (self.a_c - self.a_s) / t_servo
        return model

    def get_weights(self):
        state_weights = [
            self.params["Qp_xy"],
            self.params["Qp_xy"],
            self.params["Qp_z"],
            self.params["Qv_xy"],
            self.params["Qv_xy"],
            self.params["Qv_z"],
            0.0,
            self.params["Qq_xy"],
            self.params["Qq_xy"],
            self.params["Qq_z"],
            self.params["Qw_xy"],
            self.params["Qw_xy"],
            self.params["Qw_z"],
        ]
        state_weights.extend([self.params["Qa"]] * self.num_gimbals)
        if self.include_thrust_model:
            state_weights.extend([self.params["Qft"]] * self.num_modules)
        if self.include_cog_dist_model:
            state_weights.extend([0.0] * 6)

        input_weights = [self.params["Rt"]] * self.num_modules
        input_weights.extend([self.params["Rac_d"]] * self.num_gimbals)
        return np.diag(state_weights), np.diag(input_weights)

    def get_cost_function(self, lin_acc_w=None, ang_acc_b=None):
        raise NotImplementedError("The DRAGON model assembles its cost with its two-axis gimbal state.")

    def create_acados_ocp_solver(self, build=True):
        ocp = self.get_ocp()
        nx = ocp.model.x.size1()
        nu = ocp.model.u.size1()
        n_param = ocp.model.p.size1()
        Q, R = self.get_weights()

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
        ocp.cost.W_e = Q

        servo_indices = np.arange(13, 13 + self.num_gimbals)
        ocp.constraints.idxbx = np.concatenate((np.array([3, 4, 5, 10, 11, 12]), servo_indices))
        ocp.constraints.lbx = np.concatenate(
            (
                [self.params["v_min"]] * 3,
                [self.params["w_min"]] * 3,
                [self.params["a_min"]] * self.num_gimbals,
            )
        )
        ocp.constraints.ubx = np.concatenate(
            (
                [self.params["v_max"]] * 3,
                [self.params["w_max"]] * 3,
                [self.params["a_max"]] * self.num_gimbals,
            )
        )
        if self.include_thrust_model:
            thrust_state_indices = np.arange(13 + self.num_gimbals, 13 + self.num_gimbals + self.num_modules)
            ocp.constraints.idxbx = np.concatenate((ocp.constraints.idxbx, thrust_state_indices))
            ocp.constraints.lbx = np.concatenate((ocp.constraints.lbx, [self.params["thrust_min"]] * self.num_modules))
            ocp.constraints.ubx = np.concatenate((ocp.constraints.ubx, [self.params["thrust_max"]] * self.num_modules))

        ocp.constraints.idxbx_e = ocp.constraints.idxbx.copy()
        ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
        ocp.constraints.ubx_e = ocp.constraints.ubx.copy()

        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.lbu = np.concatenate(
            (
                [self.params["thrust_min"]] * self.num_modules,
                [self.params["a_min"]] * self.num_gimbals,
            )
        )
        ocp.constraints.ubu = np.concatenate(
            (
                [self.params["thrust_max"]] * self.num_modules,
                [self.params["a_max"]] * self.num_gimbals,
            )
        )
        ocp.constraints.lh = np.full(self.num_gimbals, -self.params["servo_rate_max"])
        ocp.constraints.uh = np.full(self.num_gimbals, self.params["servo_rate_max"])

        x_ref = np.zeros(nx)
        x_ref[6] = 1.0
        hover_thrust, hover_angles = self._nominal_allocation(
            np.array([0.0, 0.0, self.phys.mass * self.phys.gravity, 0.0, 0.0, 0.0])
        )
        x_ref[13 : 13 + self.num_gimbals] = hover_angles
        if self.include_thrust_model:
            thrust_state_start = 13 + self.num_gimbals
            x_ref[thrust_state_start : thrust_state_start + self.num_modules] = hover_thrust
        ocp.constraints.x0 = x_ref

        u_ref = np.zeros(nu)
        u_ref[: self.num_modules] = hover_thrust
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref

        self.acados_init_p = np.zeros(n_param)
        self.acados_init_p[0] = 1.0
        self.acados_init_p[4 : 4 + len(self.phys.physical_param_list)] = self.phys.physical_param_list
        ocp.parameter_values = self.acados_init_p

        ocp.solver_options.tf = self.params["T_horizon"]
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hpipm_mode = "BALANCE"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver_cond_N = self.params["N_steps"]

        json_path = os.path.join(".", f"{ocp.model.name}_acados_ocp.json")
        return AcadosOcpSolver(ocp, json_file=json_path, build=build)

    def _nominal_allocation(self, target_wrench):
        generator = DragonNMPCReferenceGenerator(self, self.phys.p_b, self.phys.q_bl, self.phys.mass, self.phys.gravity)
        return generator.allocate_wrench(target_wrench)

    def _create_reference_generator(self):
        return DragonNMPCReferenceGenerator(self, self.phys.p_b, self.phys.q_bl, self.phys.mass, self.phys.gravity)

    def get_reference(self, target_xyz, target_qwxyz, thrust_ref, angle_ref):
        ocp = self.get_ocp()
        horizon = ocp.solver_options.N_horizon
        nx = ocp.dims.nx
        nu = ocp.dims.nu

        xr = np.zeros((horizon + 1, nx))
        xr[:, :3] = np.asarray(target_xyz).reshape(1, 3)
        xr[:, 6:10] = np.asarray(target_qwxyz).reshape(1, 4)
        xr[:, 13 : 13 + self.num_gimbals] = np.asarray(angle_ref).reshape(1, self.num_gimbals)
        if self.include_thrust_model:
            thrust_state_start = 13 + self.num_gimbals
            xr[:, thrust_state_start : thrust_state_start + self.num_modules] = np.asarray(thrust_ref).reshape(1, 4)

        ur = np.zeros((horizon, nu))
        ur[:, : self.num_modules] = np.asarray(thrust_ref).reshape(1, 4)
        return xr, ur


class NMPCDragonGimbalServoDist(DragonGimbalNMPCBase):
    def __init__(self, build=True, phys=phys_dragon):
        super().__init__(
            "dragon_gimbal_servo_dist_mdl",
            include_thrust_model=False,
            include_cog_dist_model=True,
            build=build,
            phys=phys,
        )


class NMPCDragonGimbalServoThrust(DragonGimbalNMPCBase):
    def __init__(self, build=True, phys=phys_dragon):
        super().__init__(
            "dragon_gimbal_servo_thrust_mdl",
            include_thrust_model=True,
            include_cog_dist_model=False,
            build=build,
            phys=phys,
        )
