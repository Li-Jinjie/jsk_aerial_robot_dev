"""
 Created by li-jinjie on 24-3-9.
"""
import time

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import scienceplots
import argparse
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
from tf_conversions import transformations as tf

from nmpc_over_act_no_servo_delay import NMPCOverActNoServoDelay
from nmpc_over_act_old_servo_cost import NMPCOverActOldServoCost
from nmpc_over_act_no_servo_new_cost import NMPCOverActNoServoNewCost
from nmpc_over_act_vel_input import NMPCOverActVelInput
from nmpc_over_act_full import NMPCOverActFull
from nmpc_over_act_full_i_term import NMPCOverActFullITerm

legend_alpha = 0.5


def create_acados_sim_solver(ocp_model: AcadosModel, ts_sim: float) -> AcadosSimSolver:
    acados_sim = AcadosSim()
    acados_sim.model = ocp_model
    n_params = ocp_model.p.size()[0]
    acados_sim.dims.np = n_params  # TODO: seems that the np needn't to be set manually in the latest version of acados
    acados_sim.parameter_values = np.zeros(n_params)
    acados_sim.solver_options.T = ts_sim
    acados_sim_solver = AcadosSimSolver(acados_sim, json_file="acados_ocp_" + ocp_model.name + ".json")
    return acados_sim_solver


class Visualizer:
    def __init__(self, N_sim, nx, nu, x0):
        self.x_sim_all = np.ndarray((N_sim + 1, nx))
        self.u_sim_all = np.ndarray((N_sim, nu))
        self.x_sim_all[0, :] = x0
        self.comp_time = np.zeros(N_sim)

        self.data_idx = 0

    def update(self, i, x, u):
        self.x_sim_all[i + 1, :] = x
        self.u_sim_all[i, :] = u

        self.data_idx = i + 1

    def visualize(self, ocp_model_name: str, sim_model_name: str, ts_ctrl: float, ts_sim: float, t_total_sim: float,
                  t_servo_ctrl: float = 0.0, t_servo_sim: float = 0.0, t_sqp_start: float = 0, t_sqp_end: float = 0):
        x_sim_all = self.x_sim_all
        u_sim_all = self.u_sim_all

        is_plot_sqp = False
        if t_sqp_start != t_sqp_end and t_sqp_end > t_sqp_start:
            is_plot_sqp = True

        fig = plt.figure(figsize=(20, 15))
        fig.suptitle(
            f"Controller: {ocp_model_name}, ts_ctrl = {ts_ctrl} s, servo delay: {t_servo_ctrl} s\n"
            f"Simulator: {sim_model_name}, ts_sim = {ts_sim} s, servo delay: {t_servo_sim} s"
        )

        time_data_x = np.arange(self.data_idx) * ts_sim

        plt.subplot(5, 2, 1)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 0], label="x")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 1], label="y")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 2], label="z")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Position (m)")
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)
            plt.text(1.5, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
            plt.text((t_sqp_start + t_sqp_end) / 2, 0.5, "SQP", horizontalalignment="center",
                     verticalalignment="center")
            plt.text(4.0, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
        plt.grid(True)

        plt.subplot(5, 2, 3)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 3], label="vx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 4], label="vy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 5], label="vz")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Velocity (m/s)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 5)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 6], label="qw")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 7], label="qx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 8], label="qy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 9], label="qz")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Quaternion")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 7)
        # use tf2 to convert x_sim_all[:, 6:10] to euler angle
        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))
            euler[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")

        plt.plot(time_data_x, euler[:self.data_idx, 0], label="roll")
        plt.plot(time_data_x, euler[:self.data_idx, 1], label="pitch")
        plt.plot(time_data_x, euler[:self.data_idx, 2], label="yaw")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Euler Angle (rad)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 2)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 10], label="wx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 11], label="wy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 12], label="wz")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Body Rate (rad/s)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 4)
        if x_sim_all.shape[1] > 13:
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 13], label="a1")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 14], label="a2")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 15], label="a3")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 16], label="a4")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Servo Angle (rad)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 6)
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 0], label="ft1")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 1], label="ft2")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 2], label="ft3")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 3], label="ft4")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Thrust (N)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 8)
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 4], label="a1c")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 5], label="a2c")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 6], label="a3c")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 7], label="a4c")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Servo Sngle Cmd (rad)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        print("Average computation time: ", np.mean(self.comp_time))
        plt.subplot(5, 2, 9)
        plt.plot(time_data_x, self.comp_time)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Computation Time (s)")
        plt.grid(True)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        plt.show()

    def visualize_less(self, ocp_model_name: str, sim_model_name: str, ts_ctrl: float, ts_sim: float,
                       t_total_sim: float, t_servo_ctrl: float = 0.0, t_servo_sim: float = 0.0):

        plt.style.use(["science", "grid"])

        # set font size
        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 15

        # # to avoid the warning of font in pdf check. Seems no need after using scienceplots.
        # matplotlib.rcParams['pdf.fonttype'] = 42
        # matplotlib.rcParams['ps.fonttype'] = 42

        x_sim_all = self.x_sim_all
        u_sim_all = self.u_sim_all

        fig = plt.figure(figsize=(7, 3))
        title = str(f"Ctrl = {ocp_model_name}, ts ctrl = {ts_ctrl} s, servo delay = {t_servo_ctrl} s")
        title = title.replace("_", r"\_")
        fig.suptitle(title)

        time_data_x = np.arange(self.data_idx) * ts_sim

        ax = plt.subplot(1, 2, 1)

        if x_sim_all.shape[1] > 13:
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 13], label="$\\alpha_1$")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 14], label="$\\alpha_2$")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 15], label="$\\alpha_3$")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 16], label="$\\alpha_4$")
        plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.xlim([0, t_total_sim])
        plt.ylabel("Servo Angle State (rad)", fontsize=label_size)
        plt.ylim([-1.0, 1.0])  # -0.18, 0.18; -1.0, 1.0

        ax_right = ax.twinx()
        ax_right.plot([0, t_total_sim], [1.0, 1.0], label="$z_r$", linestyle="-.", color="blue")
        ax_right.plot(time_data_x, x_sim_all[:self.data_idx, 2], label="$z$", linestyle="--", color="black")
        ax_right.set_ylabel("Height (m)", fontsize=label_size)
        ax_right.legend(framealpha=legend_alpha, loc="upper left")

        time_data_u = np.arange(self.data_idx - 1) * ts_sim

        plt.subplot(1, 2, 2)
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 4], label="$\\alpha_{c1}$")
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 5], label="$\\alpha_{c2}$")
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 6], label="$\\alpha_{c3}$")
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 7], label="$\\alpha_{c4}$")
        plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.xlim([0, t_total_sim])
        plt.ylabel("Servo Angle Cmd (rad)", fontsize=label_size)
        plt.ylim([-1.0, 1.0])  # -0.8, 0.8; -1.6, 1.6

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        plt.show()

    def visualize_rpy(self, ocp_model_name: str, sim_model_name: str, ts_ctrl: float, ts_sim: float,
                      t_total_sim: float, t_servo_ctrl: float = 0.0, t_servo_sim: float = 0.0):

        plt.style.use(["science", "grid"])

        # set font size
        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 14

        x_sim_all = self.x_sim_all
        u_sim_all = self.u_sim_all

        fig = plt.figure(figsize=(3.5, 2.0))
        title = str(f"{ocp_model_name}")
        title = title.replace("_", r"\_")
        # fig.title(title)

        time_data_x = np.arange(self.data_idx) * ts_sim

        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))
            euler[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")

        # plt.title(title)
        # plot reference as 0.5
        plt.plot([0, t_total_sim], [0.5, 0.5], label="ref", linestyle="-.")
        plt.plot(time_data_x, euler[:self.data_idx, 0], label="roll")
        plt.plot(time_data_x, euler[:self.data_idx, 1], label="pitch")
        plt.plot(time_data_x, euler[:self.data_idx, 2], label="yaw")
        plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.xlim([0, t_total_sim])
        plt.ylim([-0.02, 0.52])
        plt.ylabel("Euler Angle (rad)", fontsize=label_size)

        # plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.tight_layout()

        plt.show()


if __name__ == "__main__":
    # read arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models.")
    parser.add_argument(
        "model",
        type=int,
        help="The NMPC model to be simulated. Options: 0 (no_servo_delay), 1 (old servo cost), 2 (full).",
    )
    parser.add_argument("-p", "--plot_type", type=int, default=0, help="The type of plot. Options: 0 (full), 1, 2.")

    args = parser.parse_args()

    # ========== init ==========
    # ---------- Controller ----------
    if args.model == 0:
        nmpc = NMPCOverActNoServoDelay()
    elif args.model == 1:
        nmpc = NMPCOverActOldServoCost()
    elif args.model == 2:
        nmpc = NMPCOverActNoServoNewCost()
    elif args.model == 3:
        nmpc = NMPCOverActFull()
    elif args.model == 4:
        nmpc = NMPCOverActFullITerm()
    elif args.model == 5:
        nmpc = NMPCOverActVelInput()
        alpha_integ = np.zeros(4)
    else:
        raise ValueError(f"Invalid model {args.model}.")

    # check if there is t_servo in the controller
    if hasattr(nmpc, "t_servo"):
        t_servo_ctrl = nmpc.t_servo
    else:
        t_servo_ctrl = 0.0
    ts_ctrl = nmpc.ts_ctrl

    # ocp solver
    ocp_solver = nmpc.get_ocp_solver()
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    n_param = ocp_solver.acados_ocp.dims.np  # np has been used for numpy

    x_init = np.zeros(nx)
    x_init[6] = 1.0  # qw
    u_init = np.zeros(nu)

    for stage in range(ocp_solver.N + 1):
        ocp_solver.set(stage, "x", x_init)
    for stage in range(ocp_solver.N):
        ocp_solver.set(stage, "u", u_init)

    # ---------- Simulator ----------
    sim_nmpc = NMPCOverActFull()

    if hasattr(sim_nmpc, "t_servo"):
        t_servo_sim = sim_nmpc.t_servo
    else:
        t_servo_sim = 0.0

    ts_sim = 0.005

    t_total_sim = 15.0
    if args.plot_type == 1:
        t_total_sim = 2.0
    if args.plot_type == 2:
        t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # sim solver
    sim_nmpc.get_ocp_model()
    sim_solver = create_acados_sim_solver(sim_nmpc.get_ocp_model(), ts_sim)
    nx_sim = sim_solver.acados_sim.dims.nx

    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw

    # ---------- Others ----------
    xr_ur_converter = nmpc.get_xr_ur_converter()
    viz = Visualizer(N_sim, nx_sim, nu, x_init_sim)

    is_sqp_change = False
    t_sqp_start = 2.5
    t_sqp_end = 3.0

    # ========== update ==========
    u_cmd = u_init
    t_ctl = 0.0
    x_now_sim = x_init_sim
    for i in range(N_sim):
        # --------- update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim

        # --------- update state estimation ---------
        x_now = x_now_sim[:nx]  # the dimension of x_now may be smaller than x_now_sim

        # -------- update control target --------
        target_xyz = np.array([[0.0, 0.0, 1.0]]).T
        target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if args.plot_type == 2:
            target_xyz = np.array([[0.0, 0.0, 0.0]]).T
            target_rpy = np.array([[0.5, 0.5, 0.5]]).T

        if t_total_sim > 3.0:
            if 3.0 <= t_now < 5.5:
                assert t_sqp_end <= 3.0
                target_xyz = np.array([[1.0, 1.0, 1.0]]).T
                target_rpy = np.array([[0.0, 0.0, 0.0]]).T

            if t_now >= 5.5:
                target_xyz = np.array([[1.0, 1.0, 1.0]]).T

                roll = 30.0 / 180.0 * np.pi
                pitch = 0.0 / 180.0 * np.pi
                yaw = 0.0 / 180.0 * np.pi
                target_rpy = np.array([[roll, pitch, yaw]]).T

        xr, ur = xr_ur_converter.pose_point_2_xr_ur(target_xyz, target_rpy)

        if args.plot_type == 2:
            if nx > 13:
                xr[:, 13:] = 0.0
            ur[:, 4:] = 0.0

        # -------- sqp mode --------
        if is_sqp_change and t_sqp_start > t_sqp_end:
            if t_now >= t_sqp_start:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP"

            if t_now >= t_sqp_end:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP_RTI"

        # -------- update solver --------
        comp_time_start = time.time()

        if t_ctl >= ts_ctrl:
            t_ctl = 0.0

            # 0 ~ N-1
            for j in range(ocp_solver.N):
                yr = np.concatenate((xr[j, :], ur[j, :]))
                ocp_solver.set(j, "yref", yr)
                quaternion_r = xr[j, 6:10]
                params = np.zeros(n_param)
                params[0:4] = quaternion_r
                ocp_solver.set(j, "p", params)  # for nonlinear quaternion error

            # N
            yr = xr[ocp_solver.N, :]
            ocp_solver.set(ocp_solver.N, "yref", yr)  # final state of x, no u
            quaternion_r = xr[ocp_solver.N, 6:10]
            params = np.zeros(n_param)
            params[0:4] = quaternion_r
            ocp_solver.set(ocp_solver.N, "p", params)  # for nonlinear quaternion error

            # feedback, take the first action
            try:
                u_cmd = ocp_solver.solve_for_x0(x_now)
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                break

        comp_time_end = time.time()
        viz.comp_time[i] = comp_time_end - comp_time_start

        # if nmpc is NMPCOverActNoServoNewCost
        if type(nmpc) is NMPCOverActNoServoNewCost:
            xr_ur_converter.update_a_prev(u_cmd.item(4), u_cmd.item(5), u_cmd.item(6), u_cmd.item(7))

        if type(nmpc) is NMPCOverActVelInput:
            alpha_integ += u_cmd[4:] * ts_ctrl
            u_cmd[4:] = alpha_integ  # convert from delta to absolute

        # --------- update simulation ----------
        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", u_cmd)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # --------- update visualizer ----------
        viz.update(i, x_now_sim, u_cmd)  # note that the recording frequency of u_cmd is the same as ts_sim

    # ========== visualize ==========
    if args.plot_type == 0:
        viz.visualize(ocp_solver.acados_ocp.model.name, sim_solver.model_name, ts_ctrl, ts_sim, t_total_sim,
                      t_servo_ctrl=t_servo_ctrl, t_servo_sim=t_servo_sim)
    elif args.plot_type == 1:
        viz.visualize_less(ocp_solver.acados_ocp.model.name, sim_solver.model_name, ts_ctrl, ts_sim, t_total_sim,
                           t_servo_ctrl=t_servo_ctrl, t_servo_sim=t_servo_sim)
    elif args.plot_type == 2:
        viz.visualize_rpy(ocp_solver.acados_ocp.model.name, sim_solver.model_name, ts_ctrl, ts_sim, t_total_sim,
                          t_servo_ctrl=t_servo_ctrl, t_servo_sim=t_servo_sim)