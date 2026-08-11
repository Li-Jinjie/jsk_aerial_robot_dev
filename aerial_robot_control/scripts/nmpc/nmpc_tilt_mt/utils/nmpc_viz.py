import transformations as tf
import matplotlib.pyplot as plt
from math import ceil
import numpy as np
import datetime
import os
import scienceplots  # DON'T DELETE, it is used indirectly in matplotlib for fonts

legend_alpha = 0.3


class Visualizer:
    def __init__(
        self,
        robot_arch,
        N_sim,
        nx,
        nu,
        x0,
        tilt=False,
        include_servo_model=False,
        include_thrust_model=False,
        include_cog_dist_model=False,
        include_cog_dist_est=False,
        is_record_diff_u=False,
        state_frame="cog",
        ee_p=None,
        ee_q=None,
    ):
        # Store robot architecture
        self.is_bi, self.is_tri, self.is_qd = False, False, False
        if robot_arch == "bi":
            self.is_bi = True
        elif robot_arch == "tri":
            self.is_tri = True
        elif robot_arch == "qd":
            self.is_qd = True
        elif robot_arch == "":
            pass
        else:
            raise ValueError("This robot architecture is not implemented yet!")

        # Store model properties
        self.tilt = tilt
        self.include_servo_model = include_servo_model
        self.include_thrust_model = include_thrust_model
        self.include_cog_dist_model = include_cog_dist_model
        self.include_cog_dist_est = include_cog_dist_est
        self.is_record_diff_u = is_record_diff_u

        if state_frame not in ("cog", "ee"):
            raise ValueError("State frame must be either 'cog' or 'ee'.")
        if state_frame == "ee" and (ee_p is None or ee_q is None):
            raise ValueError("ee_p and ee_q are required when plotting EE-centric states.")
        self.state_frame = state_frame
        self.ee_p = None if ee_p is None else np.asarray(ee_p, dtype=float)
        self.ee_q = None if ee_q is None else np.asarray(ee_q, dtype=float)

        self.x_sim_all = np.ndarray((N_sim + 1, nx))
        self.u_sim_all = np.ndarray((N_sim, nu))

        self.x_sim_all[0, :] = x0
        self.comp_time = np.zeros(N_sim)

        if self.is_record_diff_u:
            self.u_sim_mpc_all = np.ndarray((N_sim, nu))

        self.est_disturb_f_w_all = np.ndarray((N_sim, 3))
        self.est_disturb_tau_g_all = np.ndarray((N_sim, 3))

        self.data_idx = 0

    def update(self, i, x, u):
        self.x_sim_all[i + 1, :] = x  # Next time step
        self.u_sim_all[i, :] = u  # Current time step

        self.data_idx = i + 1

    def update_u_mpc(self, i, u_mpc):
        self.u_sim_mpc_all[i, :] = u_mpc

    def update_est_disturb(self, i, est_disturb_f_w, est_disturb_tau_g):
        self.est_disturb_f_w_all[i, :] = est_disturb_f_w
        self.est_disturb_tau_g_all[i, :] = est_disturb_tau_g

    @staticmethod
    def _rotation_matrix_from_quaternion(qwxyz):
        qw, qx, qy, qz = qwxyz
        return np.array(
            [
                [1 - 2 * qy**2 - 2 * qz**2, 2 * qx * qy - 2 * qw * qz, 2 * qx * qz + 2 * qw * qy],
                [2 * qx * qy + 2 * qw * qz, 1 - 2 * qx**2 - 2 * qz**2, 2 * qy * qz - 2 * qw * qx],
                [2 * qx * qz - 2 * qw * qy, 2 * qy * qz + 2 * qw * qx, 1 - 2 * qx**2 - 2 * qy**2],
            ]
        )

    @staticmethod
    def _quaternion_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ]
        )

    def _get_plot_states(self):
        """Return states expressed at the selected plotting frame."""
        if self.state_frame == "cog":
            return self.x_sim_all

        x_plot = self.x_sim_all.copy()
        rot_be = self._rotation_matrix_from_quaternion(self.ee_q)
        rot_eb = rot_be.T

        for i in range(self.data_idx + 1):
            q_wb = self.x_sim_all[i, 6:10]
            rot_wb = self._rotation_matrix_from_quaternion(q_wb)
            omega_b = self.x_sim_all[i, 10:13]

            x_plot[i, 0:3] = self.x_sim_all[i, 0:3] + rot_wb @ self.ee_p
            x_plot[i, 3:6] = self.x_sim_all[i, 3:6] + rot_wb @ np.cross(omega_b, self.ee_p)
            x_plot[i, 6:10] = self._quaternion_multiply(q_wb, self.ee_q)
            x_plot[i, 10:13] = rot_eb @ omega_b

            if self.include_cog_dist_model:
                force_w = self.x_sim_all[i, -6:-3]
                torque_cog_b = self.x_sim_all[i, -3:]
                force_b = rot_wb.T @ force_w
                x_plot[i, -3:] = rot_eb @ (torque_cog_b - np.cross(self.ee_p, force_b))

        return x_plot

    def visualize(
        self,
        ocp_model_name: str,
        sim_model_name: str,
        ts_ctrl: float,
        ts_sim: float,
        t_total_sim: float,
        t_servo_ctrl: float = 0.0,
        t_servo_sim: float = 0.0,
        t_sqp_start: float = 0.0,
        t_sqp_end: float = 0.0,
    ):
        x_sim_all = self._get_plot_states()
        u_sim_all = self.u_sim_all

        is_plot_sqp = False
        if t_sqp_start != t_sqp_end and t_sqp_end > t_sqp_start:
            is_plot_sqp = True

        # Set up plot
        fig = plt.figure(figsize=(20, 15))
        fig.suptitle(
            f"Controller: {ocp_model_name}, ts_ctrl = {ts_ctrl} s, servo delay: {t_servo_ctrl} s\n"
            f"Simulator: {sim_model_name}, ts_sim = {ts_sim} s, servo delay: {t_servo_sim} s"
        )
        # Number of subplots
        n_plots = 7  # States, Controls and Computation Time
        if self.include_servo_model:
            n_plots += 1  # Additional State: Servo Angle
        if self.include_thrust_model:
            n_plots += 1  # Additional State: Thrust
        if self.tilt:
            n_plots += 1  # Additional Control: Servon Angle
        if self.include_cog_dist_model:
            n_plots += 2  # Additional State: Disturbance Force and Torque
        if self.include_cog_dist_est:
            n_plots += 2  # Disturbance Force and Torque
        if hasattr(self, "u_sim_mpc_all"):
            n_plots += 2  # MPC controls if needed

        # Timeseries
        time_data_x = np.arange(self.data_idx) * ts_sim

        # Plot Position
        plt.subplot(ceil(n_plots / 2), 2, 1)
        plt.plot(time_data_x, x_sim_all[: self.data_idx, 0], label="x")
        plt.plot(time_data_x, x_sim_all[: self.data_idx, 1], label="y")
        plt.plot(time_data_x, x_sim_all[: self.data_idx, 2], label="z")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Position (m)")
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)
            plt.text(1.5, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
            plt.text(
                (t_sqp_start + t_sqp_end) / 2, 0.5, "SQP", horizontalalignment="center", verticalalignment="center"
            )
            plt.text(4.0, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
        plt.grid(True)

        # fmt: off
        # Plot Velocity
        plt.subplot(ceil(n_plots/2), 2, 3)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 3], label="vx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 4], label="vy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 5], label="vz")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Velocity (m/s)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Quaternions
        plt.subplot(ceil(n_plots/2), 2, 5)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 6], label="qw")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 7], label="qx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 8], label="qy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 9], label="qz")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Quaternion")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Convert x_sim_all[:, 6:10] to euler angle
        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            euler[i, :] = tf.euler_from_quaternion(qwxyz, axes="sxyz")

        # Plot Euler Angles
        plt.subplot(ceil(n_plots / 2), 2, 2)
        plt.plot(time_data_x, euler[: self.data_idx, 0], label="roll")
        plt.plot(time_data_x, euler[: self.data_idx, 1], label="pitch")
        plt.plot(time_data_x, euler[: self.data_idx, 2], label="yaw")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Euler Angle (rad)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Angular Velocity
        plt.subplot(ceil(n_plots / 2), 2, 4)
        plt.plot(time_data_x, x_sim_all[: self.data_idx, 10], label="wx")
        plt.plot(time_data_x, x_sim_all[: self.data_idx, 11], label="wy")
        plt.plot(time_data_x, x_sim_all[: self.data_idx, 12], label="wz")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Angular Velocity (rad/s)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Computation Time
        # Remove data below 0.0002s, since all the data of some rounds are not meaningful.
        comp_time_filtered = self.comp_time[self.comp_time > 0.0002]
        print("Average computation time: ", np.mean(comp_time_filtered))
        plt.subplot(ceil(n_plots / 2), 2, 6)
        plt.plot(time_data_x, self.comp_time)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Computation Time (s)")
        plt.grid(True)

        # Plot Servo Angle as State
        x_idx = 12
        if self.tilt and self.include_servo_model:
            plt.subplot(ceil(n_plots/2), 2, 7)
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="a1s")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+2], label="a2s")
            x_idx += 2
            if self.is_tri or self.is_qd:
                plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="a3s")
                x_idx += 1
            if self.is_qd:
                plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="a4s")
                x_idx += 1
            plt.legend(framealpha=legend_alpha)
            # plt.xlabel("Time (s)")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Servo Angle State (rad)")
            plt.grid(True)
            if is_plot_sqp:
                plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Thrust as State
        if self.include_thrust_model:
            plt.subplot(ceil(n_plots/2), 2, 8)
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="ft1s")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+2], label="ft2s")
            x_idx += 2
            if self.is_tri or self.is_qd:
                plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="ft3s")
                x_idx += 1
            if self.is_qd:
                plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="ft4s")
                x_idx += 1
            plt.legend(framealpha=legend_alpha)
            # plt.xlabel("Time (s)")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Thrust State (N)")
            plt.grid(True)
            if is_plot_sqp:
                plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Thrust as Control Input
        if self.include_thrust_model:
            plt.subplot(ceil(n_plots/2), 2, 10)
        else:
            plt.subplot(ceil(n_plots/2), 2, 8)
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 0], label="ft1c")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 1], label="ft2c")
        u_idx = 1
        if self.is_tri or self.is_qd:
            plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 2], label="ft3c")
            u_idx = 2
        if self.is_qd:
            plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 3], label="ft4c")
            u_idx = 3
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Thrust Cmd (N)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Servo Angle as Control Input
        if self.tilt:
            if self.include_servo_model:
                plt.subplot(ceil(n_plots/2), 2, 9)
            else:
                plt.subplot(ceil(n_plots/2), 2, 7)
            plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, u_idx+1], label="a1c")
            plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, u_idx+2], label="a2c")
            if self.is_tri or self.is_qd:
                plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, u_idx+3], label="a3c")
            if self.is_qd:
                plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, u_idx+4], label="a4c")
            plt.legend(framealpha=legend_alpha)
            # plt.xlabel("Time (s)")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Servo Angle Cmd (rad)")
            plt.grid(True)
            if is_plot_sqp:
                plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plot_idx = 10
        # Plot seperate MPC control variable
        if self.is_record_diff_u:
            plt.subplot(ceil(n_plots/2), 2, 11)
            plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 4], label="a1c_mpc")
            plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 5], label="a2c_mpc")
            if self.is_tri or self.is_qd:
                plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 6], label="a3c_mpc")
            if self.is_qd:
                plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 7], label="a4c_mpc")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Servo Angle Cmd MPC (rad)")
            plt.grid(True)

            plt.subplot(ceil(n_plots/2), 2, 12)
            plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 0], label="ft1c_mpc")
            plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 1], label="ft2c_mpc")
            if self.is_tri or self.is_qd:
                plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 2], label="ft3c_mpc")
            if self.is_qd:
                plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 3], label="ft4c_mpc")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Thrust Cmd MPC (N)")
            plt.grid(True)

            plot_idx += 2

        if self.include_cog_dist_model:
            # Plot Disturbance Force as State in NMPC
            plt.subplot(ceil(n_plots/2), 2, plot_idx+1)
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="f_d_x")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+2], label="f_d_y")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+3], label="f_d_z")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Disturbance Force (N)")
            plt.grid(True)

            # Plot Disturbance Torque as State in NMPC
            plt.subplot(ceil(n_plots/2), 2, plot_idx+2)
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+4], label="tau_d_x")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+5], label="tau_d_y")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+6], label="tau_d_z")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Disturbance Torque (N*m)")
            plt.grid(True)

            plot_idx += 2

        if self.include_cog_dist_est:
            # Plot estimated Disturbance Force by MHE
            plt.subplot(ceil(n_plots/2), 2, plot_idx+1)
            plt.plot(time_data_x, self.est_disturb_f_w_all[:self.data_idx, 0], label="est. f_d_x")
            plt.plot(time_data_x, self.est_disturb_f_w_all[:self.data_idx, 1], label="est. f_d_y")
            plt.plot(time_data_x, self.est_disturb_f_w_all[:self.data_idx, 2], label="est. f_d_z")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Est. Disturbance Force (N)")
            plt.grid(True)

            # Plot estimated Disturbance Torque by MHE
            plt.subplot(ceil(n_plots/2), 2, plot_idx+2)
            plt.plot(time_data_x, self.est_disturb_tau_g_all[:self.data_idx, 0], label="est. tau_d_x")
            plt.plot(time_data_x, self.est_disturb_tau_g_all[:self.data_idx, 1], label="est. tau_d_y")
            plt.plot(time_data_x, self.est_disturb_tau_g_all[:self.data_idx, 2], label="est. tau_d_z")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Est. Disturbance Torque (N*m)")
            plt.grid(True)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        plt.show()
        # fmt: on

    def visualize_less(self, ts_sim: float, t_total_sim: float):
        plt.style.use(["science", "grid"])

        # Font size
        plt.rcParams.update({"font.size": 11})  # Default is 10
        label_size = 12

        # To avoid warning of font in pdf check -> Seems no need after using scienceplots.
        # matplotlib.rcParams['pdf.fonttype'] = 42
        # matplotlib.rcParams['ps.fonttype'] = 42

        x_sim_all = self._get_plot_states()
        u_sim_all = self.u_sim_all

        # Timeseries
        time_data_x = np.arange(self.data_idx) * ts_sim
        time_data_u = np.arange(self.data_idx - 1) * ts_sim + ts_sim

        # Set up plot
        fig = plt.figure(figsize=(7, 4.5))
        # title = str(f"Ctrl = {ocp_model_name}, ts ctrl = {ts_ctrl} s, servo delay = {t_servo_ctrl} s")
        # title = title.replace("_", r"\_")
        # fig.suptitle(title)
        ax = plt.subplot(211)

        # fmt: off
        # Plot Position
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 0], label="x")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 1], label="y")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 2], label="z")
        plt.ylabel("Position (m)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.0, 0.65), loc="lower left")

        # Convert Quaternions into Euler Angles using transformations
        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            euler[i, :] = tf.euler_from_quaternion(qwxyz, axes="sxyz")
        euler = euler * 180 / np.pi  # in degree

        # Plot Euler Angles (with y-axis on the right side)
        ax_right = ax.twinx()
        ax_right.plot(time_data_x, euler[:self.data_idx, 0], label="roll", linestyle="--")
        ax_right.plot(time_data_x, euler[:self.data_idx, 1], label="pitch", linestyle="--")
        ax_right.plot(time_data_x, euler[:self.data_idx, 2], label="yaw", linestyle="--")
        ax_right.set_ylabel("Euler Angle ($^\\circ$)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.45, 0.7), loc="lower left")

        # Plot Thrust as Control Input
        ax2 = plt.subplot(212, sharex=ax)
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 0], label="$f_{c1}$")
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 1], label="$f_{c2}$")
        if self.is_tri or self.is_qd:
            plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 2], label="$f_{c3}$")
        if self.is_qd:
            plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 3], label="$f_{c4}$")
        plt.ylabel("Thrust Cmd. (N)", fontsize=label_size)
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=4, bbox_to_anchor=(0.1, 0.75), loc="lower left")

        # Plot Sensor Angle as Control Input (in degree)
        if self.tilt:
            ax2_right = ax2.twinx()
            if self.is_bi:
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 2] * 180 / np.pi, label="$\\alpha_{c1}$",
                               linestyle="--")
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 3] * 180 / np.pi, label="$\\alpha_{c2}$",
                               linestyle="--")
            if self.is_tri:
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 3] * 180 / np.pi, label="$\\alpha_{c1}$",
                               linestyle="--")
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 4] * 180 / np.pi, label="$\\alpha_{c2}$",
                               linestyle="--")
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 5] * 180 / np.pi, label="$\\alpha_{c3}$",
                               linestyle="--")
            if self.is_qd:
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 4] * 180 / np.pi, label="$\\alpha_{c1}$",
                            linestyle="--")
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 5] * 180 / np.pi, label="$\\alpha_{c2}$",
                            linestyle="--")
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 6] * 180 / np.pi, label="$\\alpha_{c3}$",
                        linestyle="--")
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 7] * 180 / np.pi, label="$\\alpha_{c4}$",
                        linestyle="--")
            ax2_right.set_ylabel("Servo Angle Cmd. ($^\\circ$)", fontsize=label_size)
            plt.legend(framealpha=legend_alpha, ncol=4, bbox_to_anchor=(0.1, 0.00), loc="lower left")
            plt.xlim([-0.1, t_total_sim])
            # plt.ylim([-1.0, 1.0])  # -0.8, 0.8; -1.6, 1.6

        # plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.tight_layout()
        fig.subplots_adjust(hspace=0.15)

        plt.show()
        # fmt: on

    def visualize_tracking_actuators(
        self,
        ts_sim: float,
        t_total_sim: float,
        position_cmd: np.ndarray,
        attitude_cmd_q: np.ndarray,
        output_prefix: str = None,
    ):
        """Plot pose tracking and actuator command/state pairs in a 2x2 layout."""
        if not (self.is_qd and self.include_servo_model and self.include_thrust_model):
            raise ValueError(
                "Tracking/actuator visualization requires a quadrotor simulator with servo and thrust states."
            )

        position_cmd = np.asarray(position_cmd)
        attitude_cmd_q = np.asarray(attitude_cmd_q)
        n_data = min(self.data_idx, len(position_cmd), len(attitude_cmd_q))
        if n_data == 0:
            raise ValueError("No simulation/reference data available for visualization.")

        plt.style.use(["science", "grid"])
        plt.rcParams.update(
            {
                "font.size": 14,
                "axes.labelsize": 14,
                "axes.titlesize": 14,
                "legend.fontsize": 12,
            }
        )

        # update() stores the state resulting from command sample i at state row i+1.
        state = self.x_sim_all[1 : n_data + 1]
        control = self.u_sim_all[:n_data]
        time_data = np.arange(n_data) * ts_sim

        euler_state = np.zeros((n_data, 3))
        euler_cmd = np.zeros((n_data, 3))
        for i in range(n_data):
            euler_state[i] = tf.euler_from_quaternion(state[i, 6:10], axes="sxyz")
            euler_cmd[i] = tf.euler_from_quaternion(attitude_cmd_q[i], axes="sxyz")
        euler_state = np.degrees(euler_state)
        euler_cmd = np.degrees(euler_cmd)

        colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
        fig, axes = plt.subplots(2, 2, figsize=(11, 6.5), sharex=True)

        # Position: color identifies x/y/z, line style identifies cmd/state.
        ax = axes[0, 0]
        for idx, label in enumerate(("x", "y", "z")):
            color = colors[idx]
            ax.plot(time_data, position_cmd[:n_data, idx], "--", color=color, label=f"{label} cmd")
            ax.plot(time_data, state[:, idx], "-", color=color, label=f"{label} state")
        ax.set_title("Position tracking")
        ax.set_ylabel("Position (m)")
        ax.set_ylim([-3.5, 3.5])
        ax.legend(framealpha=legend_alpha, ncol=2)

        # Euler angles in degrees.
        ax = axes[0, 1]
        for idx, label in enumerate(("roll", "pitch", "yaw")):
            color = colors[idx]
            ax.plot(time_data, euler_cmd[:, idx], "--", color=color, label=f"{label} cmd")
            ax.plot(time_data, euler_state[:, idx], "-", color=color, label=f"{label} state")
        ax.set_title("Attitude tracking")
        ax.set_ylabel(r"Euler angle ($^\circ$)")
        ax.set_ylim([-45, 70])
        ax.legend(framealpha=legend_alpha, ncol=2)

        # Simulator state order is base(13), servo(4), thrust(4).
        ax = axes[1, 0]
        servo_state_deg = np.degrees(state[:, 13:17])
        servo_cmd_deg = np.degrees(control[:, 4:8])
        for idx in range(4):
            color = colors[idx]
            rotor = idx + 1
            ax.plot(time_data, servo_cmd_deg[:, idx], "--", color=color, label=rf"$\alpha_{{c{rotor}}}$")
            ax.plot(time_data, servo_state_deg[:, idx], "-", color=color, label=rf"$\alpha_{{s{rotor}}}$")
        ax.set_title("Servo command and state")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(r"Servo angle ($^\circ$)")
        ax.legend(framealpha=legend_alpha, ncol=2)

        ax = axes[1, 1]
        for idx in range(4):
            color = colors[idx]
            rotor = idx + 1
            ax.plot(time_data, control[:, idx], "--", color=color, label=rf"$f_{{c{rotor}}}$")
            ax.plot(time_data, state[:, 17 + idx], "-", color=color, label=rf"$f_{{s{rotor}}}$")
        ax.set_title("Thrust command and state")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Thrust (N)")
        ax.legend(framealpha=legend_alpha, ncol=2)

        for ax in axes.flat:
            ax.set_xlim([0.0, t_total_sim])

        plt.tight_layout()
        fig.subplots_adjust(hspace=0.2, wspace=0.2)
        if output_prefix is not None:
            output_prefix = os.path.abspath(output_prefix)
            os.makedirs(os.path.dirname(output_prefix), exist_ok=True)
            fig.savefig(output_prefix + ".png", dpi=200, bbox_inches="tight")
            fig.savefig(output_prefix + ".pdf", bbox_inches="tight")
        plt.show()

    def visualize_rpy(self, ocp_model_name: str, ts_sim: float, t_total_sim: float):
        plt.style.use(["science", "grid"])

        # Font size
        plt.rcParams.update({"font.size": 11})  # Default is 10
        label_size = 14

        x_sim_all = self._get_plot_states()

        # Timeseries
        time_data_x = np.arange(self.data_idx) * ts_sim

        # Set up plot
        fig = plt.figure(figsize=(3.5, 2.0))
        title = str(f"{ocp_model_name}")
        title = title.replace("_", r"\_")
        # fig.title(title)

        # Convert Quaternions into Euler Angles using transformations
        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            euler[i, :] = tf.euler_from_quaternion(qwxyz, axes="sxyz")

        # fmt: off
        # Plot Euler Angles
        plt.plot([0, t_total_sim], [0.5, 0.5], label="ref", linestyle="-.")  # Plot reference as constant 0.5
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
        # fmt: on

    def visualize_disturb(self, ts_sim: float, t_total_sim: float):
        """
        Visualize disturbance wrench, position and 3-2-1 Euler attitude (degrees).

        Parameters
        ----------
        ts_sim : float
            Simulation sampling period [s].
        t_total_sim : float
            X-axis span for all sub-plots [s].
        """
        x_sim_all = self._get_plot_states()

        # ── appearance ────────────────────────────────────────────────
        plt.style.use(["science", "grid"])
        plt.rcParams.update({"font.size": 10})
        legend_alpha = 0.3

        # ── time axis ─────────────────────────────────────────────────
        t = np.arange(self.data_idx) * ts_sim

        # ── quaternion → Euler (deg) ─────────────────────────────────
        euler_rad = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))  # (x,y,z,w) → (w,x,y,z)
            euler_rad[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")
        euler_deg = np.rad2deg(euler_rad)  # convert to degrees

        # ── figure with 8 sub-plots ─────────────────────────────────
        fig = plt.figure(figsize=(7, 10))

        # 1) Disturbance force
        ax1 = plt.subplot(8, 1, 1)
        ax1.plot(t, self.est_disturb_f_w_all[: self.data_idx, 0], label=r"$f_x$")
        ax1.plot(t, self.est_disturb_f_w_all[: self.data_idx, 1], label=r"$f_y$")
        ax1.plot(t, self.est_disturb_f_w_all[: self.data_idx, 2], label=r"$f_z$")
        ax1.set_ylabel(r"Force [N]")
        ax1.legend(framealpha=legend_alpha, ncol=3, loc="upper left")

        # 2) p_x
        ax2 = plt.subplot(8, 1, 2, sharex=ax1)
        ax2.plot(t, x_sim_all[: self.data_idx, 0], label=r"$p_x$")
        ax2.set_ylabel(r"$p_x$ [m]")
        ax2.legend(framealpha=legend_alpha, loc="upper left")

        # 3) p_y
        ax3 = plt.subplot(8, 1, 3, sharex=ax1)
        ax3.plot(t, x_sim_all[: self.data_idx, 1], label=r"$p_y$")
        ax3.set_ylabel(r"$p_y$ [m]")
        ax3.legend(framealpha=legend_alpha, loc="upper left")

        # 4) p_z
        ax4 = plt.subplot(8, 1, 4, sharex=ax1)
        ax4.plot(t, x_sim_all[: self.data_idx, 2], label=r"$p_z$")
        ax4.set_ylabel(r"$p_z$ [m]")
        ax4.legend(framealpha=legend_alpha, loc="upper left")

        # 5) Disturbance torque
        ax5 = plt.subplot(8, 1, 5, sharex=ax1)
        ax5.plot(t, self.est_disturb_tau_g_all[: self.data_idx, 0], label=r"$\tau_x$")
        ax5.plot(t, self.est_disturb_tau_g_all[: self.data_idx, 1], label=r"$\tau_y$")
        ax5.plot(t, self.est_disturb_tau_g_all[: self.data_idx, 2], label=r"$\tau_z$")
        ax5.set_ylabel(r"Torque [N$\!\cdot$m]")
        ax5.legend(framealpha=legend_alpha, ncol=3, loc="upper left")

        # 6) roll (φ)
        ax6 = plt.subplot(8, 1, 6, sharex=ax1)
        ax6.plot(t, euler_deg[: self.data_idx, 0], label=r"$\phi$")
        ax6.set_ylabel(r"Roll [$^\circ$]")
        ax6.legend(framealpha=legend_alpha, loc="upper left")

        # 7) pitch (θ)
        ax7 = plt.subplot(8, 1, 7, sharex=ax1)
        ax7.plot(t, euler_deg[: self.data_idx, 1], label=r"$\theta$")
        ax7.set_ylabel(r"Pitch [$^\circ$]")
        ax7.legend(framealpha=legend_alpha, loc="upper left")

        # 8) yaw (ψ)
        ax8 = plt.subplot(8, 1, 8, sharex=ax1)
        ax8.plot(t, euler_deg[: self.data_idx, 2], label=r"$\psi$")
        ax8.set_ylabel(r"Yaw [$^\circ$]")
        ax8.set_xlabel(r"Time [s]")
        ax8.legend(framealpha=legend_alpha, loc="upper left")

        # ── common limits & layout ────────────────────────────────────
        ax1.set_xlim([-0.1, t_total_sim])
        fig.subplots_adjust(hspace=0.25)
        plt.tight_layout()
        plt.show()

    def visualize_nothing_but_save(self, model_name: str, sim_name: str):
        """
        Visualize nothing but save the figure with the model and simulation names.
        :param model_name:
        :param sim_name:
        :return:
        """
        folder_path = "../../../../scripts/nmpc/sim_data"

        # if there is no sim_data folder, create it
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        # save self.x_sim_all, self.u_sim_all, self.est_disturb_f_w_all, and self.est_disturb_tau_g_all to a file
        time_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        np.save(f"{folder_path}/{time_stamp}_{model_name}_{sim_name}_x_sim_all.npy", self.x_sim_all)
        np.save(f"{folder_path}/{time_stamp}_{model_name}_{sim_name}_u_sim_all.npy", self.u_sim_all)
        np.save(f"{folder_path}/{time_stamp}_{model_name}_{sim_name}_est_disturb_f_w_all.npy", self.est_disturb_f_w_all)
        np.save(
            f"{folder_path}/{time_stamp}_{model_name}_{sim_name}_est_disturb_tau_g_all.npy", self.est_disturb_tau_g_all
        )

        print(f"Data saved to {folder_path}/{time_stamp}_{model_name}_{sim_name}_*.npy files.")


class SensorVisualizer:
    def __init__(self, N_sim):
        self.gyro_sim_all = np.ndarray((N_sim, 3))

        self.ang_acc_real_all = np.ndarray((N_sim, 3))
        self.ang_acc_est_all = np.ndarray((N_sim, 3))

        self.sf_sim_all = np.ndarray((N_sim, 3))
        self.lin_acc_est_all = np.ndarray((N_sim, 3))

        self.data_idx = 0

    def update_gyro(self, i, gyro, ang_acc_est, ang_acc_real):
        self.gyro_sim_all[i, :] = gyro
        self.ang_acc_est_all[i, :] = ang_acc_est
        self.ang_acc_real_all[i, :] = ang_acc_real

    def vis_gyro(self, ts_sim: float, t_total_sim: float):
        plt.style.use(["science", "grid"])

        # Font size
        plt.rcParams.update({"font.size": 11})

        fig = plt.figure(figsize=(7, 4.5))

        # Timeseries
        time_data_x = np.arange(self.data_idx) * ts_sim

        # fmt: off
        # Plot Angular Velocity as States
        ax = plt.subplot(211)
        plt.plot(time_data_x, self.gyro_sim_all[:self.data_idx, 0], label="gyro_x")
        plt.plot(time_data_x, self.gyro_sim_all[:self.data_idx, 1], label="gyro_y")
        plt.plot(time_data_x, self.gyro_sim_all[:self.data_idx, 2], label="gyro_z")
        plt.ylabel("Gyro Angular Vel. (rad/s)")
        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.0, 0.7), loc="lower left")

        # Plot Angular Acceleration as States and Estimates
        ax2 = plt.subplot(212, sharex=ax)
        plt.plot(time_data_x, self.ang_acc_real_all[:self.data_idx, 0], label="ang_acc_real_x")
        plt.plot(time_data_x, self.ang_acc_real_all[:self.data_idx, 1], label="ang_acc_real_y")
        plt.plot(time_data_x, self.ang_acc_real_all[:self.data_idx, 2], label="ang_acc_real_z")
        plt.plot(time_data_x, self.ang_acc_est_all[:self.data_idx, 0], label="ang_acc_est_x")
        plt.plot(time_data_x, self.ang_acc_est_all[:self.data_idx, 1], label="ang_acc_est_y")
        plt.plot(time_data_x, self.ang_acc_est_all[:self.data_idx, 2], label="ang_acc_est_z")
        plt.ylabel("Angular Acc. (rad/s^2)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.0, 0.7), loc="lower left")

        plt.xlim([-0.1, t_total_sim])

        plt.tight_layout()
        fig.subplots_adjust(hspace=0.15)

        plt.show()
        # fmt: on
