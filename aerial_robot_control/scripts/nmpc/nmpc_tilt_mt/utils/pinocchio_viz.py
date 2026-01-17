#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Pinocchio-specific visualization for multi-body dynamics
Provides additional plots for joint torques, energies, and multi-body effects
"""

import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # DON'T DELETE, it is used indirectly in matplotlib for fonts

legend_alpha = 0.3


class PinocchioVisualizer:
    """
    Visualizer for Pinocchio multi-body simulation data.
    Shows joint torques, energies, center of mass, and thrust forces.
    """

    def __init__(self, multibody_data: dict, x_sim_all: np.ndarray, u_sim_all: np.ndarray):
        """
        Initialize Pinocchio visualizer.

        Args:
            multibody_data: Dictionary from simulator.get_multibody_info()
            x_sim_all: State history (N+1, nx)
            u_sim_all: Control history (N, nu)
        """
        self.multibody_data = multibody_data
        self.x_sim_all = x_sim_all
        self.u_sim_all = u_sim_all

    def visualize_multibody(self, ts_sim: float, t_total_sim: float):
        """
        Visualize multi-body dynamics information.

        Args:
            ts_sim: Simulation timestep
            t_total_sim: Total simulation time
        """
        plt.style.use(["science", "grid"])
        plt.rcParams.update({"font.size": 11})

        # Create figure with subplots
        fig = plt.figure(figsize=(20, 15))
        fig.suptitle("Pinocchio Multi-Body Dynamics Visualization", fontsize=16, fontweight="bold")

        # Time vector
        N = len(self.multibody_data["kinetic_energy"])
        time_data = np.arange(N) * ts_sim

        # --- 1. Joint Torques ---
        ax1 = plt.subplot(4, 2, 1)
        joint_torques = self.multibody_data["joint_torques"]
        plt.plot(time_data, joint_torques[:, 0], label="Gimbal 1 τ", linewidth=2)
        plt.plot(time_data, joint_torques[:, 1], label="Gimbal 2 τ", linewidth=2)
        plt.ylabel("Joint Torque (N·m)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Gimbal Joint Torques")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 2. Joint Velocities ---
        ax2 = plt.subplot(4, 2, 2)
        joint_vels = self.multibody_data["joint_velocities"]
        plt.plot(time_data, joint_vels[:, 0], label="Gimbal 1 ω", linewidth=2)
        plt.plot(time_data, joint_vels[:, 1], label="Gimbal 2 ω", linewidth=2)
        plt.ylabel("Joint Velocity (rad/s)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Gimbal Joint Velocities")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 3. Energy (Kinetic, Potential, Total) ---
        ax3 = plt.subplot(4, 2, 3)
        KE = self.multibody_data["kinetic_energy"]
        PE = self.multibody_data["potential_energy"]
        TE = KE + PE
        plt.plot(time_data, KE, label="Kinetic Energy", linewidth=2)
        plt.plot(time_data, PE, label="Potential Energy", linewidth=2)
        plt.plot(time_data, TE, label="Total Energy", linewidth=2, linestyle="--")
        plt.ylabel("Energy (J)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("System Energy")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 4. Energy Rate (Power) ---
        ax4 = plt.subplot(4, 2, 4)
        if len(TE) > 1:
            dTE_dt = np.diff(TE) / ts_sim
            time_data_diff = time_data[:-1]
            plt.plot(time_data_diff, dTE_dt, label="dE/dt (Power)", linewidth=2, color="red")
            plt.ylabel("Power (W)")
            plt.xlabel("Time (s)")
            plt.legend(framealpha=legend_alpha)
            plt.title("Energy Rate of Change")
            plt.xlim([0, t_total_sim])
            plt.grid(True)

        # --- 5. Center of Mass Position ---
        ax5 = plt.subplot(4, 2, 5)
        com = self.multibody_data["com_position"]
        plt.plot(time_data, com[:, 0], label="CoM x", linewidth=2)
        plt.plot(time_data, com[:, 1], label="CoM y", linewidth=2)
        plt.plot(time_data, com[:, 2], label="CoM z", linewidth=2)
        plt.ylabel("Position (m)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Center of Mass Position")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 6. Thrust Force Magnitude ---
        ax6 = plt.subplot(4, 2, 6)
        thrust_forces = self.multibody_data["thrust_forces"]  # (N, 2, 3)
        thrust1_mag = np.linalg.norm(thrust_forces[:, 0, :], axis=1)
        thrust2_mag = np.linalg.norm(thrust_forces[:, 1, :], axis=1)
        plt.plot(time_data, thrust1_mag, label="Thrust 1 |F|", linewidth=2)
        plt.plot(time_data, thrust2_mag, label="Thrust 2 |F|", linewidth=2)
        plt.ylabel("Force Magnitude (N)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Thrust Force Magnitude")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 7. Thrust Force Direction (z-component in world frame) ---
        ax7 = plt.subplot(4, 2, 7)
        thrust1_z = thrust_forces[:, 0, 2]
        thrust2_z = thrust_forces[:, 1, 2]
        plt.plot(time_data, thrust1_z, label="Thrust 1 F_z", linewidth=2)
        plt.plot(time_data, thrust2_z, label="Thrust 2 F_z", linewidth=2)
        plt.ylabel("Force (N)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Thrust Force Z-Component (World Frame)")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 8. Gimbal Angles (from state) ---
        ax8 = plt.subplot(4, 2, 8)
        gimbal_angles = self.x_sim_all[:-1, 13:15]  # Remove last element to match time_data
        plt.plot(time_data, gimbal_angles[:, 0], label="Gimbal 1 α", linewidth=2)
        plt.plot(time_data, gimbal_angles[:, 1], label="Gimbal 2 α", linewidth=2)
        # Also plot commands
        if self.u_sim_all.shape[1] >= 4:
            plt.plot(time_data, self.u_sim_all[:, 2], label="α1 cmd", linestyle="--", alpha=0.7)
            plt.plot(time_data, self.u_sim_all[:, 3], label="α2 cmd", linestyle="--", alpha=0.7)
        plt.ylabel("Angle (rad)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Gimbal Angles (State vs Command)")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        plt.tight_layout()
        plt.show()

    def visualize_comparison(self, standard_x: np.ndarray, standard_u: np.ndarray, ts_sim: float, t_total_sim: float):
        """
        Compare Pinocchio simulation with standard single-body simulation.

        Args:
            standard_x: State history from standard simulator (N+1, nx)
            standard_u: Control history from standard simulator (N, nu)
            ts_sim: Simulation timestep
            t_total_sim: Total simulation time
        """
        plt.style.use(["science", "grid"])
        plt.rcParams.update({"font.size": 11})

        fig = plt.figure(figsize=(20, 12))
        fig.suptitle("Pinocchio vs Standard Single-Body Simulation", fontsize=16, fontweight="bold")

        # Time vectors
        N_pin = self.x_sim_all.shape[0] - 1
        N_std = standard_x.shape[0] - 1
        time_pin = np.arange(N_pin + 1) * ts_sim
        time_std = np.arange(N_std + 1) * ts_sim

        # --- 1. Position Comparison ---
        ax1 = plt.subplot(3, 2, 1)
        plt.plot(time_pin, self.x_sim_all[:, 0], label="Pinocchio x", linewidth=2)
        plt.plot(time_pin, self.x_sim_all[:, 1], label="Pinocchio y", linewidth=2)
        plt.plot(time_pin, self.x_sim_all[:, 2], label="Pinocchio z", linewidth=2)
        plt.plot(time_std, standard_x[:, 0], label="Standard x", linestyle="--", alpha=0.7)
        plt.plot(time_std, standard_x[:, 1], label="Standard y", linestyle="--", alpha=0.7)
        plt.plot(time_std, standard_x[:, 2], label="Standard z", linestyle="--", alpha=0.7)
        plt.ylabel("Position (m)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha, ncol=2)
        plt.title("Position Comparison")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 2. Position Error ---
        ax2 = plt.subplot(3, 2, 2)
        min_len = min(N_pin + 1, N_std + 1)
        pos_error = self.x_sim_all[:min_len, 0:3] - standard_x[:min_len, 0:3]
        time_err = np.arange(min_len) * ts_sim
        plt.plot(time_err, pos_error[:, 0], label="Δx", linewidth=2)
        plt.plot(time_err, pos_error[:, 1], label="Δy", linewidth=2)
        plt.plot(time_err, pos_error[:, 2], label="Δz", linewidth=2)
        plt.ylabel("Position Error (m)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Position Error (Pinocchio - Standard)")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 3. Velocity Comparison ---
        ax3 = plt.subplot(3, 2, 3)
        plt.plot(time_pin, self.x_sim_all[:, 3], label="Pinocchio vx", linewidth=2)
        plt.plot(time_pin, self.x_sim_all[:, 4], label="Pinocchio vy", linewidth=2)
        plt.plot(time_pin, self.x_sim_all[:, 5], label="Pinocchio vz", linewidth=2)
        plt.plot(time_std, standard_x[:, 3], label="Standard vx", linestyle="--", alpha=0.7)
        plt.plot(time_std, standard_x[:, 4], label="Standard vy", linestyle="--", alpha=0.7)
        plt.plot(time_std, standard_x[:, 5], label="Standard vz", linestyle="--", alpha=0.7)
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha, ncol=2)
        plt.title("Velocity Comparison")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 4. Quaternion qw Comparison ---
        ax4 = plt.subplot(3, 2, 4)
        plt.plot(time_pin, self.x_sim_all[:, 6], label="Pinocchio qw", linewidth=2)
        plt.plot(time_std, standard_x[:, 6], label="Standard qw", linestyle="--", alpha=0.7)
        plt.ylabel("Quaternion qw")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Quaternion Scalar Comparison")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 5. Gimbal Angle Comparison ---
        ax5 = plt.subplot(3, 2, 5)
        plt.plot(time_pin, self.x_sim_all[:, 13], label="Pinocchio α1", linewidth=2)
        plt.plot(time_pin, self.x_sim_all[:, 14], label="Pinocchio α2", linewidth=2)
        plt.plot(time_std, standard_x[:, 13], label="Standard α1", linestyle="--", alpha=0.7)
        plt.plot(time_std, standard_x[:, 14], label="Standard α2", linestyle="--", alpha=0.7)
        plt.ylabel("Angle (rad)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Gimbal Angle Comparison")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        # --- 6. Total Position Error Norm ---
        ax6 = plt.subplot(3, 2, 6)
        pos_error_norm = np.linalg.norm(pos_error, axis=1)
        plt.plot(time_err, pos_error_norm, label="|Δp|", linewidth=2, color="red")
        plt.ylabel("Position Error Norm (m)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha)
        plt.title("Total Position Error")
        plt.xlim([0, t_total_sim])
        plt.grid(True)

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    print("Pinocchio visualization module loaded successfully")
    print("Usage: Import and use PinocchioVisualizer class with simulation data")
