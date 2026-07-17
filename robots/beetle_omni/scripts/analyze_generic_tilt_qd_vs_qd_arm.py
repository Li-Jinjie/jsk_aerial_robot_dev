#!/usr/bin/env python3
"""
Compare an idealized tilt-quadrotor and a conventional quadrotor with a 2-DoF arm.

The tilt-quadrotor model used here is deliberately generic:
  - vehicle CoG is at the body/world origin for the nominal comparison,
  - four rotors are placed in an X layout with radius 0.275 m,
  - rotor z offsets are zero,
  - each rotor has two allocatable components: tangential and vertical thrust.

The qd+arm model reuses the static allocation model in
analyze_qd_arm_wrench_allocation.py with the same 0.275 m rotor/arm lengths.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import cvxpy as cp
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np
import scienceplots  # noqa: F401

from analyze_qd_arm_wrench_allocation import (
    QuadrotorArmParams,
    compute_allocation,
    sweep_force_torque_bounds,
)

from utils import matlab_blue, matlab_orange, matlab_yellow, matlab_purple

# MATLAB R2014b default color order
_MC = [matlab_blue, matlab_orange, matlab_yellow, matlab_purple]


@dataclass(frozen=True)
class GenericParams:
    rotor_arm_length_m: float = 0.275
    mass_kg: float = 3.039
    gravity_m_s2: float = 9.81
    max_thrust_per_rotor_N: float = 23.0
    yaw_moment_coeff_m: float = 0.0165

    @property
    def weight_N(self) -> float:
        return self.mass_kg * self.gravity_m_s2


def generic_tilt_qd_allocation_matrix(params: GenericParams) -> np.ndarray:
    """Build a generic 6x8 tilt-quadrotor allocation matrix.

    Decision vector:
        x = [ft1, fz1, ft2, fz2, ft3, fz3, ft4, fz4]^T

    where fti is the tangential horizontal thrust component of rotor i and fzi
    is its vertical thrust component.  The per-rotor thrust magnitude is
    sqrt(fti^2 + fzi^2).

    Rotor order matches analyze_qd_arm_wrench_allocation.py:
        1 front-left, 2 rear-left, 3 rear-right, 4 front-right.
    """

    L = params.rotor_arm_length_m
    a = L / math.sqrt(2.0)
    positions = np.array(
        [
            [a, a, 0.0],
            [-a, a, 0.0],
            [-a, -a, 0.0],
            [a, -a, 0.0],
        ],
        dtype=float,
    )

    # This sign choice makes the vertical-thrust yaw row identical to the
    # conventional quadrotor matrix: [+gamma, -gamma, +gamma, -gamma].
    drag_signs = np.array([-1.0, 1.0, -1.0, 1.0], dtype=float)

    A = np.zeros((6, 8), dtype=float)
    gamma = params.yaw_moment_coeff_m

    for i, (p_b, dr) in enumerate(zip(positions, drag_signs)):
        x_b, y_b, z_b = p_b
        r_xy = math.hypot(x_b, y_b)
        col_t = 2 * i
        col_z = col_t + 1

        A[0, col_t] = y_b / r_xy
        A[1, col_t] = -x_b / r_xy
        A[2, col_z] = 1.0

        A[3, col_t] = -dr * gamma * y_b / r_xy + x_b * z_b / r_xy
        A[4, col_t] = dr * gamma * x_b / r_xy + y_b * z_b / r_xy
        A[5, col_t] = -(x_b * x_b + y_b * y_b) / r_xy

        A[3, col_z] = y_b
        A[4, col_z] = -x_b
        A[5, col_z] = -dr * gamma

    return A


def rot_y_world_from_body(theta_rad: float) -> np.ndarray:
    """Rotation matrix from body to world for the pitch convention used here."""

    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )


def solve_tilt_min_norm(
    alloc_mtx: np.ndarray,
    wrench: np.ndarray,
    fmax: float,
    tol: float = 1e-7,
) -> tuple[np.ndarray | None, bool]:
    """Return a minimum-norm tilt-qd allocation for an exact target wrench."""

    x = cp.Variable(8)
    constraints = [alloc_mtx @ x == wrench]
    for i in range(4):
        constraints.append(cp.norm(x[2 * i : 2 * i + 2], 2) <= fmax)

    prob = cp.Problem(cp.Minimize(cp.sum_squares(x)), constraints)
    try:
        prob.solve(verbose=False)
    except cp.SolverError:
        return None, False

    if prob.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) or x.value is None:
        return None, False

    err = np.linalg.norm(alloc_mtx @ x.value - wrench)
    return x.value, bool(err <= tol)


def tilt_rotor_magnitudes(x: np.ndarray | None) -> np.ndarray:
    if x is None:
        return np.full(4, np.nan)
    return np.array([math.hypot(x[2 * i], x[2 * i + 1]) for i in range(4)], dtype=float)


def solve_tilt_tau_bound(
    alloc_mtx: np.ndarray,
    force_x_N: float,
    params: GenericParams,
    sign: float,
    install_angle_deg: float,
) -> float:
    """Maximize positive or negative world x torque for a fixed world x force.

    This follows analyze_wrench_coupled.py's convention.  For install_angle=90 deg,
    the searched body-frame torque axis is body z, and with pitch=90 deg that
    body z axis is aligned with world x.
    """

    install_angle_rad = math.radians(install_angle_deg)
    rot_wb = rot_y_world_from_body(install_angle_rad)
    rot_bw = rot_wb.T
    target_force_b = rot_bw @ np.array([force_x_N, 0.0, params.weight_N], dtype=float)
    torque_axis_b = np.array([math.cos(install_angle_rad), 0.0, math.sin(install_angle_rad)], dtype=float)

    x = cp.Variable(8)
    wrench = alloc_mtx @ x
    tau_b = wrench[3:6]
    tau_axis = torque_axis_b @ tau_b
    constraints = [
        wrench[0:3] == target_force_b,
        tau_b == tau_axis * torque_axis_b,
    ]
    for i in range(4):
        constraints.append(cp.norm(x[2 * i : 2 * i + 2], 2) <= params.max_thrust_per_rotor_N)

    objective = cp.Maximize(sign * tau_axis)
    prob = cp.Problem(objective, constraints)
    try:
        prob.solve(verbose=False)
    except cp.SolverError:
        return np.nan

    if prob.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) or x.value is None:
        return np.nan
    tau_b_value = (alloc_mtx @ x.value)[3:6]
    return float(torque_axis_b @ tau_b_value)


def compute_force_sweep(
    force_values: np.ndarray,
    params: GenericParams,
    qd_params: QuadrotorArmParams,
    install_angle_deg: float,
):
    tilt_A = generic_tilt_qd_allocation_matrix(params)
    rot_bw = rot_y_world_from_body(math.radians(install_angle_deg)).T

    qd_pitch_deg = []
    qd_tau_y_abs = []
    qd_rotors = []
    tilt_pitch_deg = []
    tilt_servo_tau = []
    tilt_rotors = []

    for force in force_values:
        qd_result = compute_allocation(float(force), 0.0, qd_params)
        if qd_result.feasible and qd_result.rotor_thrusts_N is not None:
            qd_pitch_deg.append(qd_result.pitch_angle_deg)
            qd_tau_y_abs.append(abs(qd_result.tau_prop_W_Nm[1]))
            qd_rotors.append(qd_result.rotor_thrusts_N)
        else:
            qd_pitch_deg.append(np.nan)
            qd_tau_y_abs.append(np.nan)
            qd_rotors.append(np.full(4, np.nan))

        # The idealized tilt-qd keeps level attitude and produces horizontal
        # force directly through rotor tilting.  The response plot uses the
        # same install-angle convention as analyze_wrench_coupled.py:
        # world target force [Fx, 0, mg] is rotated into the body frame at
        # pitch=install_angle_deg.
        tilt_force_b = rot_bw @ np.array([force, 0.0, params.weight_N], dtype=float)
        tilt_wrench = np.array([tilt_force_b[0], tilt_force_b[1], tilt_force_b[2], 0.0, 0.0, 0.0], dtype=float)
        tilt_x, tilt_ok = solve_tilt_min_norm(tilt_A, tilt_wrench, params.max_thrust_per_rotor_N)
        tilt_f = tilt_rotor_magnitudes(tilt_x)
        if tilt_ok and np.nanmax(tilt_f) <= params.max_thrust_per_rotor_N + 1e-6:
            tilt_pitch_deg.append(install_angle_deg)
            tilt_servo_tau.append(0.0)
            tilt_rotors.append(tilt_f)
        else:
            tilt_pitch_deg.append(np.nan)
            tilt_servo_tau.append(np.nan)
            tilt_rotors.append(np.full(4, np.nan))

    return {
        "force": force_values,
        "tilt_A": tilt_A,
        "qd_pitch_deg": np.asarray(qd_pitch_deg),
        "qd_tau_y_abs": np.asarray(qd_tau_y_abs),
        "qd_rotors": np.asarray(qd_rotors),
        "tilt_pitch_deg": np.asarray(tilt_pitch_deg),
        "tilt_servo_tau": np.asarray(tilt_servo_tau),
        "tilt_rotors": np.asarray(tilt_rotors),
    }


def plot_force_response(data: dict[str, np.ndarray], output_path: Path, fmax: float = 23.0) -> None:
    plt.style.use(["science", "grid"])
    plt.rcParams.update(
        {
            "font.size": 14,
            "axes.titlesize": 14,
            "legend.fontsize": 14,
        }
    )

    force = data["force"]

    # Determine feasible force range for each configuration
    qd_feasible = ~np.isnan(data["qd_pitch_deg"])
    tilt_feasible = ~np.isnan(data["tilt_pitch_deg"])
    qd_force_max = float(force[qd_feasible].max()) if qd_feasible.any() else float(force[0])
    tilt_force_max = float(force[tilt_feasible].max()) if tilt_feasible.any() else float(force[0])

    fig, axes = plt.subplots(3, 1, figsize=(8.0, 8.2), sharex=True)

    # Background shading: available force range for each configuration (all subplots)
    for ax in axes:
        ax.axvspan(force[0], qd_force_max, alpha=0.10, color=_MC[0], zorder=0)
        ax.axvspan(force[0], tilt_force_max, alpha=0.10, color=_MC[1], zorder=0)

    # --- Plot 1: attitude change relative to initial (force=0) ---
    qd_pitch_0 = data["qd_pitch_deg"][0]
    tilt_pitch_0 = data["tilt_pitch_deg"][0]
    if np.isnan(qd_pitch_0):
        idx = np.where(qd_feasible)[0]
        qd_pitch_0 = data["qd_pitch_deg"][idx[0]] if len(idx) else 0.0
    if np.isnan(tilt_pitch_0):
        idx = np.where(tilt_feasible)[0]
        tilt_pitch_0 = data["tilt_pitch_deg"][idx[0]] if len(idx) else 0.0

    qd_pitch_change = data["qd_pitch_deg"] - qd_pitch_0
    tilt_pitch_change = data["tilt_pitch_deg"] - tilt_pitch_0

    (qd_line,) = axes[0].plot(force, qd_pitch_change, color=_MC[0], label="qd+arm", linewidth=2.0)
    (tilt_line,) = axes[0].plot(force, tilt_pitch_change, color=_MC[1], label="tilt-qd", linewidth=2.0)
    qd_patch = Patch(color=_MC[0], alpha=0.1, label=f"qd+arm avail. range ($\\leq${qd_force_max:.0f} N)")
    tilt_patch = Patch(color=_MC[1], alpha=0.1, label=f"tilt-qd avail. range ($\\leq${tilt_force_max:.0f} N)")
    axes[0].set_ylabel("Attitude change [deg]")
    axes[0].legend(handles=[qd_line, tilt_line, qd_patch, tilt_patch], fontsize=13, framealpha=0.9)

    # --- Plot 2: servo torque ---
    axes[1].plot(force, data["qd_tau_y_abs"], color=_MC[0], label=r"qd+arm $q_1$ joint $|\tau_{q_1}|$", linewidth=2.0)
    axes[1].plot(force, data["tilt_servo_tau"], color=_MC[1], label="tilt-qd servo torque", linewidth=2.0)
    axes[1].set_ylabel("Max servo torque [N m]")
    axes[1].legend(framealpha=0.9, fontsize=13)

    # --- Plot 3: rotor thrusts ---
    # Rotors 1 & 2 drawn thicker so they are visible beneath rotors 3 & 4
    colors = _MC
    rotor_lws = [3.0, 3.0, 1.8, 1.8]
    for i, (color, lw) in enumerate(zip(colors, rotor_lws)):
        axes[2].plot(
            force,
            data["qd_rotors"][:, i],
            color=color,
            linestyle="-",
            linewidth=lw,
            label=f"qd+arm rotor {i + 1}",
        )
        axes[2].plot(
            force,
            data["tilt_rotors"][:, i],
            color=color,
            linestyle="--",
            linewidth=lw,
            label=f"tilt-qd rotor {i + 1}",
        )
    axes[2].axhline(fmax, color="0.3", linewidth=1.5, linestyle=":", label=f"Rotor limit ({fmax:.0f} N)")
    axes[2].set_xlabel("Applied force $f_n$ [N]")
    axes[2].set_ylabel("Rotor thrust [N]")
    axes[2].legend(ncol=2, fontsize=13, framealpha=0.6)

    for ax in axes:
        ax.set_xlim(force[0], force[-1])

    fig.tight_layout()
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_coupled_wrench_envelope(
    force_values: np.ndarray,
    params: GenericParams,
    qd_params: QuadrotorArmParams,
    output_path: Path,
    install_angle_deg: float,
) -> None:
    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 14})

    qd_rows = sweep_force_torque_bounds(force_values, qd_params)
    qd_tau_pos = np.array(
        [np.nan if row["tau_link_max_positive_Nm"] is None else row["tau_link_max_positive_Nm"] for row in qd_rows],
        dtype=float,
    )
    qd_tau_neg = np.array(
        [np.nan if row["tau_link_max_negative_Nm"] is None else row["tau_link_max_negative_Nm"] for row in qd_rows],
        dtype=float,
    )

    tilt_A = generic_tilt_qd_allocation_matrix(params)
    tilt_tau_pos = np.array(
        [
            solve_tilt_tau_bound(tilt_A, float(force), params, sign=+1.0, install_angle_deg=install_angle_deg)
            for force in force_values
        ],
        dtype=float,
    )
    tilt_tau_neg = np.array(
        [
            solve_tilt_tau_bound(tilt_A, float(force), params, sign=-1.0, install_angle_deg=install_angle_deg)
            for force in force_values
        ],
        dtype=float,
    )

    fig, ax = plt.subplots(figsize=(8.0, 4.6))
    ax.plot(force_values, qd_tau_pos, color=_MC[0], linewidth=2.0, label=r"qd+arm $+\tau_n$")
    ax.plot(force_values, qd_tau_neg, color=_MC[0], linewidth=2.0, linestyle="--", label=r"qd+arm $-\tau_n$")
    ax.plot(force_values, tilt_tau_pos, color=_MC[1], linewidth=2.0, label=r"tilt-qd $+\tau_n$")
    ax.plot(force_values, tilt_tau_neg, color=_MC[1], linewidth=2.0, linestyle="--", label=r"tilt-qd $-\tau_n$")
    ax.fill_between(force_values, qd_tau_neg, qd_tau_pos, color=_MC[0], alpha=0.10)
    ax.fill_between(force_values, tilt_tau_neg, tilt_tau_pos, color=_MC[1], alpha=0.10)
    ax.axhline(0.0, color="0.3", linewidth=0.8)
    ax.set_xlabel("Required horizontal force $f_n$ [N]")
    ax.set_ylabel(r"Feasible torque envelope $\tau_n$ [N m]")
    ax.set_xlim(force_values[0], force_values[-1])
    ax.legend(ncol=2, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generic tilt-qd vs qd+arm wrench comparison.")
    parser.add_argument("--force-min", type=float, default=0.0, help="Minimum horizontal force [N].")
    parser.add_argument("--force-max", type=float, default=85.0, help="Maximum horizontal force [N].")
    parser.add_argument("--num-force", type=int, default=86, help="Number of force samples.")
    parser.add_argument("--mass", type=float, default=3.039, help="Vehicle mass [kg].")
    parser.add_argument("--g", type=float, default=9.81, help="Gravity [m/s^2].")
    parser.add_argument("--length", type=float, default=0.275, help="Rotor radius and arm link length [m].")
    parser.add_argument("--fmax", type=float, default=23.0, help="Maximum thrust per rotor [N].")
    parser.add_argument("--gamma", type=float, default=0.0165, help="Yaw moment per thrust [m].")
    parser.add_argument(
        "--install-angle",
        type=float,
        default=90.0,
        help="Tilt-qd install angle for coupled wrench envelope [deg].",
    )
    parser.add_argument("--response-plot", type=Path, default=Path("generic_tilt_qd_vs_qd_arm_response.pdf"))
    parser.add_argument("--envelope-plot", type=Path, default=Path("generic_tilt_qd_vs_qd_arm_envelope.pdf"))
    args = parser.parse_args()

    params = GenericParams(
        rotor_arm_length_m=args.length,
        mass_kg=args.mass,
        gravity_m_s2=args.g,
        max_thrust_per_rotor_N=args.fmax,
        yaw_moment_coeff_m=args.gamma,
    )
    qd_params = QuadrotorArmParams(
        rotor_arm_length_m=args.length,
        arm_link1_length_m=args.length,
        arm_link2_length_m=args.length,
        mass_kg=args.mass,
        gravity_m_s2=args.g,
        max_thrust_per_rotor_N=args.fmax,
        yaw_moment_coeff_m=args.gamma,
    )

    force_values = np.linspace(args.force_min, args.force_max, args.num_force)
    data = compute_force_sweep(force_values, params, qd_params, args.install_angle)

    np.set_printoptions(precision=5, suppress=True)
    print("Generic tilt-qd allocation matrix:")
    print(data["tilt_A"])

    plot_force_response(data, args.response_plot, fmax=params.max_thrust_per_rotor_N)
    plot_coupled_wrench_envelope(force_values, params, qd_params, args.envelope_plot, args.install_angle)
    print(f"Saved response plot: {args.response_plot}")
    print(f"Saved coupled wrench envelope plot: {args.envelope_plot}")


if __name__ == "__main__":
    main()
