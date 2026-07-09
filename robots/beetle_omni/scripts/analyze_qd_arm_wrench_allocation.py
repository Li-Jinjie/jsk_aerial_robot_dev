#!/usr/bin/env python3
"""
Static wrench/allocation analysis for a conventional quadrotor with a 2-DoF arm.

Problem setting
---------------
The vehicle is a fixed-rotor conventional quadrotor. The first arm link is kept
vertical downward and the second link is kept horizontal forward in the world
frame. The end-effector is therefore located at

    r_ee^W = [L2, 0, -L1]^T

relative to the vehicle center of mass. The end-effector applies to the wall

    f_app^W   = [F_link, 0, 0]^T,       force along the second link
    tau_app^W = [tau_link, 0, 0]^T,     torque about the second link

where the world x-axis is the second-link direction and the world z-axis is
upward. The environment applies the opposite wrench to the robot.

The script computes
1. the required body pitch angle for force equilibrium,
2. the required propeller moment for moment equilibrium,
3. the four rotor thrusts from a 4-by-4 control allocation matrix,
4. feasibility with individual rotor thrust limits, and
5. optionally, the maximum feasible torque about the second link for each
   prescribed force along the second link.

Important sign convention
-------------------------
A positive F_link means that the robot pushes the wall along +W x. The wall
reaction on the robot is therefore -W x. A positive tau_link means that the
robot applies a positive torque about +W x to the wall. The wall applies the
opposite torque to the robot.

Rotor layout and allocation matrix
----------------------------------
An X-type quadrotor is assumed. L is the rotor distance from the vehicle center.
The projected x/y offsets are a = L/sqrt(2):

    rotor 1: front-left,  position [ a,  a, 0]^T
    rotor 2: rear-left,   position [-a,  a, 0]^T
    rotor 3: rear-right,  position [-a, -a, 0]^T
    rotor 4: front-right, position [ a, -a, 0]^T

All rotor thrusts are along +B z. With f = [f1, f2, f3, f4]^T, the body-frame
control input is

    u = [T, tau_x, tau_y, tau_z]^T = B f,

where

    B = [[1,       1,       1,       1      ],
         [a,       a,      -a,      -a      ],
         [-a,      a,       a,      -a      ],
         [gamma,  -gamma,  gamma,  -gamma  ]].

The first three rows follow tau = r x f_z, i.e., tau_x = y f and
tau_y = -x f. Here gamma is the rotor drag moment coefficient divided by
thrust coefficient, with unit meter.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class QuadrotorArmParams:
    """Physical parameters for the static analysis."""

    rotor_arm_length_m: float = 0.275
    arm_link1_length_m: float = 0.275
    arm_link2_length_m: float = 0.275
    mass_kg: float = 3.039
    gravity_m_s2: float = 9.81
    max_thrust_per_rotor_N: float = 23.0
    yaw_moment_coeff_m: float = 0.0165  # gamma = yaw moment / thrust, platform-dependent

    @property
    def weight_N(self) -> float:
        return self.mass_kg * self.gravity_m_s2

    @property
    def max_total_thrust_N(self) -> float:
        return 4.0 * self.max_thrust_per_rotor_N


@dataclass(frozen=True)
class AllocationResult:
    """Result for one prescribed end-effector force and torque."""

    force_link_N: float
    torque_link_Nm: float
    pitch_angle_rad: float
    pitch_angle_deg: float
    total_thrust_N: float
    tau_prop_W_Nm: np.ndarray
    tau_prop_B_Nm: np.ndarray
    allocation_matrix: np.ndarray
    rotor_thrusts_N: np.ndarray | None
    feasible: bool
    feasibility_reason: str

    @property
    def max_rotor_thrust_N(self) -> float | None:
        if self.rotor_thrusts_N is None:
            return None
        return float(np.max(self.rotor_thrusts_N))

    @property
    def min_rotor_thrust_N(self) -> float | None:
        if self.rotor_thrusts_N is None:
            return None
        return float(np.min(self.rotor_thrusts_N))


def rot_y(theta_rad: float) -> np.ndarray:
    """Rotation matrix R_WB for a positive pitch that tilts +Bz toward +Wx."""

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


def allocation_matrix(params: QuadrotorArmParams) -> np.ndarray:
    """Return the X-configuration quadrotor allocation matrix B.

    Rotor order:
        1: front-left,  r1 = [ a,  a, 0]^T
        2: rear-left,   r2 = [-a,  a, 0]^T
        3: rear-right,  r3 = [-a, -a, 0]^T
        4: front-right, r4 = [ a, -a, 0]^T

    For vertical thrust along +Bz, r_i x [0,0,f_i]^T gives
        tau_x = y_i f_i,
        tau_y = -x_i f_i.

    The yaw row assumes alternating rotor spin directions:
        [+gamma, -gamma, +gamma, -gamma].
    Change this row if your motor numbering or spin convention differs.
    """

    L = params.rotor_arm_length_m
    a = L / math.sqrt(2.0)
    gamma = params.yaw_moment_coeff_m
    return np.array(
        [
            [1.0, 1.0, 1.0, 1.0],
            [a, a, -a, -a],
            [-a, a, a, -a],
            [gamma, -gamma, gamma, -gamma],
        ],
        dtype=float,
    )


def required_static_wrench(
    force_link_N: float,
    torque_link_Nm: float,
    params: QuadrotorArmParams,
) -> tuple[float, float, np.ndarray, np.ndarray]:
    """Compute required pitch, total thrust, and propeller moment.

    Returns
    -------
    theta_rad:
        Required pitch angle satisfying force equilibrium.
    total_thrust_N:
        Required total thrust magnitude.
    tau_prop_W:
        Required propeller moment expressed in the world frame.
    tau_prop_B:
        Required propeller moment expressed in the body frame.

    Derivation
    ----------
    The robot applies f_app = [F,0,0] and tau_app = [tau,0,0] to the wall.
    The wall applies f_env = -f_app and tau_env = -tau_app to the robot.

    Moment from wall about the vehicle center is

        tau_ext^W = r_ee^W x f_env^W + tau_env^W
                  = [-tau, L1*F, 0]^T.

    Static moment equilibrium requires

        tau_prop^W + tau_ext^W = 0,

    hence

        tau_prop^W = [tau, -L1*F, 0]^T.
    """

    W = params.weight_N
    theta_rad = math.atan2(force_link_N, W)
    total_thrust_N = math.hypot(W, force_link_N)

    tau_prop_W = np.array(
        [
            torque_link_Nm,
            -params.arm_link1_length_m * force_link_N,
            0.0,
        ],
        dtype=float,
    )

    R_WB = rot_y(theta_rad)
    tau_prop_B = R_WB.T @ tau_prop_W
    return theta_rad, total_thrust_N, tau_prop_W, tau_prop_B


def compute_allocation(
    force_link_N: float,
    torque_link_Nm: float,
    params: QuadrotorArmParams = QuadrotorArmParams(),
    tol: float = 1e-9,
) -> AllocationResult:
    """Compute rotor thrusts for a requested force and torque at the end-effector."""

    B = allocation_matrix(params)
    theta, total_thrust, tau_prop_W, tau_prop_B = required_static_wrench(force_link_N, torque_link_Nm, params)

    u = np.array([total_thrust, tau_prop_B[0], tau_prop_B[1], tau_prop_B[2]], dtype=float)

    if total_thrust > params.max_total_thrust_N + tol:
        return AllocationResult(
            force_link_N=force_link_N,
            torque_link_Nm=torque_link_Nm,
            pitch_angle_rad=theta,
            pitch_angle_deg=math.degrees(theta),
            total_thrust_N=total_thrust,
            tau_prop_W_Nm=tau_prop_W,
            tau_prop_B_Nm=tau_prop_B,
            allocation_matrix=B,
            rotor_thrusts_N=None,
            feasible=False,
            feasibility_reason=(
                f"total thrust {total_thrust:.3f} N exceeds limit " f"{params.max_total_thrust_N:.3f} N"
            ),
        )

    try:
        f_rotors = np.linalg.solve(B, u)
    except np.linalg.LinAlgError:
        return AllocationResult(
            force_link_N=force_link_N,
            torque_link_Nm=torque_link_Nm,
            pitch_angle_rad=theta,
            pitch_angle_deg=math.degrees(theta),
            total_thrust_N=total_thrust,
            tau_prop_W_Nm=tau_prop_W,
            tau_prop_B_Nm=tau_prop_B,
            allocation_matrix=B,
            rotor_thrusts_N=None,
            feasible=False,
            feasibility_reason="allocation matrix is singular; check yaw_moment_coeff_m",
        )

    min_f = float(np.min(f_rotors))
    max_f = float(np.max(f_rotors))
    feasible = min_f >= -tol and max_f <= params.max_thrust_per_rotor_N + tol
    if feasible:
        reason = "feasible"
    elif min_f < -tol and max_f > params.max_thrust_per_rotor_N + tol:
        reason = f"negative thrust and saturation: min={min_f:.3f} N, " f"max={max_f:.3f} N"
    elif min_f < -tol:
        reason = f"negative thrust required: min={min_f:.3f} N"
    else:
        reason = f"rotor saturation: max={max_f:.3f} N exceeds " f"{params.max_thrust_per_rotor_N:.3f} N"

    return AllocationResult(
        force_link_N=force_link_N,
        torque_link_Nm=torque_link_Nm,
        pitch_angle_rad=theta,
        pitch_angle_deg=math.degrees(theta),
        total_thrust_N=total_thrust,
        tau_prop_W_Nm=tau_prop_W,
        tau_prop_B_Nm=tau_prop_B,
        allocation_matrix=B,
        rotor_thrusts_N=f_rotors,
        feasible=feasible,
        feasibility_reason=reason,
    )


def is_feasible(force_link_N: float, torque_link_Nm: float, params: QuadrotorArmParams) -> bool:
    return compute_allocation(force_link_N, torque_link_Nm, params).feasible


def find_torque_bound(
    force_link_N: float,
    params: QuadrotorArmParams,
    sign: float,
    initial_hi: float = 0.1,
    max_hi: float = 200.0,
    iterations: int = 80,
) -> float | None:
    """Find the positive magnitude bound for torque with a fixed sign.

    Returns None if zero torque is infeasible for the given force.
    """

    if sign == 0:
        raise ValueError("sign must be nonzero")
    sign = 1.0 if sign > 0 else -1.0

    if not is_feasible(force_link_N, 0.0, params):
        return None

    lo = 0.0
    hi = initial_hi
    while hi < max_hi and is_feasible(force_link_N, sign * hi, params):
        lo = hi
        hi *= 2.0

    if hi >= max_hi and is_feasible(force_link_N, sign * hi, params):
        return max_hi

    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        if is_feasible(force_link_N, sign * mid, params):
            lo = mid
        else:
            hi = mid
    return lo


def sweep_force_torque_bounds(
    force_values: Iterable[float],
    params: QuadrotorArmParams,
) -> list[dict[str, float | None]]:
    """Compute positive/negative torque bounds for each force."""

    rows: list[dict[str, float | None]] = []
    for F in force_values:
        r0 = compute_allocation(F, 0.0, params)
        tau_pos = find_torque_bound(F, params, sign=+1.0)
        tau_neg_mag = find_torque_bound(F, params, sign=-1.0)
        rows.append(
            {
                "force_link_N": F,
                "pitch_angle_deg": r0.pitch_angle_deg,
                "total_thrust_N": r0.total_thrust_N,
                "zero_torque_feasible": 1.0 if r0.feasible else 0.0,
                "tau_link_max_positive_Nm": tau_pos,
                "tau_link_max_negative_Nm": None if tau_neg_mag is None else -tau_neg_mag,
            }
        )
    return rows


def print_single_result(result: AllocationResult) -> None:
    """Print a detailed result for one force/torque pair."""

    np.set_printoptions(precision=6, suppress=True)
    print("Requested end-effector wrench applied to the wall:")
    print(f"  force along second link F_link = {result.force_link_N:.6g} N")
    print(f"  torque about second link tau_link = {result.torque_link_Nm:.6g} Nm")
    print()
    print("Static equilibrium requirement:")
    print(f"  pitch angle theta = {result.pitch_angle_deg:.6f} deg")
    print(f"  total thrust T = {result.total_thrust_N:.6f} N")
    print(f"  required propeller moment in W frame [tau_x, tau_y, tau_z] = {result.tau_prop_W_Nm} Nm")
    print(f"  required propeller moment in B frame [tau_x, tau_y, tau_z] = {result.tau_prop_B_Nm} Nm")
    print()
    print("Control allocation matrix B, with u=[T,tau_x,tau_y,tau_z]^T = B f:")
    print(result.allocation_matrix)
    print()
    if result.rotor_thrusts_N is not None:
        print("Rotor thrusts [front-left, rear-left, rear-right, front-right] in N:")
        print(result.rotor_thrusts_N)
        print(f"  min rotor thrust = {result.min_rotor_thrust_N:.6f} N")
        print(f"  max rotor thrust = {result.max_rotor_thrust_N:.6f} N")
    else:
        print("Rotor thrusts: not available")
    print(f"Feasible: {result.feasible} ({result.feasibility_reason})")


def print_sweep(rows: list[dict[str, float | None]]) -> None:
    header = (
        "F_link[N]",
        "pitch[deg]",
        "T[N]",
        "tau_min[Nm]",
        "tau_max[Nm]",
        "zero_tau_ok",
    )
    print(
        f"{header[0]:>12s}  {header[1]:>12s}  {header[2]:>10s}  "
        f"{header[3]:>12s}  {header[4]:>12s}  {header[5]:>11s}"
    )
    print("-" * 82)
    for row in rows:
        tau_min = row["tau_link_max_negative_Nm"]
        tau_max = row["tau_link_max_positive_Nm"]
        tau_min_s = "--" if tau_min is None else f"{tau_min:.4f}"
        tau_max_s = "--" if tau_max is None else f"{tau_max:.4f}"
        print(
            f"{row['force_link_N']:12.4f}  {row['pitch_angle_deg']:12.4f}  "
            f"{row['total_thrust_N']:10.4f}  {tau_min_s:>12s}  "
            f"{tau_max_s:>12s}  {int(row['zero_torque_feasible']):11d}"
        )


def save_sweep_csv(rows: list[dict[str, float | None]], path: Path) -> None:
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def save_sweep_plot(rows: list[dict[str, float | None]], path: Path) -> None:
    import matplotlib.pyplot as plt

    F = np.array([float(r["force_link_N"]) for r in rows])
    tau_pos = np.array(
        [np.nan if r["tau_link_max_positive_Nm"] is None else float(r["tau_link_max_positive_Nm"]) for r in rows]
    )
    tau_neg = np.array(
        [np.nan if r["tau_link_max_negative_Nm"] is None else float(r["tau_link_max_negative_Nm"]) for r in rows]
    )

    plt.figure(figsize=(5.0, 3.5))
    plt.plot(F, tau_pos, label="positive torque bound")
    plt.plot(F, tau_neg, label="negative torque bound")
    plt.axhline(0.0, linewidth=0.8)
    plt.xlabel("Force along the second link [N]")
    plt.ylabel("Feasible torque about the second link [Nm]")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=300)
    plt.close()


def make_params_from_args(args: argparse.Namespace) -> QuadrotorArmParams:
    return QuadrotorArmParams(
        rotor_arm_length_m=args.rotor_length,
        arm_link1_length_m=args.link1_length,
        arm_link2_length_m=args.link2_length,
        mass_kg=args.mass,
        gravity_m_s2=args.g,
        max_thrust_per_rotor_N=args.fmax,
        yaw_moment_coeff_m=args.gamma,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Compute rotor thrust allocation for a conventional quadrotor with "
            "a 2-DoF arm applying force and torque along/about the second link."
        )
    )
    parser.add_argument("--force", type=float, default=20.0, help="Force along second link in N.")
    parser.add_argument("--torque", type=float, default=0.0, help="Torque about second link in Nm.")
    parser.add_argument("--mass", type=float, default=3.039, help="Total mass in kg.")
    parser.add_argument("--g", type=float, default=9.81, help="Gravity in m/s^2.")
    parser.add_argument("--rotor-length", type=float, default=0.275, help="Rotor distance from center in m.")
    parser.add_argument("--link1-length", type=float, default=0.275, help="First arm link length in m.")
    parser.add_argument("--link2-length", type=float, default=0.275, help="Second arm link length in m.")
    parser.add_argument("--fmax", type=float, default=23.0, help="Maximum thrust per rotor in N.")
    parser.add_argument(
        "--gamma",
        type=float,
        default=0.0165,
        help=("Yaw moment coefficient in meter, i.e., yaw moment per thrust. " "Default: 0.0165 m."),
    )
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="Sweep force values and compute feasible torque bounds.",
    )
    parser.add_argument("--force-min", type=float, default=0.0, help="Minimum force for sweep in N.")
    parser.add_argument("--force-max", type=float, default=40.0, help="Maximum force for sweep in N.")
    parser.add_argument("--num-force", type=int, default=41, help="Number of force samples in sweep.")
    parser.add_argument("--csv", type=Path, default=None, help="Optional CSV path for sweep results.")
    parser.add_argument("--plot", type=Path, default=None, help="Optional PNG/PDF path for sweep plot.")

    args = parser.parse_args()
    params = make_params_from_args(args)

    print("Parameters:")
    print(f"  mass = {params.mass_kg:.6g} kg")
    print(f"  g = {params.gravity_m_s2:.6g} m/s^2")
    print(f"  weight = {params.weight_N:.6g} N")
    print(f"  rotor arm length = {params.rotor_arm_length_m:.6g} m")
    print(f"  arm link lengths = ({params.arm_link1_length_m:.6g}, {params.arm_link2_length_m:.6g}) m")
    print(f"  max thrust per rotor = {params.max_thrust_per_rotor_N:.6g} N")
    print(f"  yaw moment coefficient gamma = {params.yaw_moment_coeff_m:.6g} m")
    print()

    if args.sweep:
        force_values = np.linspace(args.force_min, args.force_max, args.num_force)
        rows = sweep_force_torque_bounds(force_values, params)
        print_sweep(rows)
        if args.csv is not None:
            save_sweep_csv(rows, args.csv)
            print(f"\nSaved CSV to: {args.csv}")
        if args.plot is not None:
            save_sweep_plot(rows, args.plot)
            print(f"Saved plot to: {args.plot}")
    else:
        result = compute_allocation(args.force, args.torque, params)
        print_single_result(result)


if __name__ == "__main__":
    main()
