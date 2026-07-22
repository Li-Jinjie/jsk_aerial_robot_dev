import sys, os
from copy import deepcopy
import time
import numpy as np
import argparse
import csv
import json
import transformations as tf

from nmpc_tilt_mt.utils.nmpc_viz import Visualizer
from nmpc_tilt_mt.utils.step_response_experiment import (
    ALL_AXES as STEP_RESPONSE_AXES,
    ATTITUDE_AXES as STEP_ATTITUDE_AXES,
    POSITION_AXES as STEP_POSITION_AXES,
    SCENARIO_NAME as STEP_RESPONSE_SCENARIO,
    StepCase,
    compute_run_metrics,
    save_run_bundle,
)

# Quadrotor
import nmpc_tilt_mt.tilt_qd.phys_param_beetle_omni as phys_omni
import nmpc_tilt_mt.archive.phys_param_beetle_art as phys_art

# - Naive models
from nmpc_tilt_mt.archive.tilt_qd_no_servo_ac_cost import NMPCTiltQdNoServoAcCost
from nmpc_tilt_mt.tilt_qd.tilt_qd_no_servo import NMPCTiltQdNoServo

# - Consider the servo delay with its model
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo import NMPCTiltQdServo
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_dist import NMPCTiltQdServoDist
from nmpc_tilt_mt.archive.tilt_qd_servo_drag_w_dist import NMPCTiltQdServoDragDist
from nmpc_tilt_mt.archive.tilt_qd_servo_w_cog_end_dist import NMPCTiltQdServoWCogEndDist

# - Conside servo angle derivative as state
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_diff import NMPCTiltQdServoDiff

# - Consider the absolute servo angle command in cost
from nmpc_tilt_mt.archive.tilt_qd_servo_old_cost import NMPCTiltQdServoOldCost

# - Consider the thrust delay with its model
from nmpc_tilt_mt.tilt_qd.tilt_qd_thrust import NMPCTiltQdThrust

# - Consider the servo & thrust delay with its models
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_thrust import NMPCTiltQdServoThrust
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist
from nmpc_tilt_mt.archive.tilt_qd_servo_thrust_drag import NMPCTiltQdServoThrustDrag

# - Classic baseline: geometric controller + pseudo-inverse allocation
from nmpc_tilt_mt.tilt_qd.tilt_qd_geom_baseline import GeomControllerTiltQd

# Birotor
from nmpc_tilt_mt.tilt_bi.tilt_bi_servo import NMPCTiltBiServo
from nmpc_tilt_mt.tilt_bi.tilt_bi_2ord_servo import NMPCTiltBi2OrdServo

# Trirotor
from nmpc_tilt_mt.tilt_tri.tilt_tri_servo import NMPCTiltTriServo
from nmpc_tilt_mt.tilt_tri.tilt_tri_servo_dist import NMPCTiltTriServoDist


def set_servo_time_constant(phys, time_constant):
    """Override the servo time constant in both named and flattened physical parameters."""
    # physical_param_list order is base parameters(6), four times [dr, p_b(3)],
    # then t_rotor and t_servo.
    servo_param_idx = 6 + 4 * 4 + 1
    phys.t_servo = time_constant
    phys.physical_param_list[servo_param_idx] = time_constant


def get_constraint_sweep_target(args, t_now):
    """Return the pose target and active sweep segment for the constraint test."""
    target_xyz = np.zeros((3, 1))
    target_rpy = np.radians(np.asarray(args.test_attitude_deg, dtype=float)).reshape(3, 1)

    if t_now < args.warmup_duration:
        return target_xyz, target_rpy, -1

    segment = int((t_now - args.warmup_duration) / args.segment_duration)
    segment = min(segment, len(args.step_levels) - 1)
    direction = 1.0 if segment % 2 == 0 else -1.0
    axis = 0 if args.sweep_axis == "x" else 1
    target_xyz[axis, 0] = direction * args.step_levels[segment]
    return target_xyz, target_rpy, segment


def get_servo_delay_sweep_target(args, t_now):
    """Return the legacy target, optionally shifted after a hover warm-up."""
    onset_time = 0.0 if args.startup_mode == "cold" else args.startup_warmup_duration
    local_time = t_now - onset_time
    if local_time < 0.0:
        return np.zeros((3, 1)), np.zeros((3, 1)), -1

    target_xyz = np.array([[0.3, 0.6, 1.0]]).T
    target_rpy = np.zeros((3, 1))
    phase = 0
    if 2.0 <= local_time < 6.0:
        target_rpy = np.radians(np.array([[30.0, 60.0, 90.0]]).T)
        phase = 1
    elif local_time >= 6.0:
        target_xyz = np.array([[1.0, 1.0, 1.0]]).T
        phase = 2
    return target_xyz, target_rpy, phase


def get_step_response_target(args, t_now):
    """Return the workpoint pose with one commanded axis stepped at step_time."""
    target_xyz = np.zeros((3, 1))
    target_rpy = np.radians(np.asarray(args.workpoint_rpy_deg, dtype=float)).reshape(3, 1)
    if t_now >= args.step_time:
        if args.step_axis in STEP_POSITION_AXES:
            target_xyz[STEP_POSITION_AXES.index(args.step_axis), 0] = args.step_amplitude
        else:
            target_rpy[STEP_ATTITUDE_AXES.index(args.step_axis), 0] += np.radians(args.step_amplitude)
    return target_xyz, target_rpy, int(t_now >= args.step_time)


def apply_common_actuator_bounds(nmpc, ocp_solver, args, is_baseline):
    """Apply identical actuator limits to every controller in a comparison."""
    angle_max = np.radians(args.servo_angle_max_deg)

    if is_baseline:
        nmpc.thrust_min = 0.0
        nmpc.thrust_max = args.test_thrust_max
        nmpc.a_min = -angle_max
        nmpc.a_max = angle_max
        return

    # Input order is [four thrust commands, four servo-angle commands].
    lbu = np.concatenate((np.zeros(4), np.full(4, -angle_max)))
    ubu = np.concatenate((np.full(4, args.test_thrust_max), np.full(4, angle_max)))
    for stage in range(ocp_solver.N):
        ocp_solver.constraints_set(stage, "lbu", lbu)
        ocp_solver.constraints_set(stage, "ubu", ubu)

    # Constrained states start with v(3), w(3). Servo-aware models append the
    # four actual servo angles. Widen velocity bounds for the constraint task.
    lbx = np.asarray(ocp_solver.acados_ocp.constraints.lbx, dtype=float).copy()
    ubx = np.asarray(ocp_solver.acados_ocp.constraints.ubx, dtype=float).copy()
    if args.scenario == "constraint_sweep":
        lbx[:3] = -args.test_velocity_max
        ubx[:3] = args.test_velocity_max
    if nmpc.include_servo_model:
        lbx[-4:] = -angle_max
        ubx[-4:] = angle_max
    for stage in range(1, ocp_solver.N):
        ocp_solver.constraints_set(stage, "lbx", lbx)
        ocp_solver.constraints_set(stage, "ubx", ubx)

    lbx_e = np.asarray(ocp_solver.acados_ocp.constraints.lbx_e, dtype=float).copy()
    ubx_e = np.asarray(ocp_solver.acados_ocp.constraints.ubx_e, dtype=float).copy()
    if args.scenario == "constraint_sweep":
        lbx_e[:3] = -args.test_velocity_max
        ubx_e[:3] = args.test_velocity_max
    if nmpc.include_servo_model:
        lbx_e[-4:] = -angle_max
        ubx_e[-4:] = angle_max
    ocp_solver.constraints_set(ocp_solver.N, "lbx", lbx_e)
    ocp_solver.constraints_set(ocp_solver.N, "ubx", ubx_e)


def apply_constraint_sweep_bounds(nmpc, ocp_solver, args, is_baseline):
    """Backward-compatible wrapper for the constraint-active experiment."""
    apply_common_actuator_bounds(nmpc, ocp_solver, args, is_baseline)


def quaternion_geodesic_error(q, q_ref):
    """Quaternion attitude error in radians, invariant to quaternion sign."""
    q_norm = q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)
    qr_norm = q_ref / np.maximum(np.linalg.norm(q_ref, axis=1, keepdims=True), 1e-12)
    dot = np.clip(np.abs(np.sum(q_norm * qr_norm, axis=1)), 0.0, 1.0)
    return 2.0 * np.arccos(dot)


def bound_activity(values, lower, upper):
    """Return time-wise and element-wise fractions close to either bound."""
    tolerance = 0.01 * (upper - lower)
    active = (values <= lower + tolerance) | (values >= upper - tolerance)
    return float(np.mean(np.any(active, axis=1))), float(np.mean(active))


def compute_constraint_sweep_metrics(
    args,
    ts_sim,
    x_history,
    u_history,
    target_xyz_history,
    target_q_history,
    segment_history,
    solve_time_history,
    solver_failure_count,
):
    """Compute and print comparable tracking, constraint, and timing metrics."""
    x = np.asarray(x_history)
    u = np.asarray(u_history)
    xyz_ref = np.asarray(target_xyz_history)
    q_ref = np.asarray(target_q_history)
    segments = np.asarray(segment_history)
    angle_max = np.radians(args.servo_angle_max_deg)

    pos_error = x[:, :3] - xyz_ref
    horizontal_error = np.linalg.norm(pos_error[:, :2], axis=1)
    position_error = np.linalg.norm(pos_error, axis=1)
    attitude_error_deg = np.degrees(quaternion_geodesic_error(x[:, 6:10], q_ref))

    servo_actual = x[:, 13:17]
    thrust_actual = x[:, 17:21]
    servo_delta = (u[:, 4:8] - servo_actual + np.pi) % (2.0 * np.pi) - np.pi
    servo_rate_abs = np.abs(np.diff(servo_actual, axis=0) / ts_sim)
    # Align differentiated samples with the segment/time masks of their endpoints.

    def summarize(mask):
        thrust_any, thrust_samples = bound_activity(u[mask, :4], 0.0, args.test_thrust_max)
        servo_any, servo_samples = bound_activity(u[mask, 4:8], -angle_max, angle_max)
        rate_mask = mask[1:]
        selected_rate = servo_rate_abs[rate_mask]
        return {
            "position_rmse_m": float(np.sqrt(np.mean(position_error[mask] ** 2))),
            "horizontal_rmse_m": float(np.sqrt(np.mean(horizontal_error[mask] ** 2))),
            "position_max_m": float(np.max(position_error[mask])),
            "attitude_rmse_deg": float(np.sqrt(np.mean(attitude_error_deg[mask] ** 2))),
            "attitude_max_deg": float(np.max(attitude_error_deg[mask])),
            "thrust_active_time_pct": 100.0 * thrust_any,
            "thrust_active_samples_pct": 100.0 * thrust_samples,
            "thrust_cmd_min_n": float(np.min(u[mask, :4])),
            "thrust_cmd_max_n": float(np.max(u[mask, :4])),
            "thrust_actual_max_n": float(np.max(thrust_actual[mask])),
            "servo_active_time_pct": 100.0 * servo_any,
            "servo_active_samples_pct": 100.0 * servo_samples,
            "servo_cmd_abs_max_deg": float(np.degrees(np.max(np.abs(u[mask, 4:8])))),
            "servo_lag_rms_deg": float(np.degrees(np.sqrt(np.mean(servo_delta[mask] ** 2)))),
            "servo_rate_rms_deg_s": float(np.degrees(np.sqrt(np.mean(selected_rate**2)))),
            "servo_rate_p95_deg_s": float(np.degrees(np.percentile(selected_rate, 95))),
            "servo_rate_max_deg_s": float(np.degrees(np.max(selected_rate))),
        }

    evaluation_mask = segments >= 0
    overall = summarize(evaluation_mask)
    solve_times = np.asarray(solve_time_history)
    overall.update(
        {
            "solve_time_mean_ms": float(1e3 * np.mean(solve_times)),
            "solve_time_p95_ms": float(1e3 * np.percentile(solve_times, 95)),
            "solve_time_max_ms": float(1e3 * np.max(solve_times)),
            "solver_failures": int(solver_failure_count),
        }
    )

    per_segment = []
    for segment, level in enumerate(args.step_levels):
        mask = segments == segment
        if not np.any(mask):
            continue
        result = summarize(mask)
        result["segment"] = segment + 1
        result["target_m"] = (1.0 if segment % 2 == 0 else -1.0) * level
        per_segment.append(result)

    print("\n========== Constraint sweep configuration ==========")
    print(f"model={args.model}, axis={args.sweep_axis}, step_levels_m={args.step_levels}")
    print(
        f"attitude_deg={args.test_attitude_deg}, thrust_bounds_N=[0, {args.test_thrust_max:.3f}], "
        f"servo_bounds_deg=[-{args.servo_angle_max_deg:.3f}, {args.servo_angle_max_deg:.3f}], "
        f"servo_time_constant_s={args.effective_servo_time_constant:.6f}, "
        f"NMPC_velocity_bounds_m_s=+-{args.test_velocity_max:.3f}"
    )
    print("\n========== Per-segment metrics ==========")
    print(" seg  target[m]  pos_rmse[m]  att_rmse[deg]  thrust_active[%]  " "servo_active[%]  servo_rate_p95[deg/s]")
    for result in per_segment:
        print(
            f" {result['segment']:>3d}  {result['target_m']:>9.3f}  "
            f"{result['position_rmse_m']:>11.4f}  {result['attitude_rmse_deg']:>13.3f}  "
            f"{result['thrust_active_time_pct']:>16.2f}  {result['servo_active_time_pct']:>15.2f}  "
            f"{result['servo_rate_p95_deg_s']:>21.2f}"
        )

    print("\n========== Overall metrics ==========")
    for key, value in overall.items():
        if isinstance(value, float):
            print(f"{key}: {value:.6g}")
        else:
            print(f"{key}: {value}")

    summary = {
        "scenario": "constraint_sweep",
        "model": args.model,
        "parameters": {
            "axis": args.sweep_axis,
            "step_levels_m": args.step_levels,
            "attitude_deg": args.test_attitude_deg,
            "thrust_bounds_n": [0.0, args.test_thrust_max],
            "servo_bounds_deg": [-args.servo_angle_max_deg, args.servo_angle_max_deg],
            "servo_time_constant_s": args.effective_servo_time_constant,
            "velocity_bounds_m_s": [-args.test_velocity_max, args.test_velocity_max],
        },
        "overall": overall,
        "segments": per_segment,
    }
    print("METRICS_JSON=" + json.dumps(summary, separators=(",", ":")))


def _excess_travel(values):
    """Return mean per-channel travel beyond the direct start-to-end change."""
    if len(values) < 2:
        return 0.0
    travel = np.sum(np.abs(np.diff(values, axis=0)), axis=0)
    direct = np.abs(values[-1] - values[0])
    return float(np.mean(np.maximum(travel - direct, 0.0)))


def compute_servo_delay_sweep_metrics(
    args,
    ts_sim,
    x_history,
    u_history,
    target_xyz_history,
    target_q_history,
    control_update_history,
    solve_time_history,
    solver_failure_count,
):
    """Measure tracking and actuator oscillation for the servo-model ablation."""
    x = np.asarray(x_history)
    u = np.asarray(u_history)
    xyz_ref = np.asarray(target_xyz_history)
    q_ref = np.asarray(target_q_history)
    control_updates = np.asarray(control_update_history, dtype=bool)
    time_axis = np.arange(len(x)) * ts_sim
    onset_time = 0.0 if args.startup_mode == "cold" else args.startup_warmup_duration

    position_error = np.linalg.norm(x[:, :3] - xyz_ref, axis=1)
    attitude_error_deg = np.degrees(quaternion_geodesic_error(x[:, 6:10], q_ref))
    servo_actual = x[:, 13:17]
    thrust_actual = x[:, 17:21]
    servo_rate = np.diff(servo_actual, axis=0) / ts_sim
    angle_max = np.radians(args.servo_angle_max_deg)

    def summarize(mask):
        if not np.any(mask):
            raise ValueError("Servo-delay metric window contains no simulation samples.")
        command_mask = mask & control_updates
        u_control = u[command_mask]
        if len(u_control) == 0:
            raise ValueError("Servo-delay metric window contains no controller updates.")
        servo_cmd = u_control[:, 4:8]
        thrust_cmd = u_control[:, :4]
        servo_increment = np.abs(np.diff(servo_cmd, axis=0))
        thrust_increment = np.abs(np.diff(thrust_cmd, axis=0))
        rate_selected = np.abs(servo_rate[mask[1:]])
        servo_selected = servo_actual[mask]
        thrust_selected = thrust_actual[mask]
        thrust_any, thrust_samples = bound_activity(thrust_cmd, 0.0, args.test_thrust_max)
        servo_any, servo_samples = bound_activity(servo_cmd, -angle_max, angle_max)

        def increment_stat(values, fn):
            return 0.0 if values.size == 0 else float(fn(values))

        return {
            "position_rmse_m": float(np.sqrt(np.mean(position_error[mask] ** 2))),
            "position_max_m": float(np.max(position_error[mask])),
            "attitude_rmse_deg": float(np.sqrt(np.mean(attitude_error_deg[mask] ** 2))),
            "attitude_max_deg": float(np.max(attitude_error_deg[mask])),
            "servo_cmd_rms_deg": float(np.degrees(np.sqrt(np.mean(servo_cmd**2)))),
            "servo_cmd_peak_to_peak_deg": float(np.degrees(np.max(np.ptp(servo_cmd, axis=0)))),
            "servo_cmd_increment_rms_deg": float(
                np.degrees(increment_stat(servo_increment, lambda v: np.sqrt(np.mean(v**2))))
            ),
            "servo_cmd_increment_p95_deg": float(
                np.degrees(increment_stat(servo_increment, lambda v: np.percentile(v, 95)))
            ),
            "servo_cmd_increment_max_deg": float(np.degrees(increment_stat(servo_increment, np.max))),
            "servo_cmd_excess_travel_deg": float(np.degrees(_excess_travel(servo_cmd))),
            "servo_actual_peak_to_peak_deg": float(np.degrees(np.max(np.ptp(servo_selected, axis=0)))),
            "servo_actual_excess_travel_deg": float(np.degrees(_excess_travel(servo_selected))),
            "servo_rate_rms_deg_s": float(np.degrees(np.sqrt(np.mean(rate_selected**2)))),
            "servo_rate_p95_deg_s": float(np.degrees(np.percentile(rate_selected, 95))),
            "servo_rate_max_deg_s": float(np.degrees(np.max(rate_selected))),
            "thrust_cmd_rms_n": float(np.sqrt(np.mean(thrust_cmd**2))),
            "thrust_cmd_increment_rms_n": increment_stat(thrust_increment, lambda v: np.sqrt(np.mean(v**2))),
            "thrust_cmd_excess_travel_n": _excess_travel(thrust_cmd),
            "thrust_actual_max_n": float(np.max(thrust_selected)),
            "thrust_active_time_pct": 100.0 * thrust_any,
            "thrust_active_samples_pct": 100.0 * thrust_samples,
            "servo_active_time_pct": 100.0 * servo_any,
            "servo_active_samples_pct": 100.0 * servo_samples,
        }

    startup_mask = (time_axis >= onset_time) & (time_axis < onset_time + args.analysis_window_duration)
    overall_mask = time_axis >= onset_time
    startup = summarize(startup_mask)
    overall = summarize(overall_mask)
    solve_times = np.asarray(solve_time_history)
    timing = {
        "solve_time_mean_ms": float(1e3 * np.mean(solve_times)) if solve_times.size else None,
        "solve_time_p95_ms": float(1e3 * np.percentile(solve_times, 95)) if solve_times.size else None,
        "solve_time_max_ms": float(1e3 * np.max(solve_times)) if solve_times.size else None,
        "solver_failures": int(solver_failure_count),
    }

    controller_names = {0: "no_servo", 1: "servo_current_cost", 92: "servo_old_cost"}
    summary = {
        "scenario": "servo_delay_sweep",
        "model": args.model,
        "controller": controller_names[args.model],
        "parameters": {
            "servo_time_constant_s": args.effective_servo_time_constant,
            "controller_servo_time_constant_s": args.effective_controller_servo_time_constant,
            "ocp_sim_method_num_steps": args.ocp_sim_num_steps,
            "ocp_shooting_interval_s": float(args.effective_ocp_shooting_interval),
            "startup_mode": args.startup_mode,
            "startup_warmup_duration_s": args.startup_warmup_duration,
            "analysis_window_duration_s": args.analysis_window_duration,
            "thrust_bounds_n": [0.0, args.test_thrust_max],
            "servo_bounds_deg": [-args.servo_angle_max_deg, args.servo_angle_max_deg],
        },
        "startup": startup,
        "overall": overall,
        "timing": timing,
    }

    print("\n========== Servo-delay sweep configuration ==========")
    print(
        f"model={args.model} ({controller_names[args.model]}), startup={args.startup_mode}, "
        f"servo_time_constant_s={args.effective_servo_time_constant:.6f}, "
        f"ERK_steps={args.ocp_sim_num_steps}, bounds=[0,{args.test_thrust_max:.3f}]N/"
        f"+-{args.servo_angle_max_deg:.3f}deg"
    )
    print("\n========== Startup-window metrics ==========")
    for key, value in startup.items():
        print(f"{key}: {value:.6g}")
    print("\n========== Overall metrics ==========")
    for key, value in overall.items():
        print(f"{key}: {value:.6g}")
    for key, value in timing.items():
        print(f"{key}: {value:.6g}" if isinstance(value, float) else f"{key}: {value}")
    print("METRICS_JSON=" + json.dumps(summary, separators=(",", ":")))


def write_solve_time_csv(file_path, rows):
    """Write one row per controller update with wall-clock and acados timings."""
    output_path = os.path.abspath(file_path)
    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    fieldnames = [
        "control_round",
        "sim_step",
        "sim_time_s",
        "reference_phase",
        "wall_time_ms",
        "acados_time_tot_ms",
        "acados_time_lin_ms",
        "acados_time_qp_ms",
        "solver_status",
    ]
    with open(output_path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"SOLVE_TIME_CSV={output_path}")


def main(args):
    # ========== Init ==========
    # Preserve compatibility with callers that construct the pre-scenario
    # argparse Namespace themselves instead of using this file's CLI parser.
    args.scenario = getattr(args, "scenario", "legacy")
    args.servo_time_constant = getattr(args, "servo_time_constant", None)
    args.ocp_sim_num_steps = getattr(args, "ocp_sim_num_steps", 1)
    args.startup_mode = getattr(args, "startup_mode", "cold")
    args.startup_warmup_duration = getattr(args, "startup_warmup_duration", 2.0)
    args.analysis_window_duration = getattr(args, "analysis_window_duration", 2.0)
    args.solve_time_csv = getattr(args, "solve_time_csv", None)
    args.step_axis = getattr(args, "step_axis", "x")
    args.step_amplitude = getattr(args, "step_amplitude", 0.2)
    args.workpoint_rpy_deg = getattr(args, "workpoint_rpy_deg", [0.0, 0.0, 0.0])
    args.step_time = getattr(args, "step_time", 2.0)
    args.total_duration = getattr(args, "total_duration", 10.0)
    args.steady_window = getattr(args, "steady_window", [9.0, 10.0])
    args.save_run = getattr(args, "save_run", None)
    args.no_build = getattr(args, "no_build", False)
    args.plot_output = getattr(args, "plot_output", None)
    if args.solve_time_csv is not None:
        args.solve_time_csv = os.path.abspath(args.solve_time_csv)
    if args.save_run is not None:
        args.save_run = os.path.abspath(args.save_run)
        if os.path.exists(args.save_run):
            raise FileExistsError(f"Refusing to overwrite existing run bundle: {args.save_run}")
    if args.plot_output is not None:
        args.plot_output = os.path.abspath(args.plot_output)
    if args.ocp_sim_num_steps < 1:
        raise ValueError("ocp_sim_num_steps must be a positive integer.")
    if args.scenario == "constraint_sweep":
        if args.arch != "qd" or args.model not in (1, 4):
            raise ValueError("constraint_sweep supports only qd model 1 (servo NMPC) and model 4 (geometric baseline).")
        if args.test_thrust_max <= 0.0:
            raise ValueError("test_thrust_max must be positive.")
        if not 0.0 < args.servo_angle_max_deg <= 180.0:
            raise ValueError("servo_angle_max_deg must be greater than 0 and no greater than 180 degrees.")
        if args.test_velocity_max <= 0.0 or args.segment_duration <= 0.0 or args.warmup_duration < 0.0:
            raise ValueError("Velocity/duration parameters must be positive (warmup may be zero).")
        if not args.step_levels or any(level <= 0.0 for level in args.step_levels):
            raise ValueError("step_levels must contain positive amplitudes.")
    elif args.scenario == "servo_delay_sweep":
        if args.arch != "qd" or args.model not in (0, 1, 92):
            raise ValueError("servo_delay_sweep supports only qd models 0, 1, and 92.")
        if args.servo_time_constant is None:
            raise ValueError("servo_delay_sweep requires --servo-time-constant.")
        if args.test_thrust_max <= 0.0:
            raise ValueError("test_thrust_max must be positive.")
        if not 0.0 < args.servo_angle_max_deg <= 180.0:
            raise ValueError("servo_angle_max_deg must be greater than 0 and no greater than 180 degrees.")
        if args.startup_warmup_duration < 0.0 or args.analysis_window_duration <= 0.0:
            raise ValueError("startup warm-up must be nonnegative and the analysis window must be positive.")
    elif args.scenario == STEP_RESPONSE_SCENARIO:
        if args.arch != "qd" or args.model != 1 or args.sim_model != 0:
            raise ValueError("step_response requires qd model=1 and sim_model=0.")
        if args.step_axis not in STEP_RESPONSE_AXES:
            raise ValueError(f"Invalid step axis {args.step_axis}.")
        if args.step_amplitude == 0.0:
            raise ValueError("step_amplitude must be nonzero.")
        if len(args.workpoint_rpy_deg) != 3:
            raise ValueError("workpoint_rpy_deg must contain roll, pitch, and yaw.")
        if not 0.0 < args.step_time < args.total_duration:
            raise ValueError("step_time must lie strictly inside the simulation duration.")
        if len(args.steady_window) != 2 or not (
            args.step_time < args.steady_window[0] < args.steady_window[1] <= args.total_duration
        ):
            raise ValueError("steady_window must lie after the step and inside the simulation duration.")
        if args.save_run is None:
            raise ValueError("step_response requires --save-run for reproducibility.")
    if args.servo_time_constant is not None:
        if args.arch != "qd":
            raise ValueError("servo_time_constant override is currently supported only for the qd architecture.")
        if args.servo_time_constant <= 0.0:
            raise ValueError("servo_time_constant must be positive.")
        set_servo_time_constant(phys_art, args.servo_time_constant)
        set_servo_time_constant(phys_omni, args.servo_time_constant)

    # ---------- Controller ----------
    if args.arch == "qd":
        if args.model == 0:
            nmpc = NMPCTiltQdNoServo(phys=phys_art, ocp_sim_method_num_steps=args.ocp_sim_num_steps)
        elif args.model == 1:
            nmpc = NMPCTiltQdServo(
                build=not args.no_build,
                phys=phys_art,
                ocp_sim_method_num_steps=args.ocp_sim_num_steps,
            )
        elif args.model == 2:
            nmpc = NMPCTiltQdThrust(phys=phys_art)
        elif args.model == 3:
            nmpc = NMPCTiltQdServoThrust(phys=phys_art)
        elif args.model == 4:
            nmpc = GeomControllerTiltQd(phys=phys_art)

        elif args.model == 21:
            nmpc = NMPCTiltQdServoDist(phys=phys_omni)
        elif args.model == 22:
            nmpc = NMPCTiltQdServoThrustDist(phys=phys_omni)
        elif args.model == 29:
            nmpc = GeomControllerTiltQd(phys=phys_omni)

        # Archived methods
        elif args.model == 91:
            nmpc = NMPCTiltQdNoServoAcCost()
        elif args.model == 92:
            nmpc = NMPCTiltQdServoOldCost(phys=phys_art, ocp_sim_method_num_steps=args.ocp_sim_num_steps)
        elif args.model == 93:
            nmpc = NMPCTiltQdServoDiff()
            alpha_integ = np.zeros(4)
        elif args.model == 94:
            nmpc = NMPCTiltQdServoDragDist()
        elif args.model == 95:
            nmpc = NMPCTiltQdServoThrustDrag()
        elif args.model == 96:
            nmpc = NMPCTiltQdServoWCogEndDist()
        else:
            raise ValueError(f"Invalid control model {args.model}.")

    elif args.arch == "bi":
        if args.model == 0:
            nmpc = NMPCTiltBiServo()
        elif args.model == 1:
            nmpc = NMPCTiltBi2OrdServo()
        else:
            raise ValueError(f"Invalid model {args.model}.")

    elif args.arch == "tri":
        if args.model == 0:
            nmpc = NMPCTiltTriServo()
        elif args.model == 1:
            nmpc = NMPCTiltTriServoDist()
        else:
            raise ValueError(f"Invalid model {args.model}.")

    else:
        raise ValueError(f"Invalid robot architecture {args.arch}.")

    is_baseline = isinstance(nmpc, GeomControllerTiltQd)

    # Get time constants
    if nmpc.include_servo_model:
        t_servo_ctrl = nmpc.phys.t_servo
    else:
        t_servo_ctrl = 0.0
    ts_ctrl = nmpc.params["T_samp"]
    args.effective_ocp_shooting_interval = nmpc.params.get("T_step", 0.0)

    # OCP solver
    if is_baseline:
        ocp_solver = None
        nx = nmpc.nx
        nu = nmpc.nu

        u_init = np.zeros(nu)
    else:
        ocp_solver = nmpc.get_ocp_solver()
        nx = ocp_solver.acados_ocp.dims.nx
        nu = ocp_solver.acados_ocp.dims.nu
        n_param = ocp_solver.acados_ocp.dims.np

        x_init = np.zeros(nx)
        x_init[6] = 1.0  # qw
        u_init = np.zeros(nu)

        for stage in range(ocp_solver.N + 1):
            ocp_solver.set(stage, "x", x_init)
        for stage in range(ocp_solver.N):
            ocp_solver.set(stage, "u", u_init)

    if args.scenario == "constraint_sweep":
        apply_constraint_sweep_bounds(nmpc, ocp_solver, args, is_baseline)
    elif args.scenario == "servo_delay_sweep":
        apply_common_actuator_bounds(nmpc, ocp_solver, args, is_baseline)

    # ---------- Simulator ----------
    if args.arch == "qd":
        sim_phy = phys_omni if 20 < args.model < 30 else phys_art
        if args.sim_model == 0:
            sim_nmpc = NMPCTiltQdServoThrust(
                build=not args.no_build, phys=sim_phy
            )  # Consider both the servo delay and the thrust delay
        elif args.sim_model == 1:
            sim_nmpc = NMPCTiltQdServoThrustDrag(
                build=not args.no_build, phys=sim_phy
            )  # Also consider drag in wrench formulation
        else:
            raise ValueError(f"Invalid sim model {args.sim_model}.")

    elif args.arch == "bi":
        if args.sim_model == 0:
            sim_nmpc = NMPCTiltBiServo()
        # elif args.sim_model == 1:
        #     sim_nmpc = NMPCTiltBi2OrdServo()   # This model is wrong
        else:
            raise ValueError(f"Invalid sim model {args.sim_model}.")

    elif args.arch == "tri":
        sim_nmpc = NMPCTiltTriServo()

    else:
        raise ValueError(f"Invalid robot architecture {args.arch}.")

    # Get time constants
    if sim_nmpc.include_servo_model:
        t_servo_sim = sim_nmpc.phys.t_servo
    else:
        t_servo_sim = 0.0
    if sim_nmpc.include_thrust_model:
        t_rotor_sim = sim_nmpc.phys.t_rotor
    else:
        t_rotor_sim = 0.0
    args.effective_servo_time_constant = t_servo_sim
    args.effective_controller_servo_time_constant = t_servo_ctrl

    ts_sim = 0.001

    if args.scenario == "constraint_sweep":
        t_total_sim = args.warmup_duration + args.segment_duration * len(args.step_levels)
    elif args.scenario == "servo_delay_sweep":
        startup_shift = 0.0 if args.startup_mode == "cold" else args.startup_warmup_duration
        t_total_sim = 15.0 + startup_shift
    elif args.scenario == STEP_RESPONSE_SCENARIO:
        t_total_sim = args.total_duration
    else:
        t_total_sim = 15.0
        if args.plot_type == 1:
            t_total_sim = 4.0
        if args.plot_type == 2:
            t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # Sim solver
    sim_solver = sim_nmpc.create_acados_sim_solver(ts_sim, build=not args.no_build)
    nx_sim = sim_solver.acados_sim.dims.nx

    # State Initialization
    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw

    # ---------- Reference ----------
    reference_generator = nmpc.get_reference_generator() if not is_baseline else None

    equilibrium_control = u_init.copy()
    if args.scenario == STEP_RESPONSE_SCENARIO:
        workpoint_xyz, workpoint_rpy, _ = get_step_response_target(args, 0.0)
        xr_eq, ur_eq = reference_generator.compute_trajectory(workpoint_xyz, workpoint_rpy)
        equilibrium_control = np.concatenate((ur_eq[0, :4], xr_eq[0, 13:17]))

        x_init = xr_eq[0].copy()
        x_init_sim = np.zeros(nx_sim)
        x_init_sim[:13] = x_init[:13]
        x_init_sim[13:17] = xr_eq[0, 13:17]
        x_init_sim[17:21] = ur_eq[0, :4]

        lower_u = np.asarray(ocp_solver.acados_ocp.constraints.lbu, dtype=float)
        upper_u = np.asarray(ocp_solver.acados_ocp.constraints.ubu, dtype=float)
        if np.any(equilibrium_control < lower_u) or np.any(equilibrium_control > upper_u):
            raise ValueError(
                "The requested workpoint equilibrium violates the NMPC input constraints: "
                f"u_eq={equilibrium_control}, bounds=[{lower_u}, {upper_u}]."
            )
        for stage in range(ocp_solver.N + 1):
            ocp_solver.set(stage, "x", x_init)
        for stage in range(ocp_solver.N):
            ocp_solver.set(stage, "u", equilibrium_control)

    # ---------- Visualization ----------
    viz = Visualizer(
        args.arch,
        N_sim,
        nx_sim,
        nu,
        x_init_sim,
        tilt=nmpc.tilt,
        include_servo_model=sim_nmpc.include_servo_model,
        include_thrust_model=sim_nmpc.include_thrust_model,
        include_cog_dist_model=sim_nmpc.include_cog_dist_model,
    )

    # Prepare containers to record simulation data (x and u) for future comparison
    x_history = []
    u_history = []
    target_xyz_history = []
    target_q_history = []
    segment_history = []
    control_update_history = []
    solve_time_history = []
    solve_time_rows = []
    control_round = 0
    solver_failure_count = 0

    is_sqp_change = False
    t_sqp_start = 2.5
    t_sqp_end = 3.0

    control_stride = None
    if args.scenario == STEP_RESPONSE_SCENARIO:
        control_stride = int(round(ts_ctrl / ts_sim))
        if control_stride < 1 or not np.isclose(control_stride * ts_sim, ts_ctrl, rtol=0.0, atol=1e-12):
            raise ValueError("The controller period must be an integer multiple of the simulation period.")

    # ========== Run simulation ==========
    u_cmd = equilibrium_control.copy()
    t_ctl = 0.0
    x_now_sim = x_init_sim
    for i in range(N_sim):
        # --------- Update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim

        # --------- Update state estimation ---------
        # Assemble state from simulation and disturbance estimation
        if nmpc.include_cog_dist_model:
            x_now = np.zeros(nx)
            x_now[: nx - 6] = deepcopy(x_now_sim[: nx - 6])
        else:
            x_now = deepcopy(x_now_sim[:nx])  # The dimension of x_now may be smaller than x_now_sim

        # Access from less indices
        if (nmpc.include_thrust_model and not nmpc.include_servo_model) and (
            sim_nmpc.include_servo_model and sim_nmpc.include_thrust_model
        ):
            if args.arch == "bi":
                x_now[13:15] = deepcopy(x_now_sim[15:17])
            elif args.arch == "tri":
                x_now[13:16] = deepcopy(x_now_sim[16:19])
            elif args.arch == "qd":
                x_now[13:17] = deepcopy(x_now_sim[17:21])

        # -------- Update control target --------
        if args.scenario == "constraint_sweep":
            target_xyz, target_rpy, active_segment = get_constraint_sweep_target(args, t_now)
        elif args.scenario == "servo_delay_sweep":
            target_xyz, target_rpy, active_segment = get_servo_delay_sweep_target(args, t_now)
        elif args.scenario == STEP_RESPONSE_SCENARIO:
            target_xyz, target_rpy, active_segment = get_step_response_target(args, t_now)
        else:
            active_segment = -1
            target_xyz = np.array([[0.3, 0.6, 1.0]]).T
            target_rpy = np.array([[0.0, 0.0, 0.0]]).T

            if args.plot_type == 2:
                target_xyz = np.array([[0.0, 0.0, 0.0]]).T
                target_rpy = np.array([[0.5, 0.5, 0.5]]).T

            if t_total_sim > 2.0:
                if 2.0 <= t_now < 6:
                    target_xyz = np.array([[0.3, 0.6, 1.0]]).T

                    roll = 30.0 / 180.0 * np.pi
                    pitch = 60.0 / 180.0 * np.pi
                    yaw = 90.0 / 180.0 * np.pi
                    target_rpy = np.array([[roll, pitch, yaw]]).T

                # if 3.0 <= t_now < 5.5:
                #     assert t_sqp_end <= 3.0
                #     target_xyz = np.array([[1.0, 1.0, 1.0]]).T
                #     target_rpy = np.array([[0.0, 0.0, 0.0]]).T
                # if t_now >= 5.5:
                #     target_xyz = np.array([[1.0, 1.0, 1.0]]).T

                #     roll = 30.0 / 180.0 * np.pi
                #     pitch = 0.0 / 180.0 * np.pi
                #     yaw = 0.0 / 180.0 * np.pi
                #     target_rpy = np.array([[roll, pitch, yaw]]).T

                if t_now >= 6:
                    assert t_sqp_end <= 3.0
                    target_xyz = np.array([[1.0, 1.0, 1.0]]).T
                    target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        target_q = tf.quaternion_from_euler(*target_rpy.flatten(), axes="sxyz")

        if not is_baseline:
            # Compute reference trajectory from target pose
            xr, ur = reference_generator.compute_trajectory(target_xyz, target_rpy)

            if args.plot_type == 2:
                if nx > 13:
                    xr[:, 13:] = 0.0
                if args.arch == "bi":
                    ur[:, 2:] = 0.0
                elif args.arch == "tri":
                    ur[:, 3:] = 0.0
                elif args.arch == "qd":
                    ur[:, 4:] = 0.0

            # -------- Set SQP mode --------
            if is_sqp_change and t_sqp_start > t_sqp_end:
                if t_now >= t_sqp_start:
                    ocp_solver.solver_options["nlp_solver_type"] = "SQP"

                if t_now >= t_sqp_end:
                    ocp_solver.solver_options["nlp_solver_type"] = "SQP_RTI"

        # -------- Update solver --------
        comp_time_start = time.perf_counter()
        control_was_updated = False
        solver_failed = False
        acados_timing = {
            "time_tot": None,
            "time_lin": None,
            "time_qp": None,
        }
        solver_status = 0

        is_control_time = i % control_stride == 0 if control_stride is not None else t_ctl >= ts_ctrl
        if is_control_time:
            t_ctl = 0.0
            control_was_updated = True

            if is_baseline:
                u_cmd = nmpc.compute_control(x_now, target_xyz, target_rpy)
            else:
                # 0 ~ N-1
                for j in range(ocp_solver.N):
                    yr = np.concatenate((xr[j, :], ur[j, :]))
                    ocp_solver.set(j, "yref", yr)
                    quaternion_r = xr[j, 6:10]
                    nmpc.acados_init_p[0:4] = quaternion_r
                    ocp_solver.set(j, "p", nmpc.acados_init_p)  # For nonlinear quaternion error

                # N
                yr = xr[ocp_solver.N, :]
                ocp_solver.set(ocp_solver.N, "yref", yr)  # Final state of x, no u
                quaternion_r = xr[ocp_solver.N, 6:10]
                nmpc.acados_init_p[0:4] = quaternion_r
                ocp_solver.set(ocp_solver.N, "p", nmpc.acados_init_p)  # For nonlinear quaternion error

                # Compute control feedback and take the first action
                try:
                    u_cmd = ocp_solver.solve_for_x0(x_now)
                except Exception as e:
                    print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                    solver_failure_count += 1
                    solver_failed = True

                solver_status = int(ocp_solver.status)

        comp_time_end = time.perf_counter()
        viz.comp_time[i] = comp_time_end - comp_time_start
        if control_was_updated:
            wall_time = comp_time_end - comp_time_start
            solve_time_history.append(wall_time)
            if not is_baseline:
                for field in acados_timing:
                    try:
                        acados_timing[field] = float(ocp_solver.get_stats(field))
                    except (TypeError, ValueError):
                        acados_timing[field] = None
            solve_time_rows.append(
                {
                    "control_round": control_round,
                    "sim_step": i,
                    "sim_time_s": t_now,
                    "reference_phase": active_segment,
                    "wall_time_ms": 1e3 * wall_time,
                    "acados_time_tot_ms": (
                        None if acados_timing["time_tot"] is None else 1e3 * acados_timing["time_tot"]
                    ),
                    "acados_time_lin_ms": (
                        None if acados_timing["time_lin"] is None else 1e3 * acados_timing["time_lin"]
                    ),
                    "acados_time_qp_ms": (None if acados_timing["time_qp"] is None else 1e3 * acados_timing["time_qp"]),
                    "solver_status": solver_status,
                }
            )
            control_round += 1
        if solver_failed:
            break

        if args.arch == "qd":
            # Use previous servo angle as reference
            if type(nmpc) is NMPCTiltQdNoServoAcCost:
                nmpc.update_a_prev(u_cmd.item(4), u_cmd.item(5), u_cmd.item(6), u_cmd.item(7))

            # Use servo angle derivative as state and therefore integrate servo angle command
            if nmpc.include_servo_derivative:
                alpha_integ += u_cmd[4:] * ts_ctrl
                u_cmd[4:] = alpha_integ  # convert from delta input to real input

        # --------- Update simulation ----------
        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", u_cmd)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # Save current simulation data for later comparison
        x_history.append(x_now_sim.copy())
        u_history.append(u_cmd.copy())
        target_xyz_history.append(target_xyz.flatten().copy())
        target_q_history.append(target_q.copy())
        segment_history.append(active_segment)
        control_update_history.append(control_was_updated)

        # --------- Update visualizer ----------
        viz.update(i, x_now_sim, u_cmd)  # Note: The recording frequency of u_cmd is the same as ts_sim

    if args.scenario == "constraint_sweep":
        compute_constraint_sweep_metrics(
            args,
            ts_sim,
            x_history,
            u_history,
            target_xyz_history,
            target_q_history,
            segment_history,
            solve_time_history,
            solver_failure_count,
        )
    elif args.scenario == "servo_delay_sweep":
        compute_servo_delay_sweep_metrics(
            args,
            ts_sim,
            x_history,
            u_history,
            target_xyz_history,
            target_q_history,
            control_update_history,
            solve_time_history,
            solver_failure_count,
        )
    elif args.scenario == STEP_RESPONSE_SCENARIO:
        completed = len(x_history) == N_sim
        time_input = np.arange(len(x_history), dtype=float) * ts_sim
        time_state = np.arange(len(x_history) + 1, dtype=float) * ts_sim
        state_plant = np.vstack((x_init_sim, np.asarray(x_history))) if x_history else np.asarray(x_init_sim)[None, :]
        reference_position = np.zeros((len(time_state), 3))
        reference_quaternion = np.zeros((len(time_state), 4))
        reference_rpy = np.zeros((len(time_state), 3))
        for ref_index, ref_time in enumerate(time_state):
            ref_xyz, ref_rpy, _ = get_step_response_target(args, ref_time)
            reference_position[ref_index] = ref_xyz.flatten()
            reference_rpy[ref_index] = ref_rpy.flatten()
            reference_quaternion[ref_index] = tf.quaternion_from_euler(*ref_rpy.flatten(), axes="sxyz")

        pre_mask = (time_state >= max(0.0, args.step_time - 0.5)) & (time_state < args.step_time)
        if np.any(pre_mask):
            pre_position_error = np.linalg.norm(state_plant[pre_mask, :3] - reference_position[pre_mask], axis=1)
            pre_attitude_error = quaternion_geodesic_error(state_plant[pre_mask, 6:10], reference_quaternion[pre_mask])
            pre_velocity = np.linalg.norm(state_plant[pre_mask, 3:6], axis=1)
            pre_angular_velocity = np.linalg.norm(state_plant[pre_mask, 10:13], axis=1)
            stability_values = {
                "position_error_max_m": float(np.max(pre_position_error)),
                "attitude_error_max_deg": float(np.degrees(np.max(pre_attitude_error))),
                "velocity_max_m_s": float(np.max(pre_velocity)),
                "angular_velocity_max_rad_s": float(np.max(pre_angular_velocity)),
            }
            pre_step_stable = (
                stability_values["position_error_max_m"] <= 0.01
                and stability_values["attitude_error_max_deg"] <= 1.0
                and stability_values["velocity_max_m_s"] <= 0.02
                and stability_values["angular_velocity_max_rad_s"] <= 0.02
            )
        else:
            stability_values = {
                "position_error_max_m": None,
                "attitude_error_max_deg": None,
                "velocity_max_m_s": None,
                "angular_velocity_max_rad_s": None,
            }
            pre_step_stable = False

        state_constraints = ocp_solver.acados_ocp.constraints
        input_lower = np.asarray(state_constraints.lbu, dtype=float)
        input_upper = np.asarray(state_constraints.ubu, dtype=float)
        case = StepCase(args.step_axis, args.step_amplitude, tuple(args.workpoint_rpy_deg))
        solve_wall = np.asarray([row["wall_time_ms"] * 1e-3 for row in solve_time_rows])

        def timing_array(field):
            return np.asarray(
                [np.nan if row[field] is None else row[field] * 1e-3 for row in solve_time_rows], dtype=float
            )

        metadata = {
            "kind": "nmpc_step_response",
            "scenario": STEP_RESPONSE_SCENARIO,
            "case_id": case.slug,
            "controller_model": type(nmpc).__name__,
            "plant_model": type(sim_nmpc).__name__,
            "workpoint_rpy_deg": list(map(float, args.workpoint_rpy_deg)),
            "step": {
                "axis": args.step_axis,
                "amplitude_input": float(args.step_amplitude),
                "amplitude_input_unit": case.amplitude_unit,
                "amplitude_si": float(case.amplitude_si),
            },
            "timing": {
                "controller_period_s": float(ts_ctrl),
                "simulation_period_s": float(ts_sim),
                "step_time_s": float(args.step_time),
                "total_duration_s": float(args.total_duration),
                "steady_window_s": list(map(float, args.steady_window)),
            },
            "equilibrium": {"control": equilibrium_control.tolist(), "state_controller": x_init.tolist()},
            "constraints": {
                "state": {
                    "indices": np.asarray(state_constraints.idxbx, dtype=int).tolist(),
                    "lower": np.asarray(state_constraints.lbx, dtype=float).tolist(),
                    "upper": np.asarray(state_constraints.ubx, dtype=float).tolist(),
                },
                "input": {"lower": input_lower.tolist(), "upper": input_upper.tolist()},
            },
            "controller_parameters": {
                key: value.item() if isinstance(value, np.generic) else value for key, value in nmpc.params.items()
            },
            "physical_parameters": list(map(float, nmpc.phys.physical_param_list)),
            "validation": {
                "completed": completed,
                "pre_step_stable": pre_step_stable,
                "solver_failure_count": int(solver_failure_count),
                **stability_values,
            },
        }
        arrays = {
            "time_state": time_state,
            "time_input": time_input,
            "state_plant": state_plant,
            "state_controller": state_plant[:, :nx],
            "reference_position": reference_position,
            "reference_quaternion": reference_quaternion,
            "reference_rpy": reference_rpy,
            "control_applied": np.asarray(u_history).reshape((-1, nu)),
            "control_updated": np.asarray(control_update_history, dtype=bool),
            "solve_time": np.asarray([row["sim_time_s"] for row in solve_time_rows]),
            "solve_wall_time": solve_wall,
            "solve_acados_time_tot": timing_array("acados_time_tot_ms"),
            "solve_acados_time_lin": timing_array("acados_time_lin_ms"),
            "solve_acados_time_qp": timing_array("acados_time_qp_ms"),
            "solver_status": np.asarray([row["solver_status"] for row in solve_time_rows], dtype=int),
        }
        save_run_bundle(args.save_run, metadata, **arrays)
        print("STEP_RESPONSE_RUN=" + args.save_run)
        if not completed:
            raise RuntimeError("The step-response simulation ended early; a partial run bundle was saved.")
        metrics = compute_run_metrics(arrays, metadata)
        print("METRICS_JSON=" + json.dumps(metrics, separators=(",", ":"), allow_nan=True))

    if args.solve_time_csv is not None:
        write_solve_time_csv(args.solve_time_csv, solve_time_rows)

    # ========== Visualize ==========
    ctrl_name = "geom_pinv_baseline" if is_baseline else ocp_solver.acados_ocp.model.name
    if not args.no_viz:
        if args.plot_type == 0:
            viz.visualize(
                ctrl_name,
                sim_solver.model_name,
                ts_ctrl,
                ts_sim,
                t_total_sim,
                t_servo_ctrl=t_servo_ctrl,
                t_servo_sim=t_servo_sim,
            )
        elif args.plot_type == 1:
            viz.visualize_less(ts_sim, t_total_sim)
        elif args.plot_type == 2:
            viz.visualize_rpy(ctrl_name, ts_sim, t_total_sim)
        elif args.plot_type == 3:
            viz.visualize_tracking_actuators(
                ts_sim,
                t_total_sim,
                np.asarray(target_xyz_history),
                np.asarray(target_q_history),
                output_prefix=args.plot_output,
            )

    if args.save_data:
        file_path = args.file_path

        np.savez(
            file_path + f"nmpc_{type(nmpc).__name__}_sim_{type(sim_nmpc).__name__}.npz",
            x=np.array(x_history),
            u=np.array(u_history),
        )

    return np.array(x_history), np.array(u_history)


if __name__ == "__main__":
    # Read command line arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models.")
    parser.add_argument(
        "model",
        type=int,
        help="The NMPC model to be simulated. "
        "Options: 0 (basic model), 1 (servo), "
        "2 (thrust), 3(servo+thrust), "
        "4 (baseline: geometric ctrl + pinv allocation), "
        "21 (servo+dist), 22 (servo+thrust+dist), "
        "29 (baseline with omni phys params), "
        "91(no_servo_new_cost), 92(servo_old_cost), "
        "93(servo_diff), 94(servo+drag+dist), "
        "95 (servo+thrust+drag), 96 (servo+drag_param+dist).",
    )

    parser.add_argument(
        "-sim",
        "--sim_model",
        type=int,
        default=0,
        help="The simulation model. " "Options: 0 (default: servo+thrust), " "1 (servo+thrust+drag).",
    )

    parser.add_argument(
        "-p",
        "--plot_type",
        type=int,
        default=0,
        help="The type of plot. "
        "Options: 0 (default: full), 1 (less), 2 (only rpy), "
        "3 (position/attitude/servo/thrust cmd-state tracking).",
    )

    parser.add_argument(
        "-a", "--arch", type=str, default="qd", help="The robot's architecture. Options: bi, tri, qd (default)."
    )

    parser.add_argument(
        "--no_viz",
        action="store_true",
        help="Disable visualization after simulation. Note that this is different from the plot_type option, "
        "because plot_type also decides the simulation parameters.",
    )

    parser.add_argument("-s", "--save_data", action="store_true", help="Save simulation x and u data to file")

    parser.add_argument("--file_path", type=str, default=f"../../../../test/data/", help="Path to save the data file")

    parser.add_argument(
        "--scenario",
        choices=("legacy", "constraint_sweep", "servo_delay_sweep", STEP_RESPONSE_SCENARIO),
        default="legacy",
        help="Simulation task. The default preserves the original setpoint task.",
    )
    parser.add_argument(
        "--step-axis",
        choices=STEP_RESPONSE_AXES,
        default="x",
        help="Commanded DoF for step_response.",
    )
    parser.add_argument(
        "--step-amplitude",
        type=float,
        default=0.2,
        help="Step amplitude: metres for position axes and degrees for attitude axes.",
    )
    parser.add_argument(
        "--workpoint-rpy-deg",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=[0.0, 0.0, 0.0],
        help="Attitude workpoint [deg] for step_response.",
    )
    parser.add_argument("--step-time", type=float, default=2.0, help="Step application time [s].")
    parser.add_argument("--total-duration", type=float, default=10.0, help="Step-response duration [s].")
    parser.add_argument(
        "--steady-window",
        type=float,
        nargs=2,
        metavar=("START", "END"),
        default=[9.0, 10.0],
        help="Steady-state metric window [s].",
    )
    parser.add_argument("--save-run", type=str, default=None, help="Structured step-response NPZ output path.")
    parser.add_argument(
        "--no-build",
        action="store_true",
        help="Reuse previously generated acados controller and simulator code.",
    )
    parser.add_argument(
        "--plot-output",
        type=str,
        default=None,
        help="Output prefix for plot_type=3 PNG/PDF figures.",
    )
    parser.add_argument(
        "--test-thrust-max",
        type=float,
        default=12.0,
        help="Common per-rotor thrust-command upper bound [N] for comparison scenarios.",
    )
    parser.add_argument(
        "--servo-angle-max-deg",
        type=float,
        default=60.0,
        help="Common symmetric servo command/state bound [deg] for comparison scenarios.",
    )
    parser.add_argument(
        "--servo-time-constant",
        type=float,
        default=None,
        help="Override the qd controller and simulator servo time constant [s].",
    )
    parser.add_argument(
        "--ocp-sim-num-steps",
        type=int,
        default=1,
        help="Number of ERK integration substeps in every NMPC shooting interval.",
    )
    parser.add_argument(
        "--startup-mode",
        choices=("cold", "hover_warm"),
        default="cold",
        help="Start the servo-delay task immediately or after a hover stabilization period.",
    )
    parser.add_argument(
        "--startup-warmup-duration",
        type=float,
        default=2.0,
        help="Hover duration [s] before the shifted legacy task in hover_warm mode.",
    )
    parser.add_argument(
        "--analysis-window-duration",
        type=float,
        default=2.0,
        help="Duration [s] after task onset used for startup oscillation metrics.",
    )
    parser.add_argument(
        "--solve-time-csv",
        type=str,
        default=None,
        help="Optional CSV path for per-controller-update wall and acados solve timings.",
    )
    parser.add_argument(
        "--test-velocity-max",
        type=float,
        default=5.0,
        help="Symmetric NMPC prediction velocity bound [m/s] used to avoid confounding actuator saturation.",
    )
    parser.add_argument(
        "--test-attitude-deg",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=[25.0, 20.0, 0.0],
        help="Fixed roll, pitch, yaw target [deg] for constraint_sweep.",
    )
    parser.add_argument(
        "--step-levels",
        type=float,
        nargs="+",
        default=[0.25, 0.50, 0.75, 1.00, 1.25, 1.50],
        help="Positive absolute horizontal position amplitudes [m]; their signs alternate.",
    )
    parser.add_argument(
        "--sweep-axis",
        choices=("x", "y"),
        default="x",
        help="Horizontal axis used for constraint_sweep.",
    )
    parser.add_argument(
        "--warmup-duration",
        type=float,
        default=3.0,
        help="Attitude-establishment duration before the position sweep [s].",
    )
    parser.add_argument(
        "--segment-duration",
        type=float,
        default=2.0,
        help="Duration of every signed position step [s].",
    )

    args = parser.parse_args()
    main(args)
