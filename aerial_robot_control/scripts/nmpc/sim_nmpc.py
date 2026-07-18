import sys, os
from copy import deepcopy
import time
import numpy as np
import argparse
import json
import transformations as tf

from nmpc_tilt_mt.utils.nmpc_viz import Visualizer

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


def apply_constraint_sweep_bounds(nmpc, ocp_solver, args, is_baseline):
    """Apply identical test actuator limits to NMPC and geometric baseline."""
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

    # Model 1 has constrained states [v(3), w(3), servo angle(4)]. Widen the
    # velocity bounds so that this test is governed by actuator constraints.
    lbx = np.asarray(ocp_solver.acados_ocp.constraints.lbx, dtype=float).copy()
    ubx = np.asarray(ocp_solver.acados_ocp.constraints.ubx, dtype=float).copy()
    lbx[:3] = -args.test_velocity_max
    ubx[:3] = args.test_velocity_max
    lbx[-4:] = -angle_max
    ubx[-4:] = angle_max
    for stage in range(1, ocp_solver.N):
        ocp_solver.constraints_set(stage, "lbx", lbx)
        ocp_solver.constraints_set(stage, "ubx", ubx)

    lbx_e = np.asarray(ocp_solver.acados_ocp.constraints.lbx_e, dtype=float).copy()
    ubx_e = np.asarray(ocp_solver.acados_ocp.constraints.ubx_e, dtype=float).copy()
    lbx_e[:3] = -args.test_velocity_max
    ubx_e[:3] = args.test_velocity_max
    lbx_e[-4:] = -angle_max
    ubx_e[-4:] = angle_max
    ocp_solver.constraints_set(ocp_solver.N, "lbx", lbx_e)
    ocp_solver.constraints_set(ocp_solver.N, "ubx", ubx_e)


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
            "velocity_bounds_m_s": [-args.test_velocity_max, args.test_velocity_max],
        },
        "overall": overall,
        "segments": per_segment,
    }
    print("METRICS_JSON=" + json.dumps(summary, separators=(",", ":")))


def main(args):
    # ========== Init ==========
    # Preserve compatibility with callers that construct the pre-scenario
    # argparse Namespace themselves instead of using this file's CLI parser.
    args.scenario = getattr(args, "scenario", "legacy")
    if args.scenario == "constraint_sweep":
        if args.arch != "qd" or args.model not in (1, 4):
            raise ValueError("constraint_sweep supports only qd model 1 (servo NMPC) and model 4 (geometric baseline).")
        if args.test_thrust_max <= 0.0:
            raise ValueError("test_thrust_max must be positive.")
        if not 0.0 < args.servo_angle_max_deg < 180.0:
            raise ValueError("servo_angle_max_deg must be between 0 and 180 degrees.")
        if args.test_velocity_max <= 0.0 or args.segment_duration <= 0.0 or args.warmup_duration < 0.0:
            raise ValueError("Velocity/duration parameters must be positive (warmup may be zero).")
        if not args.step_levels or any(level <= 0.0 for level in args.step_levels):
            raise ValueError("step_levels must contain positive amplitudes.")

    # ---------- Controller ----------
    if args.arch == "qd":
        if args.model == 0:
            nmpc = NMPCTiltQdNoServo(phys=phys_art)
        elif args.model == 1:
            nmpc = NMPCTiltQdServo(phys=phys_art)
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
            nmpc = NMPCTiltQdServoOldCost()
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

    # ---------- Simulator ----------
    if args.arch == "qd":
        sim_phy = phys_omni if 20 < args.model < 30 else phys_art
        if args.sim_model == 0:
            sim_nmpc = NMPCTiltQdServoThrust(phys=sim_phy)  # Consider both the servo delay and the thrust delay
        elif args.sim_model == 1:
            sim_nmpc = NMPCTiltQdServoThrustDrag(phys=sim_phy)  # Also consider drag in wrench formulation
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

    ts_sim = 0.001  # or 0.001

    if args.scenario == "constraint_sweep":
        t_total_sim = args.warmup_duration + args.segment_duration * len(args.step_levels)
    else:
        t_total_sim = 15.0
        if args.plot_type == 1:
            t_total_sim = 4.0
        if args.plot_type == 2:
            t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # Sim solver
    sim_solver = sim_nmpc.create_acados_sim_solver(ts_sim, build=True)
    nx_sim = sim_solver.acados_sim.dims.nx

    # State Initialization
    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw

    # ---------- Reference ----------
    reference_generator = nmpc.get_reference_generator() if not is_baseline else None

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
    solve_time_history = []
    solver_failure_count = 0

    is_sqp_change = False
    t_sqp_start = 2.5
    t_sqp_end = 3.0

    # ========== Run simulation ==========
    u_cmd = u_init
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
        comp_time_start = time.time()
        control_was_updated = False
        solver_failed = False

        if t_ctl >= ts_ctrl:
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

        comp_time_end = time.time()
        viz.comp_time[i] = comp_time_end - comp_time_start
        if control_was_updated:
            solve_time_history.append(comp_time_end - comp_time_start)
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
        choices=("legacy", "constraint_sweep"),
        default="legacy",
        help="Simulation task. The default preserves the original setpoint task.",
    )
    parser.add_argument(
        "--test-thrust-max",
        type=float,
        default=12.0,
        help="Common per-rotor thrust-command upper bound [N] for constraint_sweep.",
    )
    parser.add_argument(
        "--servo-angle-max-deg",
        type=float,
        default=60.0,
        help="Common symmetric servo command/state bound [deg] for constraint_sweep.",
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
