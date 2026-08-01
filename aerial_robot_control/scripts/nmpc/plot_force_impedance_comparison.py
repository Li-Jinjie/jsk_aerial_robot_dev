#!/usr/bin/env python3
import argparse
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401 - registers the SciencePlots styles.

from nmpc_tilt_mt.utils.force_impedance_experiment import (
    PAPER_RESULTS_ROOT,
    SCENARIO_DURATION,
    SCENARIO_EVENT_TIMES,
    SCENARIO_NAME,
    STEADY_STATE_WINDOWS,
    load_run_bundle,
)


AXES = ("x", "y", "z")
BASELINE_WINDOW = (1.5, 2.0)
FRAME_SYMBOLS = {"cog": "B", "ee": "T"}


def _configure_plot_style():
    plt.style.use(["science", "grid"])
    plt.rcParams.update(
        {
            "font.size": 16,
            "axes.labelsize": 17,
            "axes.titlesize": 17,
            "xtick.labelsize": 16,
            "ytick.labelsize": 16,
            "legend.fontsize": 16,
            "figure.titlesize": 17,
            "lines.linewidth": 1.8,
        }
    )


def _validate_bundle(path, data, metadata):
    if metadata.get("scenario") != SCENARIO_NAME:
        raise ValueError(f"{path} is not a {SCENARIO_NAME!r} run.")

    required = ("time_state", "time_input", "applied_wrench_ee")
    missing = [name for name in required if name not in data.files]
    if missing:
        raise ValueError(f"{path} is missing arrays: {', '.join(missing)}")

    if "state_plot" not in data.files and "state_ee" not in data.files:
        raise ValueError(f"{path} is missing both state_plot and the legacy state_ee array.")
    state = data["state_plot"] if "state_plot" in data.files else data["state_ee"]
    if state.ndim != 2 or state.shape[1] < 6:
        raise ValueError(f"{path}: plotted state must have at least six columns.")


def _validate_impedance_match(nmpc_metadata, truth_metadata):
    for name in ("mass", "damping", "stiffness"):
        lhs = np.asarray(nmpc_metadata["impedance"][name], dtype=float)
        rhs = np.asarray(truth_metadata["impedance"][name], dtype=float)
        if not np.allclose(lhs, rhs, rtol=1e-10, atol=1e-12):
            raise ValueError(f"Impedance {name} differs: NMPC={lhs}, truth={rhs}.")
    nmpc_duration = float(nmpc_metadata.get("scenario_duration", SCENARIO_DURATION))
    truth_duration = float(truth_metadata.get("scenario_duration", SCENARIO_DURATION))
    if not np.isclose(nmpc_duration, truth_duration):
        raise ValueError(f"Scenario duration differs: NMPC={nmpc_duration}, truth={truth_duration}.")


def _quaternion_to_rpy(quaternion_wxyz):
    quaternion_wxyz = np.asarray(quaternion_wxyz, dtype=float)
    norms = np.linalg.norm(quaternion_wxyz, axis=1, keepdims=True)
    quaternion_wxyz = quaternion_wxyz / np.maximum(norms, np.finfo(float).eps)
    qw, qx, qy, qz = quaternion_wxyz.T

    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx**2 + qy**2))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
    return np.column_stack((roll, pitch, yaw))


def _interpolate_columns(time_source, values_source, time_target):
    return np.column_stack(
        [np.interp(time_target, time_source, values_source[:, column]) for column in range(values_source.shape[1])]
    )


def _subtract_position_baseline(time, state):
    state = state.copy()
    baseline_mask = (time >= BASELINE_WINDOW[0]) & (time < BASELINE_WINDOW[1])
    if not np.any(baseline_mask):
        raise ValueError(f"No samples in baseline window {BASELINE_WINDOW}.")
    state[:, :3] -= np.mean(state[baseline_mask, :3], axis=0)
    return state


def _get_plot_state(data):
    return data["state_plot"] if "state_plot" in data.files else data["state_ee"]


def _calculate_metrics(time, nmpc_state, truth_state):
    rows = []
    comparison_mask = (time >= 2.0) & (time <= SCENARIO_DURATION)

    for quantity, offset, unit in (("position", 0, "m"), ("velocity", 3, "m/s")):
        for axis_index, axis_name in enumerate(AXES):
            error = nmpc_state[:, offset + axis_index] - truth_state[:, offset + axis_index]
            active_error = error[comparison_mask]
            row = {
                "quantity": quantity,
                "axis": axis_name,
                "unit": unit,
                "rmse": float(np.sqrt(np.mean(active_error**2))),
                "max_abs_error": float(np.max(np.abs(active_error))),
            }
            for window_index, (start, stop) in enumerate(STEADY_STATE_WINDOWS, start=1):
                window_mask = (time >= start) & (time <= stop)
                row[f"steady_mae_{window_index}"] = float(np.mean(np.abs(error[window_mask])))
            rows.append(row)

    return rows


def _write_metrics(path, rows):
    fieldnames = [
        "quantity",
        "axis",
        "unit",
        "rmse",
        "max_abs_error",
        *[f"steady_mae_{index}" for index in range(1, len(STEADY_STATE_WINDOWS) + 1)],
    ]
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _plot(
    output_prefix,
    nmpc_data,
    truth_time,
    time_nmpc,
    state_nmpc,
    state_truth,
    plot_state_frame,
    run_label,
):
    _configure_plot_style()
    figure = plt.figure(figsize=(12, 12), constrained_layout=True)
    grid = figure.add_gridspec(5, 2)

    force_axis = figure.add_subplot(grid[0, 0])
    torque_axis = figure.add_subplot(grid[0, 1])
    force_time = nmpc_data["time_input"]
    wrench_key = "applied_wrench_at_point" if "applied_wrench_at_point" in nmpc_data.files else "applied_wrench_ee"
    applied_force = nmpc_data[wrench_key][:, :3]
    if "applied_wrench_cog" in nmpc_data.files:
        lever_arm_torque = nmpc_data["applied_wrench_cog"][:, 3:6]
    elif "torque_compensation_b" in nmpc_data.files:
        lever_arm_torque = nmpc_data["torque_compensation_b"]
    else:
        raise ValueError("NMPC bundle has neither applied_wrench_cog nor torque_compensation_b.")

    for index, axis_name in enumerate(AXES):
        force_axis.step(
            force_time,
            applied_force[:, index],
            where="post",
            label=rf"$f_{axis_name}$",
        )
        torque_axis.plot(
            force_time,
            lever_arm_torque[:, index],
            label=rf"$\tau_{axis_name}$",
        )
    force_axis.set_ylabel("Applied $^W\\boldsymbol{f}_{T_o}$ [N]")
    torque_axis.set_ylabel("Lever-arm $^B\\boldsymbol{\\tau}_{B_o}$ [N$\cdot$m]")
    force_axis.legend(ncol=3)
    torque_axis.legend(ncol=3)

    frame_symbol = FRAME_SYMBOLS.get(plot_state_frame.lower(), plot_state_frame.upper())
    for axis_index, axis_name in enumerate(AXES):
        position_axis = figure.add_subplot(grid[axis_index + 1, 0])
        velocity_axis = figure.add_subplot(grid[axis_index + 1, 1])

        position_axis.plot(
            truth_time,
            state_truth[:, axis_index],
            "k--",
            label="Nominal impedance",
        )
        position_axis.plot(time_nmpc, state_nmpc[:, axis_index], label="Force-impedance NMPC")
        position_axis.set_ylabel(rf"$^W p_{{{frame_symbol}_o,{axis_name}}}$ [m]")

        velocity_axis.plot(
            truth_time,
            state_truth[:, axis_index + 3],
            "k--",
            label="Nominal impedance",
        )
        velocity_axis.plot(time_nmpc, state_nmpc[:, axis_index + 3], label="Force-impedance NMPC")
        velocity_axis.set_ylabel(rf"$^W v_{{{frame_symbol}_o,{axis_name}}}$ [m/s]")

        if axis_index == 0:
            position_axis.legend()
            velocity_axis.legend()

    if "state_raw" not in nmpc_data.files:
        raise ValueError("NMPC bundle is missing state_raw for attitude and angular-velocity plots.")
    state_raw = nmpc_data["state_raw"]
    rpy_deg = np.rad2deg(_quaternion_to_rpy(state_raw[:, 6:10]))
    omega_b = state_raw[:, 10:13]

    attitude_axis = figure.add_subplot(grid[4, 0])
    angular_velocity_axis = figure.add_subplot(grid[4, 1])
    for axis_index, axis_name in enumerate(AXES):
        attitude_axis.plot(
            time_nmpc,
            rpy_deg[:, axis_index],
            label=(r"$\phi$", r"$\theta$", r"$\psi$")[axis_index],
        )
        angular_velocity_axis.plot(
            time_nmpc,
            omega_b[:, axis_index],
            label=rf"$\omega_{axis_name}$",
        )

    attitude_axis.set_ylabel(r"Attitude $^W\boldsymbol{\Theta}_{B_o}$ [deg]")
    angular_velocity_axis.set_ylabel(r"Angular velocity $^B\boldsymbol{\omega}_{B_o}$ [rad/s]")
    attitude_axis.set_xlabel("Time [s]")
    angular_velocity_axis.set_xlabel("Time [s]")
    attitude_axis.legend(ncol=3)
    angular_velocity_axis.legend(ncol=3)

    for axis in figure.axes:
        axis.set_xlim(0.0, SCENARIO_DURATION)

    if run_label:
        figure.suptitle(run_label)
    for extension in ("png", "pdf"):
        figure.savefig(f"{output_prefix}.{extension}", dpi=300)
    plt.close(figure)


def _plot_rotational_diagnostics(output_prefix, nmpc_data, metadata, run_label):
    _configure_plot_style()
    required = ("state_raw", "torque_compensation_b")
    missing = [name for name in required if name not in nmpc_data.files]
    if missing:
        raise ValueError(f"NMPC bundle is missing diagnostic arrays: {', '.join(missing)}")

    state_time = nmpc_data["time_state"]
    input_time = nmpc_data["time_input"]
    state_raw = nmpc_data["state_raw"]
    rpy_deg = np.rad2deg(_quaternion_to_rpy(state_raw[:, 6:10]))
    omega_b = state_raw[:, 10:13]
    wrench_key = "applied_wrench_at_point" if "applied_wrench_at_point" in nmpc_data.files else "applied_wrench_ee"
    applied_force_w = nmpc_data[wrench_key][:, :3]
    lever_arm_torque_b = nmpc_data["torque_compensation_b"]

    figure, axes = plt.subplots(4, 1, figsize=(8, 7), sharex=True, constrained_layout=True)
    for index, axis_name in enumerate(AXES):
        axes[0].step(input_time, applied_force_w[:, index], where="post", label=f"$f_{axis_name}^W$")
        axes[1].plot(state_time, rpy_deg[:, index], label=("roll", "pitch", "yaw")[index])
        axes[2].plot(state_time, omega_b[:, index], label=f"$\\omega_{axis_name}^B$")
        axes[3].plot(input_time, lever_arm_torque_b[:, index], label=f"$\\tau_{axis_name}^B$")

    axes[0].set_ylabel("EE force [N]")
    axes[1].set_ylabel("Body attitude [deg]")
    axes[2].set_ylabel("Angular velocity [rad/s]")
    axes[3].set_ylabel("Lever-arm torque [N m]")
    axes[3].set_xlabel("Time [s]")
    axes[3].set_xlim(0.0, SCENARIO_DURATION)

    for axis in axes:
        for event_time in SCENARIO_EVENT_TIMES:
            axis.axvline(event_time, color="0.65", linewidth=0.8, linestyle=":")
        axis.grid(True, alpha=0.3)
        axis.legend(ncol=3, loc="best")

    acceleration_mode = metadata.get("ee_acceleration", "unknown")
    controller_frame = metadata.get("controller_state_frame", "ee")
    wrench_point = metadata.get("wrench_application_point", metadata.get("interaction_frame", "ee"))
    plot_frame = metadata.get("plot_state_frame", metadata.get("interaction_frame", "ee"))
    title = (
        "Force-attitude coupling diagnostics\n"
        f"controller: {controller_frame.upper()}, load: {wrench_point.upper()}, "
        f"plot: {plot_frame.upper()}, acceleration: {acceleration_mode}"
    )
    if run_label:
        title += f"\n{run_label}"
    figure.suptitle(title)
    diagnostic_prefix = f"{output_prefix}_rotational_diagnostics"
    for extension in ("png", "pdf"):
        figure.savefig(f"{diagnostic_prefix}.{extension}", dpi=200)
    plt.close(figure)
    return diagnostic_prefix


def main(args):
    nmpc_data, nmpc_metadata = load_run_bundle(args.nmpc)
    truth_data, truth_metadata = load_run_bundle(args.truth)
    try:
        _validate_bundle(args.nmpc, nmpc_data, nmpc_metadata)
        _validate_bundle(args.truth, truth_data, truth_metadata)
        _validate_impedance_match(nmpc_metadata, truth_metadata)

        time_nmpc = nmpc_data["time_state"]
        truth_time = truth_data["time_state"]
        plot_state_frame = nmpc_metadata.get("plot_state_frame", nmpc_metadata.get("interaction_frame", "ee"))
        state_nmpc = _subtract_position_baseline(time_nmpc, _get_plot_state(nmpc_data)[:, :6])
        state_truth_raw = _subtract_position_baseline(truth_time, _get_plot_state(truth_data)[:, :6])
        state_truth = _interpolate_columns(truth_time, state_truth_raw, time_nmpc)

        comparison_name = os.path.splitext(os.path.basename(args.nmpc))[0] + "_vs_nominal"
        if args.output_prefix is None:
            args.output_prefix = os.path.join(PAPER_RESULTS_ROOT, "figures", comparison_name)
        args.output_prefix = os.path.abspath(args.output_prefix)
        if args.metrics_path is None:
            if args.output_prefix.startswith(os.path.join(PAPER_RESULTS_ROOT, "figures")):
                args.metrics_path = os.path.join(PAPER_RESULTS_ROOT, "metrics", comparison_name + ".csv")
            else:
                args.metrics_path = f"{args.output_prefix}_metrics.csv"

        args.metrics_path = os.path.abspath(args.metrics_path)
        output_directory = os.path.dirname(args.output_prefix)
        os.makedirs(output_directory, exist_ok=True)
        os.makedirs(os.path.dirname(args.metrics_path), exist_ok=True)
        metrics = _calculate_metrics(time_nmpc, state_nmpc, state_truth)
        metrics_path = args.metrics_path
        _write_metrics(metrics_path, metrics)
        _plot(
            args.output_prefix,
            nmpc_data,
            truth_time,
            time_nmpc,
            state_nmpc,
            state_truth_raw,
            plot_state_frame,
            args.run_label,
        )
        diagnostic_prefix = None
        if args.rotational_diagnostics:
            diagnostic_prefix = _plot_rotational_diagnostics(
                args.output_prefix,
                nmpc_data,
                nmpc_metadata,
                args.run_label,
            )
    finally:
        nmpc_data.close()
        truth_data.close()

    print(f"Comparison figure saved to {args.output_prefix}.png and {args.output_prefix}.pdf")
    if diagnostic_prefix is not None:
        print(f"Rotational diagnostics saved to {diagnostic_prefix}.png and {diagnostic_prefix}.pdf")
    print(f"Metrics saved to {metrics_path}")
    for row in metrics:
        print(
            f"{row['quantity']:8s} {row['axis']}: "
            f"RMSE={row['rmse']:.6g} {row['unit']}, max={row['max_abs_error']:.6g} {row['unit']}"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compare NMPC force impedance against the ideal second-order model.")
    parser.add_argument("--nmpc", required=True, help="NMPC run bundle generated by sim_ee_force_impedance_nmpc.py.")
    parser.add_argument("--truth", required=True, help="Ideal run bundle generated by sim_impedance_only.py.")
    parser.add_argument(
        "--output-prefix",
        default=None,
        help="Output path without an extension. Default: organized paper figures directory.",
    )
    parser.add_argument(
        "--metrics-path",
        default=None,
        help="CSV metrics path. Default: organized paper metrics directory.",
    )
    parser.add_argument("--run-label", default="", help="Optional LaTeX-compatible figure title.")
    parser.add_argument(
        "--rotational-diagnostics",
        action="store_true",
        help="Also generate the legacy attitude/angular-velocity diagnostics figure.",
    )
    main(parser.parse_args())
