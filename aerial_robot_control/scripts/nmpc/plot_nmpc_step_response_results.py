#!/usr/bin/env python3
"""Compute CSV metrics and response figures for a step-response suite."""

import argparse
import csv
import json
import os

import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401  Register the SciencePlots style sheets.
import transformations as tf
from matplotlib.lines import Line2D

from nmpc_tilt_mt.utils.step_response_experiment import (
    attitude_error_vectors,
    compute_run_metrics,
    load_run_bundle,
)


def flatten(prefix, value, output):
    if isinstance(value, dict):
        for key, child in value.items():
            flatten(f"{prefix}_{key}" if prefix else key, child, output)
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            flatten(f"{prefix}_{index}", child, output)
    else:
        output[prefix] = value


def plot_case(path, output_prefix):
    data, metadata = load_run_bundle(path)
    time = data["time_state"]
    state = data["state_plant"]
    position_ref = data["reference_position"]
    quaternion_ref = data["reference_quaternion"]
    attitude_error, geodesic = attitude_error_vectors(state[:, 6:10], quaternion_ref)
    rpy = np.asarray([tf.euler_from_quaternion(q, axes="sxyz") for q in state[:, 6:10]])
    rpy_ref = data["reference_rpy"]
    time_input = data["time_input"]
    control = data["control_applied"]

    fig, axes = plt.subplots(5, 1, figsize=(9, 14), sharex=True)
    for index, label in enumerate(("x", "y", "z")):
        axes[0].plot(time, state[:, index], label=label)
        axes[0].plot(time, position_ref[:, index], "--", linewidth=1)
    axes[0].set_ylabel("Position (m)")
    axes[0].legend(ncol=3)
    for index, label in enumerate(("roll", "pitch", "yaw")):
        axes[1].plot(time, np.degrees(rpy[:, index]), label=label)
        axes[1].plot(time, np.degrees(rpy_ref[:, index]), "--", linewidth=1)
    axes[1].set_ylabel("Attitude (deg)")
    axes[1].legend(ncol=3)
    axes[2].plot(time, np.degrees(attitude_error))
    axes[2].plot(time, np.degrees(geodesic), "k", linewidth=1.5, label="geodesic")
    axes[2].set_ylabel("SO(3) error (deg)")
    axes[2].legend()
    axes[3].plot(time_input, control[:, :4])
    axes[3].set_ylabel("Thrust command (N)")
    axes[4].plot(time_input, np.degrees(control[:, 4:8]))
    axes[4].set_ylabel("Servo command (deg)")
    axes[4].set_xlabel("Time (s)")
    for axis in axes:
        axis.axvline(metadata["timing"]["step_time_s"], color="0.3", linestyle=":")
        axis.grid(True)
    fig.suptitle(metadata["case_id"])
    fig.tight_layout()
    fig.savefig(output_prefix + ".png", dpi=180)
    fig.savefig(output_prefix + ".pdf")
    plt.close(fig)


def configure_paper_plot_style():
    """Apply the SciencePlots-based style used for paper summary figures."""
    plt.style.use(["science", "grid", "no-latex"])
    plt.rcParams.update(
        {
            "font.size": 14,
            "axes.labelsize": 16,
            "axes.titlesize": 16,
            "xtick.labelsize": 14,
            "ytick.labelsize": 14,
            "legend.fontsize": 14,
            "lines.linewidth": 1.8,
            "lines.markersize": 6.5,
            "axes.linewidth": 1.0,
            "grid.alpha": 0.3,
            "grid.linewidth": 0.5,
            "savefig.dpi": 300,
        }
    )


def plot_metric_summary(metrics, output_prefix, position):
    selected = [
        item
        for item in metrics
        if item["completed"] and item["pre_step_stable"] and (item["axis"] in ("x", "y", "z")) == position
    ]
    if not selected:
        return
    fields = (
        ("rise_time_s", "Rise Time", r"Rise time [s]", 1.0),
        ("settling_time_s", "Settling Time", r"Settling time [s]", 1.0),
        ("percentage_overshoot_pct", "Percentage Overshoot", r"Overshoot [%]", 1.0),
        (
            "rmse",
            "Root-Mean-Square Error",
            r"RMSE [m]" if position else r"RMSE [$^\circ$]",
            1.0 if position else 180.0 / np.pi,
        ),
    )
    configure_paper_plot_style()
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 8.0))
    groups = {}
    for item in selected:
        workpoint = tuple(item["workpoint_rpy_deg"])
        groups.setdefault((item["axis"], workpoint), []).append(item)

    axis_order = ("x", "y", "z") if position else ("roll", "pitch", "yaw")
    axis_labels = (
        {"x": r"$x$", "y": r"$y$", "z": r"$z$"}
        if position
        else {"roll": r"Roll $\phi$", "pitch": r"Pitch $\theta$", "yaw": r"Yaw $\psi$"}
    )
    workpoints = sorted({workpoint for _, workpoint in groups})
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"][: len(axis_order)]
    line_styles = ("-", "--", "-.")
    markers = ("o", "s", "^")
    color_by_axis = dict(zip(axis_order, colors))
    style_by_workpoint = {workpoint: (line_styles[index], markers[index]) for index, workpoint in enumerate(workpoints)}

    for axis_name in axis_order:
        for workpoint in workpoints:
            group = groups.get((axis_name, workpoint))
            if not group:
                continue
            group.sort(key=lambda item: item["amplitude"])
            amplitudes = [item["amplitude"] for item in group]
            line_style, marker = style_by_workpoint[workpoint]
            for plot_axis, (field, _, _, scale) in zip(axes.flat, fields):
                values = [item["tracking"][field] * scale for item in group]
                plot_axis.plot(
                    amplitudes,
                    values,
                    color=color_by_axis[axis_name],
                    linestyle=line_style,
                    marker=marker,
                    markeredgewidth=0.7,
                    markerfacecolor="white",
                )

    amplitudes = sorted({item["amplitude"] for item in selected})
    xlabel = (
        r"Position-step amplitude $\Delta p$ [m]" if position else r"Attitude-step amplitude $\Delta \theta$ [$^\circ$]"
    )
    for panel_index, (plot_axis, (_, title, ylabel, _)) in enumerate(zip(axes.flat, fields)):
        plot_axis.set_title(f"({chr(ord('a') + panel_index)}) {title}", pad=7)
        plot_axis.set_xlabel(xlabel)
        plot_axis.set_ylabel(ylabel)
        plot_axis.set_xticks(amplitudes)
        plot_axis.margins(x=0.06)
        plot_axis.grid(True, which="major", alpha=0.3, linewidth=0.5)

    axis_handles = [
        Line2D([], [], color=color_by_axis[axis_name], linewidth=2.2, label=axis_labels[axis_name])
        for axis_name in axis_order
    ]
    workpoint_handles = []
    for workpoint in workpoints:
        line_style, marker = style_by_workpoint[workpoint]
        workpoint_text = ", ".join(rf"{value:g}^\circ" for value in workpoint)
        workpoint_handles.append(
            Line2D(
                [],
                [],
                color="0.2",
                linestyle=line_style,
                marker=marker,
                markerfacecolor="white",
                markeredgewidth=0.7,
                label=rf"WP $({workpoint_text})$",
            )
        )
    legend_handles = axis_handles + workpoint_handles
    fig.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=3,
        frameon=False,
        columnspacing=1.8,
        handlelength=2.6,
    )
    fig.align_ylabels(axes[:, 0])
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.9), pad=0.8, w_pad=1.0, h_pad=1.0)
    fig.savefig(output_prefix + ".png", dpi=300, bbox_inches="tight", pad_inches=0.04)
    fig.savefig(output_prefix + ".pdf", bbox_inches="tight", pad_inches=0.04)
    plt.close(fig)


def main(args):
    manifest_path = os.path.abspath(args.manifest)
    with open(manifest_path) as stream:
        manifest = json.load(stream)
    output_dir = os.path.abspath(args.output_dir or os.path.join(os.path.dirname(manifest_path), "analysis"))
    if os.path.exists(output_dir) and os.listdir(output_dir):
        raise FileExistsError(f"Refusing to overwrite non-empty analysis directory: {output_dir}")
    os.makedirs(output_dir, exist_ok=True)
    plot_dir = os.path.join(output_dir, "cases")
    if not args.summary_only:
        os.makedirs(plot_dir, exist_ok=True)

    rows = []
    detailed = []
    reference_signature = None
    for entry in manifest["cases"]:
        if not entry.get("success") or not entry.get("run"):
            continue
        data, metadata = load_run_bundle(entry["run"])
        signature = (
            metadata["controller_model"],
            metadata["plant_model"],
            metadata["timing"]["controller_period_s"],
            metadata["timing"]["simulation_period_s"],
            json.dumps(metadata["controller_parameters"], sort_keys=True),
        )
        if reference_signature is None:
            reference_signature = signature
        elif signature != reference_signature:
            raise ValueError(f"Configuration mismatch in {entry['run']}")
        metrics = compute_run_metrics(data, metadata)
        detailed.append(metrics)
        row = {}
        flatten("", metrics, row)
        rows.append(row)
        if not args.summary_only:
            plot_case(entry["run"], os.path.join(plot_dir, metadata["case_id"]))

    if not rows:
        raise ValueError("The manifest contains no successful runs.")
    fieldnames = sorted({key for row in rows for key in row})
    with open(os.path.join(output_dir, "step_response_metrics.csv"), "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    with open(os.path.join(output_dir, "step_response_metrics.json"), "w") as stream:
        json.dump(detailed, stream, indent=2, sort_keys=True, allow_nan=True)
    plot_metric_summary(detailed, os.path.join(output_dir, "position_metric_summary"), position=True)
    plot_metric_summary(detailed, os.path.join(output_dir, "attitude_metric_summary"), position=False)
    print(f"Analyzed {len(rows)} runs into {output_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, help="Manifest written by run_nmpc_step_response_suite.py.")
    parser.add_argument("--output-dir", default=None, help="New analysis output directory.")
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="Generate metrics and summary figures without the per-case response figures.",
    )
    main(parser.parse_args())
