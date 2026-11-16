#!/usr/bin/env python3
"""
Plot violin distributions of trajectory-tracking errors for two cases:
1) effector-centric formulation inside NMPC
2) effector-centric conversion outside NMPC / reference generation

Usage:
    python plot_effector_centric_violin.py inside_nmpc.csv outside_nmpc.csv
    python plot_effector_centric_violin.py inside_nmpc.csv outside_nmpc.csv -o violin.pdf
"""

import argparse
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


TIME = "__time"

X_REAL = "/beetle1/uav/ee_contact/odom/pose/pose/position/x"
X_REF = "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x"

Q_REAL_PREFIX = "/beetle1/uav/ee_contact/odom/pose/pose/orientation"
Q_REF_PREFIX = "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation"


@dataclass
class ErrorData:
    x_abs_error: np.ndarray  # [m]
    pitch_abs_error_deg: np.ndarray  # [deg]


def qcols(prefix: str) -> Tuple[str, str, str, str]:
    return (
        f"{prefix}/w",
        f"{prefix}/x",
        f"{prefix}/y",
        f"{prefix}/z",
    )


def require_columns(data: pd.DataFrame, columns) -> None:
    missing = [c for c in columns if c not in data.columns]
    if missing:
        raise KeyError("Missing required columns:\n" + "\n".join(missing))


def optional_column_group_exists(data: pd.DataFrame, columns, group_name: str) -> bool:
    present = [column in data.columns for column in columns]
    if any(present) and not all(present):
        missing = [column for column, exists in zip(columns, present) if not exists]
        raise KeyError(f"Incomplete {group_name} columns:\n" + "\n".join(missing))
    return all(present)


def normalize_quat(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q, axis=1, keepdims=True)
    norm[norm == 0.0] = 1.0
    return q / norm


def continuous_pitch_from_quat(qw: np.ndarray, qx: np.ndarray, qy: np.ndarray, qz: np.ndarray) -> np.ndarray:
    """
    Extract a continuous pitch-like angle for a trajectory mainly rotating about the body/world Y axis.

    For a pure pitch rotation, q = [cos(theta/2), 0, sin(theta/2), 0], so
    theta = 2 atan2(qy, qw). This avoids the +/-90 deg limitation of standard ZYX Euler pitch.
    Small roll/yaw components are ignored here, which is usually acceptable for a pitch-axis tracking test.
    """
    q = normalize_quat(np.column_stack([qw, qx, qy, qz]))
    pitch = 2.0 * np.arctan2(q[:, 2], q[:, 0])
    return np.unwrap(pitch)


def load_error_data(file_path: str) -> ErrorData:
    data = pd.read_csv(file_path)

    real_q_cols = qcols(Q_REAL_PREFIX)
    ref_q_cols = qcols(Q_REF_PREFIX)
    require_columns(data, [TIME, X_REAL, *real_q_cols])

    has_x_ref = X_REF in data.columns
    has_q_ref = optional_column_group_exists(data, ref_q_cols, "reference quaternion")

    data_x_real = data[[TIME, X_REAL]].dropna().sort_values(TIME)
    data_q_real = data[[TIME, *real_q_cols]].dropna().sort_values(TIME)

    data_x_ref = data[[TIME, X_REF]].dropna().sort_values(TIME) if has_x_ref else None
    data_q_ref = data[[TIME, *ref_q_cols]].dropna().sort_values(TIME) if has_q_ref else None

    # Use reference timestamps as the common grid, as in the original type==0 workflow.
    time_series = [data_x_real[TIME], data_q_real[TIME]]
    if data_x_ref is not None:
        time_series.append(data_x_ref[TIME])
    if data_q_ref is not None:
        time_series.append(data_q_ref[TIME])
    t_start = max(series.iloc[0] for series in time_series)
    t_stop = min(series.iloc[-1] for series in time_series)

    # Without X_REF, use X_REAL timestamps and a constant reference equal to
    # the mean X_REAL value during its first 0.5 seconds.
    t_ref = (data_x_ref if data_x_ref is not None else data_x_real)[TIME].to_numpy()
    valid = (t_ref >= t_start) & (t_ref <= t_stop)
    t_ref = t_ref[valid]

    if data_x_ref is not None:
        x_ref = data_x_ref[X_REF].to_numpy()[valid]
    else:
        x_start = data_x_real[TIME].iloc[0]
        initial_x = data_x_real.loc[data_x_real[TIME] <= x_start + 0.5, X_REAL]
        x_ref = np.full(t_ref.shape, initial_x.mean())
    x_real = np.interp(t_ref, data_x_real[TIME].to_numpy(), data_x_real[X_REAL].to_numpy())
    x_abs_error = np.abs(x_real - x_ref)

    pitch_real = continuous_pitch_from_quat(
        data_q_real[real_q_cols[0]].to_numpy(),
        data_q_real[real_q_cols[1]].to_numpy(),
        data_q_real[real_q_cols[2]].to_numpy(),
        data_q_real[real_q_cols[3]].to_numpy(),
    )
    pitch_real_interp = np.interp(t_ref, data_q_real[TIME].to_numpy(), pitch_real)
    if data_q_ref is not None:
        pitch_ref_raw = continuous_pitch_from_quat(
            data_q_ref[ref_q_cols[0]].to_numpy(),
            data_q_ref[ref_q_cols[1]].to_numpy(),
            data_q_ref[ref_q_cols[2]].to_numpy(),
            data_q_ref[ref_q_cols[3]].to_numpy(),
        )
        pitch_ref = np.interp(t_ref, data_q_ref[TIME].to_numpy(), pitch_ref_raw)
    else:
        pitch_ref = np.zeros_like(t_ref)
    pitch_abs_error_deg = np.abs(pitch_real_interp - pitch_ref) * 180.0 / np.pi

    return ErrorData(
        x_abs_error=remove_invalid(x_abs_error),
        pitch_abs_error_deg=remove_invalid(pitch_abs_error_deg),
    )


def remove_invalid(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    return values[np.isfinite(values)]


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def add_violin(ax, values, labels, ylabel: str, colors: Optional[list] = None) -> None:
    parts = ax.violinplot(values, positions=np.arange(1, len(values) + 1), showmeans=True, showextrema=True)

    # Change colors for violin bodies
    if colors:
        for i, body in enumerate(parts["bodies"]):
            if i < len(colors):
                body.set_facecolor(colors[i])
            body.set_alpha(0.65)
    else:
        for body in parts["bodies"]:
            body.set_alpha(0.65)

    ax.set_xticks(np.arange(1, len(labels) + 1))
    ax.set_xticklabels(labels)
    ax.set_ylabel(ylabel, fontsize=14)
    ax.grid(True, axis="y", alpha=0.35)


def plot_errors(errors: Dict[str, ErrorData], output_path: str | None) -> None:
    try:
        import scienceplots  # noqa: F401

        plt.style.use(["science", "grid"])
    except Exception:
        pass

    plt.rcParams.update({"font.size": 11})

    labels = list(errors.keys())
    x_errors = [errors[label].x_abs_error for label in labels]
    pitch_errors = [errors[label].pitch_abs_error_deg for label in labels]

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.2))

    # Colors: blue for CoG (left), orange for EE (right)
    colors_x = ["#0072BD", "#FF9900"]
    colors_pitch = ["#0072BD", "#FF9900"]

    add_violin(axes[0], x_errors, labels, r"$|e_x|$ [m]", colors_x)
    add_violin(axes[1], pitch_errors, labels, r"$|e_{\mathrm{pitch}}|$ [$^\circ$]", colors_pitch)

    fig.tight_layout()

    if output_path:
        fig.savefig(output_path, bbox_inches="tight")
    else:
        plt.show()


def print_summary(name: str, err: ErrorData) -> None:
    print(f"[{name}]")
    print(f"  X abs. error:     mean = {np.mean(err.x_abs_error):.5f} m, RMSE = {rmse(err.x_abs_error):.5f} m")
    print(
        f"  Pitch abs. error: mean = {np.mean(err.pitch_abs_error_deg):.3f} deg, "
        f"RMSE = {rmse(err.pitch_abs_error_deg):.3f} deg"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot violin distributions of X and pitch tracking errors.")
    parser.add_argument(
        "inside_nmpc_file_path", type=str, help="CSV file for effector-centric formulation inside NMPC."
    )
    parser.add_argument(
        "outside_nmpc_file_path", type=str, help="CSV file for effector-centric conversion outside NMPC."
    )
    parser.add_argument("-o", "--output", type=str, default=None, help="Optional output figure path, e.g., violin.pdf.")
    args = parser.parse_args()

    errors = {
        "CoG-Centric": load_error_data(args.outside_nmpc_file_path),
        "EE-Centric": load_error_data(args.inside_nmpc_file_path),
    }

    for name, err in errors.items():
        print_summary(name, err)

    plot_errors(errors, args.output)


if __name__ == "__main__":
    main()
