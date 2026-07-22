"""Utilities for the six-DoF NMPC step-response experiment."""

import json
import os
from dataclasses import dataclass

import numpy as np


SCENARIO_NAME = "step_response"
POSITION_AXES = ("x", "y", "z")
ATTITUDE_AXES = ("roll", "pitch", "yaw")
ALL_AXES = POSITION_AXES + ATTITUDE_AXES
POSITION_AMPLITUDES_M = (0.2, 0.4, 0.6, 0.8, 1.0)
ATTITUDE_AMPLITUDES_DEG = (10.0, 30.0, 50.0, 70.0, 90.0)
WORKPOINTS_RPY_DEG = ((0.0, 0.0, 0.0), (30.0, 0.0, 0.0), (30.0, 30.0, 0.0))


@dataclass(frozen=True)
class StepCase:
    axis: str
    amplitude: float
    workpoint_rpy_deg: tuple

    @property
    def amplitude_si(self):
        return self.amplitude if self.axis in POSITION_AXES else np.radians(self.amplitude)

    @property
    def amplitude_unit(self):
        return "m" if self.axis in POSITION_AXES else "deg"

    @property
    def slug(self):
        wp = "_".join(f"{name}{value:+05.1f}" for name, value in zip(("r", "p", "y"), self.workpoint_rpy_deg))
        unit = "m" if self.axis in POSITION_AXES else "deg"
        amp = f"{self.amplitude:+06.2f}".replace(".", "p")
        return f"wp_{wp}_step_{self.axis}_{amp}{unit}"


def base36_cases():
    """Return the former 36-case matrix for loading/reproducing archived runs."""
    position_amplitudes = (0.2, 0.5, 1.0)
    attitude_amplitudes = (10.0, 30.0, 60.0)
    cases = []
    zero = WORKPOINTS_RPY_DEG[0]
    for axis in POSITION_AXES:
        cases.extend(StepCase(axis, amplitude, zero) for amplitude in position_amplitudes)
    for axis in ATTITUDE_AXES:
        cases.extend(StepCase(axis, amplitude, zero) for amplitude in attitude_amplitudes)
    for workpoint in WORKPOINTS_RPY_DEG[1:]:
        for axis in POSITION_AXES:
            cases.extend(StepCase(axis, amplitude, workpoint) for amplitude in position_amplitudes)
    if len(cases) != 36 or len({case.slug for case in cases}) != 36:
        raise RuntimeError("The base step-response matrix must contain 36 unique cases.")
    return cases


def base90_cases():
    """Return five amplitudes on all six axes at all three attitude workpoints."""
    cases = []
    for workpoint in WORKPOINTS_RPY_DEG:
        for axis in POSITION_AXES:
            cases.extend(StepCase(axis, amplitude, workpoint) for amplitude in POSITION_AMPLITUDES_M)
        for axis in ATTITUDE_AXES:
            cases.extend(StepCase(axis, amplitude, workpoint) for amplitude in ATTITUDE_AMPLITUDES_DEG)
    if len(cases) != 90 or len({case.slug for case in cases}) != 90:
        raise RuntimeError("The expanded step-response matrix must contain 90 unique cases.")
    return cases


def save_run_bundle(path, metadata, **arrays):
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez_compressed(path, metadata=json.dumps(metadata, sort_keys=True), **arrays)
    return path


def load_run_bundle(path):
    data = np.load(path, allow_pickle=False)
    metadata = json.loads(str(data["metadata"].item()))
    return data, metadata


def quaternion_to_rotation_matrix(q):
    q = np.asarray(q, dtype=float)
    q = q / max(np.linalg.norm(q), 1e-12)
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def so3_log_vector(rotation):
    cos_angle = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = float(np.arccos(cos_angle))
    vee = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])
    if angle < 1e-8:
        return 0.5 * vee
    if np.pi - angle < 1e-5:
        eigvals, eigvecs = np.linalg.eigh((rotation + np.eye(3)) * 0.5)
        axis = eigvecs[:, np.argmax(eigvals)]
        if np.dot(axis, vee) < 0.0:
            axis = -axis
        return angle * axis
    return angle * vee / (2.0 * np.sin(angle))


def attitude_error_vectors(quaternions, reference_quaternions):
    """Return Log(R_ref.T R) and its geodesic norm for every sample."""
    values = []
    for q, q_ref in zip(quaternions, reference_quaternions):
        values.append(so3_log_vector(quaternion_to_rotation_matrix(q_ref).T @ quaternion_to_rotation_matrix(q)))
    vectors = np.asarray(values)
    return vectors, np.linalg.norm(vectors, axis=1)


def _first_crossing(time, progress, level):
    indices = np.flatnonzero(progress >= level)
    if not len(indices):
        return np.nan
    i = int(indices[0])
    if i == 0 or progress[i] == progress[i - 1]:
        return float(time[i])
    fraction = (level - progress[i - 1]) / (progress[i] - progress[i - 1])
    return float(time[i - 1] + fraction * (time[i] - time[i - 1]))


def scalar_step_metrics(time, response, reference, step_time, steady_window):
    """Compute conventional signed step-response metrics."""
    time = np.asarray(time)
    response = np.asarray(response)
    reference = np.asarray(reference)
    pre = time < step_time
    post = time >= step_time
    if not np.any(pre) or not np.any(post):
        raise ValueError("Step metrics require samples both before and after step_time.")
    y0 = float(np.mean(response[pre][-max(1, min(np.count_nonzero(pre), 100)) :]))
    r0 = float(reference[np.flatnonzero(pre)[-1]])
    r1 = float(reference[np.flatnonzero(post)[0]])
    delta = r1 - r0
    if abs(delta) < 1e-12:
        raise ValueError("The commanded step amplitude is zero.")
    t_post = time[post]
    progress = (response[post] - y0) / delta
    t10 = _first_crossing(t_post, progress, 0.1)
    t90 = _first_crossing(t_post, progress, 0.9)
    rise_time = t90 - t10 if np.isfinite(t10) and np.isfinite(t90) and t90 >= t10 else np.nan
    within = np.abs(response[post] - r1) <= 0.02 * abs(delta)
    settling_time = np.nan
    suffix_all = np.logical_and.accumulate(within[::-1])[::-1]
    settled = np.flatnonzero(suffix_all)
    if len(settled):
        settling_time = float(t_post[settled[0]] - step_time)
    overshoot = 100.0 * max(float(np.max(progress)) - 1.0, 0.0)
    error = reference[post] - response[post]
    steady = (time >= steady_window[0]) & (time <= steady_window[1])
    if not np.any(steady):
        raise ValueError("The steady-state window contains no samples.")
    steady_error = float(np.mean(reference[steady] - response[steady]))
    return {
        "rise_time_s": float(rise_time),
        "settling_time_s": float(settling_time),
        "percentage_overshoot_pct": overshoot,
        "steady_state_error": steady_error,
        "steady_state_error_abs": abs(steady_error),
        "rmse": float(np.sqrt(np.mean(error**2))),
        "iae": float(np.trapz(np.abs(error), time[post])),
    }


def minimum_margin_and_violations(values, lower, upper):
    values = np.asarray(values)
    lower = np.asarray(lower)
    upper = np.asarray(upper)
    margins = np.minimum(values - lower, upper - values)
    return {
        "minimum_margin": float(np.min(margins)),
        "violation_count": int(np.count_nonzero(margins < 0.0)),
        "violation_time_samples": int(np.count_nonzero(np.any(margins < 0.0, axis=1))),
    }


def compute_run_metrics(data, metadata):
    """Compute all paper-facing metrics from a saved step-response bundle."""
    if metadata.get("scenario") != SCENARIO_NAME:
        raise ValueError("Not a step-response run bundle.")
    time = np.asarray(data["time_state"])
    state = np.asarray(data["state_plant"])
    position_ref = np.asarray(data["reference_position"])
    quaternion_ref = np.asarray(data["reference_quaternion"])
    attitude_error, attitude_geodesic = attitude_error_vectors(state[:, 6:10], quaternion_ref)
    axis = metadata["step"]["axis"]
    step_time = float(metadata["timing"]["step_time_s"])
    steady_window = tuple(metadata["timing"]["steady_window_s"])
    if axis in POSITION_AXES:
        commanded_index = POSITION_AXES.index(axis)
        response = state[:, commanded_index]
        reference = position_ref[:, commanded_index]
        coupling = np.column_stack((np.delete(state[:, :3] - position_ref, commanded_index, axis=1), attitude_error))
        coupling_units = ["m", "m", "rad", "rad", "rad"]
    else:
        commanded_index = ATTITUDE_AXES.index(axis)
        pre_indices = np.flatnonzero(time < step_time)
        workpoint_quaternion = quaternion_ref[pre_indices[-1]]
        workpoint_quaternions = np.repeat(workpoint_quaternion[None, :], len(time), axis=0)
        attitude_response, _ = attitude_error_vectors(state[:, 6:10], workpoint_quaternions)
        response = attitude_response[:, commanded_index]
        reference = np.zeros_like(response)
        reference[time >= step_time] = float(metadata["step"]["amplitude_si"])
        coupling = np.column_stack((state[:, :3] - position_ref, np.delete(attitude_error, commanded_index, axis=1)))
        coupling_units = ["m", "m", "m", "rad", "rad"]
    primary = scalar_step_metrics(time, response, reference, step_time, steady_window)
    post = time >= step_time
    cross_axis = {
        "maximum_cross_axis_deviation": np.max(np.abs(coupling[post]), axis=0).tolist(),
        "cross_axis_rmse": np.sqrt(np.mean(coupling[post] ** 2, axis=0)).tolist(),
        "units": coupling_units,
        "geodesic_attitude_max_deg": float(np.degrees(np.max(attitude_geodesic[post]))),
        "geodesic_attitude_rmse_deg": float(np.degrees(np.sqrt(np.mean(attitude_geodesic[post] ** 2)))),
    }
    control = np.asarray(data["control_applied"])
    update = np.asarray(data["control_updated"], dtype=bool)
    control_time = np.asarray(data["time_input"])[update]
    control_updates = control[update]
    equilibrium = np.asarray(metadata["equilibrium"]["control"])
    du_dt = np.diff(control_updates, axis=0) / np.diff(control_time)[:, None]
    input_bounds = metadata["constraints"]["input"]
    lower_u = np.asarray(input_bounds["lower"])
    upper_u = np.asarray(input_bounds["upper"])
    tolerance = 0.01 * (upper_u - lower_u)
    saturated = (control_updates <= lower_u + tolerance) | (control_updates >= upper_u - tolerance)
    control_metrics = {
        "peak_control_input": np.max(np.abs(control_updates), axis=0).tolist(),
        "rms_control_effort": np.sqrt(np.mean(control_updates**2, axis=0)).tolist(),
        "rms_control_deviation": np.sqrt(np.mean((control_updates - equilibrium) ** 2, axis=0)).tolist(),
        "slew_rate_rms": np.sqrt(np.mean(du_dt**2, axis=0)).tolist(),
        "slew_rate_max": np.max(np.abs(du_dt), axis=0).tolist(),
        "saturation_time_ratio": np.mean(saturated, axis=0).tolist(),
    }
    state_bounds = metadata["constraints"]["state"]
    idx = np.asarray(state_bounds["indices"], dtype=int)
    constraints = {
        "state": minimum_margin_and_violations(state[:, idx], state_bounds["lower"], state_bounds["upper"]),
        "input": minimum_margin_and_violations(control_updates, lower_u, upper_u),
    }
    if state.shape[1] >= 21:
        constraints["plant_thrust_state"] = minimum_margin_and_violations(state[:, 17:21], lower_u[:4], upper_u[:4])
    wall = np.asarray(data["solve_wall_time"])
    acados = np.asarray(data["solve_acados_time_tot"])
    acados_valid = acados[np.isfinite(acados)]
    sample = acados_valid if len(acados_valid) else wall
    period = float(metadata["timing"]["controller_period_s"])
    timing = {
        "solution_time_source": "acados_time_tot" if len(acados_valid) else "wall_time",
        "mean_solution_time_s": float(np.mean(sample)),
        "p95_solution_time_s": float(np.percentile(sample, 95)),
        "worst_case_solution_time_s": float(np.max(sample)),
        "acados_deadline_miss_count": int(np.count_nonzero(acados_valid > period)),
        "wall_deadline_miss_count": int(np.count_nonzero(wall > period)),
        "solver_failure_count": int(np.count_nonzero(np.asarray(data["solver_status"]) != 0)),
    }
    return {
        "case_id": metadata["case_id"],
        "axis": axis,
        "amplitude": metadata["step"]["amplitude_input"],
        "amplitude_unit": metadata["step"]["amplitude_input_unit"],
        "workpoint_rpy_deg": metadata["workpoint_rpy_deg"],
        "pre_step_stable": metadata["validation"]["pre_step_stable"],
        "completed": metadata["validation"]["completed"],
        "tracking": primary,
        "cross_axis": cross_axis,
        "control": control_metrics,
        "constraints": constraints,
        "timing": timing,
    }
