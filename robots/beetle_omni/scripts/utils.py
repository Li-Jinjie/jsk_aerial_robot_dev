"""
 Created by li-jinjie on 25-7-20.
"""
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

matlab_blue = "#0072BD"
matlab_orange = "#D95319"
matlab_yellow = "#EDB120"
matlab_green = "#77AC30"
matlab_purple = "#7E2F8E"


def unwrap_angle_sequence(angle_seq: np.ndarray) -> np.ndarray:
    angle_seq = angle_seq.copy()  # avoid modifying the input array
    for i in range(1, len(angle_seq)):
        delta = angle_seq[i] - angle_seq[i - 1]
        if delta > np.pi:
            angle_seq[i:] -= 2 * np.pi
        elif delta < -np.pi:
            angle_seq[i:] += 2 * np.pi
    return angle_seq


def calculate_rmse(t, x, t_ref, x_ref, is_yaw=False):
    x_ref_interp = np.interp(t, t_ref, x_ref)
    if is_yaw:
        # calculate the RMSE for yaw
        error = np.minimum(np.abs(x - x_ref_interp), 2 * np.pi - np.abs(x - x_ref_interp))
    else:
        error = x - x_ref_interp

    rmse_x = np.sqrt(np.mean(error**2))
    return rmse_x


def calculate_quat_error(
    qw: pd.Series,
    qx: pd.Series,
    qy: pd.Series,
    qz: pd.Series,
    qwr: pd.Series,
    qxr: pd.Series,
    qyr: pd.Series,
    qzr: pd.Series,
):
    """
    Quaternion tracking error e = q ⊗ qr⁻¹.

    Parameters
    ----------
    qw, qx, qy, qz : pandas.Series
        Actual quaternion components (scalar–vector order w, x, y, z).
    qwr, qxr, qyr, qzr : pandas.Series
        Reference quaternion components (same order, same index).

    Returns
    -------
    ew, ex, ey, ez : pandas.Series
        Error quaternion (w, x, y, z), indexed exactly like the inputs.
    """
    # 1. SciPy expects (x, y, z, w) → stack accordingly
    quat_act = np.column_stack([qx.to_numpy(), qy.to_numpy(), qz.to_numpy(), qw.to_numpy()])
    quat_ref = np.column_stack([qxr.to_numpy(), qyr.to_numpy(), qzr.to_numpy(), qwr.to_numpy()])

    # 2. Build Rotation objects
    rot_act = Rotation.from_quat(quat_act)
    rot_ref = Rotation.from_quat(quat_ref)

    # 3. Error rotation: qr⁻¹ ⊗ q  (actual minus reference)
    rot_err = rot_ref.inv() * rot_act

    # 4. Back to quaternion (SciPy returns x, y, z, w)
    quat_err = rot_err.as_quat()
    ex, ey, ez, ew = quat_err.T  # transpose to unpack

    # 5. Wrap as Series with the original index
    idx = qw.index
    ew = pd.Series(ew, index=idx, name="ew")
    ex = pd.Series(ex, index=idx, name="ex")
    ey = pd.Series(ey, index=idx, name="ey")
    ez = pd.Series(ez, index=idx, name="ez")
    return ew, ex, ey, ez


def quat2euler(qw: pd.Series, qx: pd.Series, qy: pd.Series, qz: pd.Series, sequence: str = "ZYX", degrees: bool = True):
    """
    Convert quaternion (w, x, y, z) series to Euler angles.
    NOTE: for euler angles, the order is ZYX by default, which corresponds to
    the common aerospace sequence (yaw, pitch, roll).
    """
    quat_array = np.column_stack([qx.to_numpy(), qy.to_numpy(), qz.to_numpy(), qw.to_numpy()])

    # Convert
    euler = Rotation.from_quat(quat_array).as_euler(sequence, degrees=degrees)

    # Wrap back into Series with the original time index
    idx = qw.index
    roll = pd.Series(euler[:, 0], index=idx, name="roll")
    pitch = pd.Series(euler[:, 1], index=idx, name="pitch")
    yaw = pd.Series(euler[:, 2], index=idx, name="yaw")
    return roll, pitch, yaw
