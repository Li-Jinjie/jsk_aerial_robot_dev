import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse
from scipy.spatial.transform import Rotation, Slerp

from matplotlib.lines import lineStyles

from utils import unwrap_angle_sequence, calculate_rmse, quat2euler, calculate_quat_error, interp_quat
from utils import matlab_yellow, matlab_green, matlab_orange, matlab_blue

legend_alpha = 0.5


def rotate_body_vectors_to_world(vector_time, vectors_body, attitude_data, attitude_topic_prefix):
    """Rotate body-frame vectors into the world frame using interpolated attitude."""
    attitude_time = attitude_data["__time"].to_numpy()
    attitude_quat = attitude_data[
        [
            f"{attitude_topic_prefix}/x",
            f"{attitude_topic_prefix}/y",
            f"{attitude_topic_prefix}/z",
            f"{attitude_topic_prefix}/w",
        ]
    ].to_numpy()

    # Slerp requires strictly increasing timestamps. Keep the first attitude
    # sample at each timestamp and clamp wrench samples to the attitude range.
    attitude_time, unique_indices = np.unique(attitude_time, return_index=True)
    attitude_rotation = Rotation.from_quat(attitude_quat[unique_indices])
    if len(attitude_time) == 1:
        return attitude_rotation[0].apply(vectors_body)

    interpolation_time = np.clip(vector_time, attitude_time[0], attitude_time[-1])
    rotation_wb = Slerp(attitude_time, attitude_rotation)(interpolation_time)
    return rotation_wb.apply(vectors_body)


def main(file_path, type, if_push_slide):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    # ======= xyz =========
    data_xyz = data[
        [
            "__time",
            "/beetle1/uav/ee_contact/odom/pose/pose/position/x",
            "/beetle1/uav/ee_contact/odom/pose/pose/position/y",
            "/beetle1/uav/ee_contact/odom/pose/pose/position/z",
        ]
    ]

    data_xyz_cog = data[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/position/x",
            "/beetle1/uav/cog/odom/pose/pose/position/y",
            "/beetle1/uav/cog/odom/pose/pose/position/z",
        ]
    ]

    try:
        data_xyz_ref = data[
            [
                "__time",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_xyz_ref = pd.DataFrame()
        data_xyz_ref["__time"] = data_xyz["__time"]
        data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x"] = 0.0
        data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y"] = 0.0
        data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z"] = 1.0

    data_xyz_ref_nmpc = data[
        [
            "__time",
            "/beetle1/nmpc/viz_ref/poses[0]/position/x",
            "/beetle1/nmpc/viz_ref/poses[0]/position/y",
            "/beetle1/nmpc/viz_ref/poses[0]/position/z",
        ]
    ]

    data_xyz = data_xyz.dropna()
    data_xyz_ref = data_xyz_ref.dropna()
    data_xyz_cog = data_xyz_cog.dropna()
    data_xyz_ref_nmpc = data_xyz_ref_nmpc.dropna()

    # ======= rpy =========
    data_qwxyz = data[
        [
            "__time",
            "/beetle1/uav/ee_contact/odom/pose/pose/orientation/w",
            "/beetle1/uav/ee_contact/odom/pose/pose/orientation/x",
            "/beetle1/uav/ee_contact/odom/pose/pose/orientation/y",
            "/beetle1/uav/ee_contact/odom/pose/pose/orientation/z",
        ]
    ]

    data_qwxyz_cog = data[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/orientation/w",
            "/beetle1/uav/cog/odom/pose/pose/orientation/x",
            "/beetle1/uav/cog/odom/pose/pose/orientation/y",
            "/beetle1/uav/cog/odom/pose/pose/orientation/z",
        ]
    ]

    try:
        data_qwxyz_ref = data[
            [
                "__time",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_qwxyz_ref = pd.DataFrame()
        data_qwxyz_ref["__time"] = data_qwxyz["__time"]
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"] = 1.0
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"] = 0.0
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"] = 0.0
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"] = 0.0

    data_qwxyz_ref = data_qwxyz_ref.dropna()
    data_qwxyz = data_qwxyz.dropna()
    data_qwxyz_cog = data_qwxyz_cog.dropna()

    # convert to euler
    data_euler_ref = pd.DataFrame()
    data_euler_ref["__time"] = data_qwxyz_ref["__time"]
    data_euler_ref["roll"], data_euler_ref["pitch"], data_euler_ref["yaw"] = quat2euler(
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"],
        sequence="ZYX",
        degrees=False,
    )
    data_euler_ref["roll"] = unwrap_angle_sequence(data_euler_ref["roll"].to_numpy())
    data_euler_ref["pitch"] = unwrap_angle_sequence(data_euler_ref["pitch"].to_numpy())
    data_euler_ref["yaw"] = unwrap_angle_sequence(data_euler_ref["yaw"].to_numpy())

    # interpolate the real quaternion date
    t_ref = np.array(data_qwxyz_ref["__time"])
    t = np.array(data_qwxyz["__time"])

    data_qwxyz_interp = interp_quat(t_ref, t, data_qwxyz, "/beetle1/uav/ee_contact/odom/pose/pose/orientation")

    # calculate the quaternion error
    ew, ex, ey, ez = calculate_quat_error(
        data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/w"],
        data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/x"],
        data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/y"],
        data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/z"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"],
    )

    e_roll, e_pitch, e_yaw = quat2euler(ew, ex, ey, ez, sequence="ZYX", degrees=False)

    data_euler = pd.DataFrame()
    data_euler["__time"] = t_ref
    data_euler["roll"] = e_roll.to_numpy() + data_euler_ref["roll"].to_numpy()
    data_euler["pitch"] = e_pitch.to_numpy() + data_euler_ref["pitch"].to_numpy()
    data_euler["yaw"] = e_yaw.to_numpy() + data_euler_ref["yaw"].to_numpy()

    # cog euler
    t_cog = np.array(data_qwxyz_cog["__time"])
    data_qwxyz_cog_interp = interp_quat(t_ref, t_cog, data_qwxyz_cog, "/beetle1/uav/cog/odom/pose/pose/orientation")

    ew_cog, ex_cog, ey_cog, ez_cog = calculate_quat_error(
        data_qwxyz_cog_interp["/beetle1/uav/cog/odom/pose/pose/orientation/w"],
        data_qwxyz_cog_interp["/beetle1/uav/cog/odom/pose/pose/orientation/x"],
        data_qwxyz_cog_interp["/beetle1/uav/cog/odom/pose/pose/orientation/y"],
        data_qwxyz_cog_interp["/beetle1/uav/cog/odom/pose/pose/orientation/z"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"],
    )
    e_roll_cog, e_pitch_cog, e_yaw_cog = quat2euler(ew_cog, ex_cog, ey_cog, ez_cog, sequence="ZYX", degrees=False)

    data_euler_cog = pd.DataFrame()
    data_euler_cog["__time"] = t_ref
    data_euler_cog["roll"] = e_roll_cog.to_numpy() + data_euler_ref["roll"].to_numpy()
    data_euler_cog["pitch"] = e_pitch_cog.to_numpy() + data_euler_ref["pitch"].to_numpy()
    data_euler_cog["yaw"] = e_yaw_cog.to_numpy() + data_euler_ref["yaw"].to_numpy()

    # ======= actuators =========
    data_thrust_cmd = data[
        [
            "__time",
            "/beetle1/four_axes/command/base_thrust[0]",
            "/beetle1/four_axes/command/base_thrust[1]",
            "/beetle1/four_axes/command/base_thrust[2]",
            "/beetle1/four_axes/command/base_thrust[3]",
        ]
    ]
    data_thrust_cmd = data_thrust_cmd.dropna()

    data_servo_angle_cmd = data[
        [
            "__time",
            "/beetle1/gimbals_ctrl/gimbal1/position",
            "/beetle1/gimbals_ctrl/gimbal2/position",
            "/beetle1/gimbals_ctrl/gimbal3/position",
            "/beetle1/gimbals_ctrl/gimbal4/position",
        ]
    ]
    data_servo_angle_cmd = data_servo_angle_cmd.dropna()

    # # real servo angle
    # data_servo_angle = data[
    #     ['__time', '/beetle1/joint_states/gimbal1/position', '/beetle1/joint_states/gimbal2/position',
    #      '/beetle1/joint_states/gimbal3/position', '/beetle1/joint_states/gimbal4/position']]
    # data_servo_angle = data_servo_angle.dropna()

    # ======= est. wrench =========
    try:
        data_iterm = data[
            [
                "__time",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/force/x",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/force/y",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/force/z",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/x",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/y",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/z",
            ]
        ]
        data_iterm = data_iterm.dropna()

        data_ext_pure = data[
            [
                "__time",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/force/x",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/force/y",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/force/z",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/torque/x",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/torque/y",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/torque/z",
            ]
        ]
        data_ext_pure = data_ext_pure.dropna()

        data_ext_wrench_est = data[
            [
                "__time",
                "/beetle1/ext_wrench_est/value/wrench/force/x",
                "/beetle1/ext_wrench_est/value/wrench/force/y",
                "/beetle1/ext_wrench_est/value/wrench/force/z",
                "/beetle1/ext_wrench_est/value/wrench/torque/x",
                "/beetle1/ext_wrench_est/value/wrench/torque/y",
                "/beetle1/ext_wrench_est/value/wrench/torque/z",
            ]
        ]
        data_ext_wrench_est = data_ext_wrench_est.dropna()
    except KeyError:
        print("No est. wrench data found!")

    # ======= plotting =========
    if type == 0:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # default is 10
        label_size = 14

        fig = plt.figure(figsize=(12, 7))

        t_bias = max(data_xyz["__time"].iloc[0], data_xyz_ref_nmpc["__time"].iloc[0])
        color_ref = "#0C5DA5"
        color_real = "#FF2C00"
        color_cog = "#f29619"  # the orange in scienceplots

        # --------------------------------
        plt.subplot(4, 2, 1)

        # plot a curve has the same length with t and the value is always 1.1. Only for admittance.
        t = np.array(data_xyz["__time"]) - t_bias
        x_original_ref = np.zeros_like(t) + 1.1
        plt.plot(t, x_original_ref, label="$p_{x,r}$", linestyle=":", color=color_cog)

        t_ref = np.array(data_xyz_ref_nmpc["__time"]) - t_bias
        x_ref = np.array(data_xyz_ref_nmpc["/beetle1/nmpc/viz_ref/poses[0]/position/x"])
        plt.plot(t_ref, x_ref, label="$p_{x,a}$", linestyle="--", color=color_ref)

        t = np.array(data_xyz["__time"]) - t_bias
        x = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/x"])
        plt.plot(t, x, label="$p_x$", linestyle="-", color=color_real)

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("Position [m]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 2)

        t = np.array(data_euler["__time"]) - t_bias
        qw = np.array(data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/w"])
        qx = np.array(data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/x"])
        qy = np.array(data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/y"])
        qz = np.array(data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/z"])
        plt.plot(t, qw, label="$q_w$", linestyle=":")
        plt.plot(t, qx, label="$q_x$", linestyle="-.")
        plt.plot(t, qy, label="$q_y$", linestyle="--")
        plt.plot(t, qz, label="$q_z$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("Orientation", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 3)

        t = np.array(data_iterm["__time"]) - t_bias
        fx_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/force/x"])
        fy_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/force/y"])
        fz_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/force/z"])
        plt.plot(t, fx_iterm, label="$f_{x}$", linestyle="-.")
        plt.plot(t, fy_iterm, label="$f_{y}$", linestyle="--")
        plt.plot(t, fz_iterm, label="$f_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^W\\boldsymbol{f}_{dm}}$ [N]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 4)

        t = np.array(data_iterm["__time"]) - t_bias
        torque_x_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/x"])
        torque_y_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/y"])
        torque_z_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/z"])
        plt.plot(t, torque_x_iterm, label="$\\tau_{x}$", linestyle="-.")
        plt.plot(t, torque_y_iterm, label="$\\tau_{y}$", linestyle="--")
        plt.plot(t, torque_z_iterm, label="$\\tau_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^B\\boldsymbol{\\tau}_{dm}}$ [N$\cdot$m]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 5)

        t = np.array(data_ext_wrench_est["__time"]) - t_bias
        fx = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/force/x"])
        fy = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/force/y"])
        fz = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/force/z"])
        plt.plot(t, fx, label="$f_{x}$", linestyle="-.")
        plt.plot(t, fy, label="$f_{y}$", linestyle="--")
        plt.plot(t, fz, label="$f_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^B\hat{\\boldsymbol{f}}_{de,0}}$ [N]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 6)

        t = np.array(data_ext_wrench_est["__time"]) - t_bias
        torque_x = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/torque/x"])
        torque_y = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/torque/y"])
        torque_z = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/torque/z"])
        plt.plot(t, torque_x, label="$\\tau_{x}$", linestyle="-.")
        plt.plot(t, torque_y, label="$\\tau_{y}$", linestyle="--")
        plt.plot(t, torque_z, label="$\\tau_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^B\hat{\\boldsymbol{\\tau}}_{de,0}}$ [N$\cdot$m]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 7)
        t = np.array(data_thrust_cmd["__time"]) - t_bias
        thrust1 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[0]"])
        plt.plot(t, thrust1, label="$f_{c1}$", linestyle="-")
        thrust2 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[1]"])
        plt.plot(t, thrust2, label="$f_{c2}$", linestyle="--")
        thrust3 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[2]"])
        plt.plot(t, thrust3, label="$f_{c3}$", linestyle="-.")
        thrust4 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[3]"])
        plt.plot(t, thrust4, label="$f_{c4}$", linestyle=":")
        plt.ylabel("Thrust Cmd [N]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=2)

        # --------------------------------
        plt.subplot(4, 2, 8)
        t = np.array(data_servo_angle_cmd["__time"]) - t_bias
        servo1 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal1/position"]) * 180 / np.pi
        plt.plot(t, servo1, label="$\\alpha_{c1}$", linestyle="-")
        servo2 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal2/position"]) * 180 / np.pi
        plt.plot(t, servo2, label="$\\alpha_{c2}$", linestyle="--")
        servo3 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal3/position"]) * 180 / np.pi
        plt.plot(t, servo3, label="$\\alpha_{c3}$", linestyle="-.")
        servo4 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal4/position"]) * 180 / np.pi
        plt.plot(t, servo4, label="$\\alpha_{c4}$", linestyle=":")

        plt.ylabel("Servo Cmd [$^\\circ$]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=2)

        # --------------------------------
        plt.tight_layout()
        # make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    elif type == 1:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # default is 10
        label_size = 14

        fig = plt.figure(figsize=(7, 7))

        t_bias = max(data_xyz["__time"].iloc[0], data_xyz_ref_nmpc["__time"].iloc[0])
        axis_colors = (matlab_blue, matlab_orange, matlab_yellow)
        line_width = 1.0
        push_t_start = 7.4
        slide_t_start = 10.4
        push_slide_t_stop = 12.4

        # --------------------------------
        plt.subplot(4, 2, 1)

        # # plot a curve has the same length with t and the value is always 1.1. Only for admittance.
        # t = np.array(data_xyz["__time"]) - t_bias
        # x_original_ref = np.zeros_like(t) + 1.1
        # plt.plot(t, x_original_ref, label="$p_{x,r}$", linestyle=":", color=color_cog)

        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        x_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x"])
        plt.plot(t_ref, x_ref, label="$p_{x,r}$", linestyle="--", color=axis_colors[0], linewidth=line_width)
        y_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y"])
        plt.plot(t_ref, y_ref, label="$p_{y,r}$", linestyle="--", color=axis_colors[1], linewidth=line_width)
        z_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z"])
        plt.plot(t_ref, z_ref, label="$p_{z,r}$", linestyle="--", color=axis_colors[2], linewidth=line_width)

        t = np.array(data_xyz["__time"]) - t_bias
        x = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/x"])
        plt.plot(t, x, label="$p_x$", linestyle="-", color=axis_colors[0], linewidth=line_width)
        y = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/y"])
        plt.plot(t, y, label="$p_y$", linestyle="-", color=axis_colors[1], linewidth=line_width)
        z = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/z"])
        plt.plot(t, z, label="$p_z$", linestyle="-", color=axis_colors[2], linewidth=line_width)

        plt.legend(framealpha=legend_alpha, ncol=2)
        plt.ylabel("Position [m]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 2)

        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        roll_ref = np.array(data_euler_ref["roll"])
        pitch_ref = np.array(data_euler_ref["pitch"])
        yaw_ref = np.array(data_euler_ref["yaw"])
        plt.plot(
            t_ref,
            roll_ref * 180 / np.pi,
            label="$\\phi_r$",
            linestyle="--",
            color=axis_colors[0],
            linewidth=line_width,
        )
        plt.plot(
            t_ref,
            pitch_ref * 180 / np.pi,
            label="$\\theta_r$",
            linestyle="--",
            color=axis_colors[1],
            linewidth=line_width,
        )
        plt.plot(
            t_ref,
            yaw_ref * 180 / np.pi,
            label="$\\psi_r$",
            linestyle="--",
            color=axis_colors[2],
            linewidth=line_width,
        )

        t = np.array(data_euler["__time"]) - t_bias
        roll = np.array(data_euler["roll"])
        pitch = np.array(data_euler["pitch"])
        yaw = np.array(data_euler["yaw"])
        plt.plot(t, roll * 180 / np.pi, label="$\\phi$", linestyle="-", color=axis_colors[0], linewidth=line_width)
        plt.plot(
            t,
            pitch * 180 / np.pi,
            label="$\\theta$",
            linestyle="-",
            color=axis_colors[1],
            linewidth=line_width,
        )
        plt.plot(t, yaw * 180 / np.pi, label="$\\psi$", linestyle="-", color=axis_colors[2], linewidth=line_width)

        plt.legend(framealpha=legend_alpha, ncol=2)
        plt.ylabel("Orientation [$^\circ$]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 3)

        t = np.array(data_iterm["__time"]) - t_bias
        fx_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/force/x"])
        fy_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/force/y"])
        fz_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/force/z"])
        plt.plot(t, fx_iterm, label="$f_{x}$", linestyle="-.")
        plt.plot(t, fy_iterm, label="$f_{y}$", linestyle="--")
        plt.plot(t, fz_iterm, label="$f_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^W\\boldsymbol{f}_{dm}}$ [N]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 4)

        t = np.array(data_iterm["__time"]) - t_bias
        torque_x_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/x"])
        torque_y_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/y"])
        torque_z_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/z"])
        plt.plot(t, torque_x_iterm, label="$\\tau_{x}$", linestyle="-.")
        plt.plot(t, torque_y_iterm, label="$\\tau_{y}$", linestyle="--")
        plt.plot(t, torque_z_iterm, label="$\\tau_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^B\\boldsymbol{\\tau}_{dm}}$ [N$\cdot$m]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 5)

        force_time = np.array(data_ext_wrench_est["__time"])
        t = force_time - t_bias
        fx = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/force/x"])
        fy = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/force/y"])
        fz = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/force/z"])
        force_world = rotate_body_vectors_to_world(
            force_time,
            np.column_stack((fx, fy, fz)),
            data_qwxyz_cog,
            "/beetle1/uav/cog/odom/pose/pose/orientation",
        )
        fx, fy, fz = force_world.T
        plt.plot(t, fx, label="$f_{x}$", linestyle="-.")
        plt.plot(t, fy, label="$f_{y}$", linestyle="--")
        plt.plot(t, fz, label="$f_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^W\hat{\\boldsymbol{f}}_{de}}$ [N]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 6)

        t = np.array(data_ext_wrench_est["__time"]) - t_bias
        torque_x = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/torque/x"])
        torque_y = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/torque/y"])
        torque_z = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/torque/z"])
        plt.plot(t, torque_x, label="$\\tau_{x}$", linestyle="-.")
        plt.plot(t, torque_y, label="$\\tau_{y}$", linestyle="--")
        plt.plot(t, torque_z, label="$\\tau_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        if if_push_slide:
            plt.ylabel("${^B\hat{\\boldsymbol{\\tau}}_{B_o,\\text{lever}}}$ [N$\\cdot$m]", fontsize=label_size)
        else:
            plt.ylabel("${^B\hat{\\boldsymbol{\\tau}}_{de}}$ [N$\\cdot$m]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 7)
        t = np.array(data_thrust_cmd["__time"]) - t_bias
        thrust1 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[0]"])
        plt.plot(t, thrust1, label="$f_{c1}$", linestyle="-")
        thrust2 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[1]"])
        plt.plot(t, thrust2, label="$f_{c2}$", linestyle="--")
        thrust3 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[2]"])
        plt.plot(t, thrust3, label="$f_{c3}$", linestyle="-.")
        thrust4 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[3]"])
        plt.plot(t, thrust4, label="$f_{c4}$", linestyle=":")
        plt.ylabel("Thrust Cmd [N]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=2)

        # --------------------------------
        plt.subplot(4, 2, 8)
        t = np.array(data_servo_angle_cmd["__time"]) - t_bias
        servo1 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal1/position"]) * 180 / np.pi
        plt.plot(t, servo1, label="$\\alpha_{c1}$", linestyle="-")
        servo2 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal2/position"]) * 180 / np.pi
        plt.plot(t, servo2, label="$\\alpha_{c2}$", linestyle="--")
        servo3 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal3/position"]) * 180 / np.pi
        plt.plot(t, servo3, label="$\\alpha_{c3}$", linestyle="-.")
        servo4 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal4/position"]) * 180 / np.pi
        plt.plot(t, servo4, label="$\\alpha_{c4}$", linestyle=":")

        plt.ylabel("Servo Cmd [$^\\circ$]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=2)

        # --------------------------------
        if if_push_slide:
            for axis in fig.axes:
                axis.axvspan(push_t_start, push_slide_t_stop, facecolor=matlab_yellow, alpha=0.2)
                axis.axvspan(
                    slide_t_start,
                    push_slide_t_stop,
                    facecolor="none",
                    edgecolor="lightgray",
                    hatch="///",
                    linewidth=0.0,
                )

        plt.tight_layout()
        # make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    else:
        print("Invalid type")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot the trajectory. Please use plotjuggler to generate the csv file."
    )
    parser.add_argument("file_path", type=str, help="The file name of the trajectory")
    parser.add_argument("-t", "--type", type=int, help="The type of the trajectory")
    parser.add_argument(
        "-p",
        "--push_slide",
        action="store_true",
        help="Highlight the pushing/sliding intervals and use the lever-torque label",
    )

    args = parser.parse_args()

    main(args.file_path, args.type, args.push_slide)
