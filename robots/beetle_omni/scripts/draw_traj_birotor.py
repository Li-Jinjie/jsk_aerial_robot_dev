import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse

from matplotlib.lines import lineStyles

from utils import unwrap_angle_sequence, calculate_rmse, quat2euler, calculate_quat_error, interp_quat
from utils import matlab_yellow, matlab_green, matlab_orange, matlab_blue

legend_alpha = 0.5


def main(file_path, type, if_hand_teleop):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    # ======= xyz =========
    data_xyz_cog = data[
        [
            "__time",
            "/ball1/uav/cog/odom/pose/pose/position/x",
            "/ball1/uav/cog/odom/pose/pose/position/y",
            "/ball1/uav/cog/odom/pose/pose/position/z",
        ]
    ]

    try:
        data_xyz_ref = data[
            [
                "__time",
                "/ball1/set_ref_traj/points[0]/transforms[0]/translation/x",
                "/ball1/set_ref_traj/points[0]/transforms[0]/translation/y",
                "/ball1/set_ref_traj/points[0]/transforms[0]/translation/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_xyz_ref = pd.DataFrame()
        data_xyz_ref["__time"] = data_xyz_cog["__time"]
        data_xyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/translation/x"] = 0.0
        data_xyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/translation/y"] = 0.0
        data_xyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/translation/z"] = 1.0

    data_xyz_ref = data_xyz_ref.dropna()
    data_xyz_cog = data_xyz_cog.dropna()

    # ======= rpy =========
    data_qwxyz_cog = data[
        [
            "__time",
            "/ball1/uav/cog/odom/pose/pose/orientation/w",
            "/ball1/uav/cog/odom/pose/pose/orientation/x",
            "/ball1/uav/cog/odom/pose/pose/orientation/y",
            "/ball1/uav/cog/odom/pose/pose/orientation/z",
        ]
    ]

    try:
        data_qwxyz_ref = data[
            [
                "__time",
                "/ball1/set_ref_traj/points[0]/transforms[0]/rotation/w",
                "/ball1/set_ref_traj/points[0]/transforms[0]/rotation/x",
                "/ball1/set_ref_traj/points[0]/transforms[0]/rotation/y",
                "/ball1/set_ref_traj/points[0]/transforms[0]/rotation/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_qwxyz_ref = pd.DataFrame()
        data_qwxyz_ref["__time"] = data_qwxyz_cog["__time"]
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/w"] = 1.0
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/x"] = 0.0
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/y"] = 0.0
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/z"] = 0.0

    data_qwxyz_ref = data_qwxyz_ref.dropna()
    data_qwxyz_cog = data_qwxyz_cog.dropna()

    # convert to euler
    data_euler_ref = pd.DataFrame()
    data_euler_ref["__time"] = data_qwxyz_ref["__time"]
    data_euler_ref["roll"], data_euler_ref["pitch"], data_euler_ref["yaw"] = quat2euler(
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/w"],
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/x"],
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/y"],
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/z"],
        sequence="ZYX",
        degrees=False,
    )
    data_euler_ref["roll"] = unwrap_angle_sequence(data_euler_ref["roll"].to_numpy())
    data_euler_ref["pitch"] = unwrap_angle_sequence(data_euler_ref["pitch"].to_numpy())
    data_euler_ref["yaw"] = unwrap_angle_sequence(data_euler_ref["yaw"].to_numpy())

    # interpolate the real quaternion date
    t_ref = np.array(data_qwxyz_ref["__time"])
    t_cog = np.array(data_qwxyz_cog["__time"])
    data_qwxyz_cog_interp = interp_quat(t_ref, t_cog, data_qwxyz_cog, "/ball1/uav/cog/odom/pose/pose/orientation")

    ew_cog, ex_cog, ey_cog, ez_cog = calculate_quat_error(
        data_qwxyz_cog_interp["/ball1/uav/cog/odom/pose/pose/orientation/w"],
        data_qwxyz_cog_interp["/ball1/uav/cog/odom/pose/pose/orientation/x"],
        data_qwxyz_cog_interp["/ball1/uav/cog/odom/pose/pose/orientation/y"],
        data_qwxyz_cog_interp["/ball1/uav/cog/odom/pose/pose/orientation/z"],
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/w"],
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/x"],
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/y"],
        data_qwxyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/rotation/z"],
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
            "/ball1/four_axes/command/base_thrust[0]",
            "/ball1/four_axes/command/base_thrust[1]",
        ]
    ]
    data_thrust_cmd = data_thrust_cmd.dropna()

    data_servo_angle_cmd = data[
        [
            "__time",
            "/ball1/gimbals_ctrl/gimbal1/position",
            "/ball1/gimbals_ctrl/gimbal2/position",
        ]
    ]
    data_servo_angle_cmd = data_servo_angle_cmd.dropna()

    # # real servo angle
    # data_servo_angle = data[
    #     ['__time', '/ball1/joint_states/gimbal1/position', '/ball1/joint_states/gimbal2/position',
    #      '/ball1/joint_states/gimbal3/position', '/ball1/joint_states/gimbal4/position']]
    # data_servo_angle = data_servo_angle.dropna()

    # ======= est. wrench =========
    try:
        data_iterm = data[
            [
                "__time",
                "/ball1/dist_w_f_cog_tq/iterm/wrench/force/x",
                "/ball1/dist_w_f_cog_tq/iterm/wrench/force/y",
                "/ball1/dist_w_f_cog_tq/iterm/wrench/force/z",
                "/ball1/dist_w_f_cog_tq/iterm/wrench/torque/x",
                "/ball1/dist_w_f_cog_tq/iterm/wrench/torque/y",
                "/ball1/dist_w_f_cog_tq/iterm/wrench/torque/z",
            ]
        ]
        data_iterm = data_iterm.dropna()

        data_ext_pure = data[
            [
                "__time",
                "/ball1/dist_w_f_cog_tq/ext/wrench/force/x",
                "/ball1/dist_w_f_cog_tq/ext/wrench/force/y",
                "/ball1/dist_w_f_cog_tq/ext/wrench/force/z",
                "/ball1/dist_w_f_cog_tq/ext/wrench/torque/x",
                "/ball1/dist_w_f_cog_tq/ext/wrench/torque/y",
                "/ball1/dist_w_f_cog_tq/ext/wrench/torque/z",
            ]
        ]
        data_ext_pure = data_ext_pure.dropna()

        data_ext_wrench_est = data[
            [
                "__time",
                "/ball1/ext_wrench_est/value/wrench/force/x",
                "/ball1/ext_wrench_est/value/wrench/force/y",
                "/ball1/ext_wrench_est/value/wrench/force/z",
                "/ball1/ext_wrench_est/value/wrench/torque/x",
                "/ball1/ext_wrench_est/value/wrench/torque/y",
                "/ball1/ext_wrench_est/value/wrench/torque/z",
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

        fig = plt.figure(figsize=(7, 7))

        t_bias = max(data_xyz_cog["__time"].iloc[0], data_xyz_ref["__time"].iloc[0], data_xyz_cog["__time"].iloc[0])
        color_ref = "#0C5DA5"
        color_real = "#FF2C00"
        color_cog = "#f29619"  # the orange in scienceplots

        # --------------------------------
        plt.subplot(4, 2, 1)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        x_ref = np.array(data_xyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/translation/x"])
        plt.plot(t_ref, x_ref, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_xyz_cog["__time"]) - t_bias
        x_cog = np.array(data_xyz_cog["/ball1/uav/cog/odom/pose/pose/position/x"])
        plt.plot(t_cog, x_cog, label="cog", linestyle="-.", color=color_real)

        plt.legend(framealpha=legend_alpha, ncol=2)
        plt.ylabel("X [m]", fontsize=label_size)

        # # right Y-axis: error plot
        # ax = plt.gca()
        # ax2 = ax.twinx()
        # error_x = abs(x - np.interp(t, t_ref, x_ref))
        # ax2.plot(t, error_x, label='error', alpha=0.5)
        # ax2.set_ylabel('Err [m]', fontsize=label_size)
        # # ax2.tick_params(axis='y', labelcolor='tab:red')  # change the color of y axis

        # calculate RMSE
        rmse_x = calculate_rmse(t_cog, x_cog, t_ref, x_ref)
        print(f"RMSE X [m]: {rmse_x}")

        # --------------------------------
        plt.subplot(4, 2, 2)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        roll_ref = np.array(data_euler_ref["roll"])
        plt.plot(t_ref, roll_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_euler_cog["__time"]) - t_bias
        roll_cog = np.array(data_euler_cog["roll"])
        plt.plot(t_cog, roll_cog * 180 / np.pi, label="cog", linestyle="-.", color=color_real)

        plt.ylabel("Roll [$^\\circ$]", fontsize=label_size)

        # calculate RMSE
        rmse_roll = calculate_rmse(t_cog, roll_cog, t_ref, roll_ref)
        print(f"RMSE Roll [rad]: {rmse_roll}")
        print(f"RMSE Roll [deg]: {rmse_roll * 180 / np.pi}")

        # --------------------------------
        plt.subplot(4, 2, 3)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        y_ref = np.array(data_xyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/translation/y"])
        plt.plot(t_ref, y_ref, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_xyz_cog["__time"]) - t_bias
        y_cog = np.array(data_xyz_cog["/ball1/uav/cog/odom/pose/pose/position/y"])
        plt.plot(t_cog, y_cog, label="cog", linestyle="-.", color=color_real)

        plt.ylabel("Y [m]", fontsize=label_size)

        # plt.legend(framealpha=legend_alpha, ncol=2)

        # calculate RMSE
        rmse_y = calculate_rmse(t_cog, y_cog, t_ref, y_ref)
        print(f"RMSE Y [m]: {rmse_y}")

        # --------------------------------
        plt.subplot(4, 2, 4)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        pitch_ref = np.array(data_euler_ref["pitch"])
        plt.plot(t_ref, pitch_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_euler_cog["__time"]) - t_bias
        pitch_cog = np.array(data_euler_cog["pitch"])
        plt.plot(t_cog, pitch_cog * 180 / np.pi, label="cog", linestyle="-.", color=color_real)

        plt.ylabel("Pitch [$^\\circ$]", fontsize=label_size)

        # calculate RMSE
        rmse_pitch = calculate_rmse(t_cog, pitch_cog, t_ref, pitch_ref)
        print(f"RMSE Pitch [rad]: {rmse_pitch}")
        print(f"RMSE Pitch [deg]: {rmse_pitch * 180 / np.pi}")

        # --------------------------------
        plt.subplot(4, 2, 5)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        z_ref = np.array(data_xyz_ref["/ball1/set_ref_traj/points[0]/transforms[0]/translation/z"])
        plt.plot(t_ref, z_ref, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_xyz_cog["__time"]) - t_bias
        z_cog = np.array(data_xyz_cog["/ball1/uav/cog/odom/pose/pose/position/z"])
        plt.plot(t_cog, z_cog, label="cog", linestyle="-.", color=color_real)

        plt.ylabel("Z [m]", fontsize=label_size)

        # calculate RMSE
        rmse_z = calculate_rmse(t_cog, z_cog, t_ref, z_ref)
        print(f"RMSE Z [m]: {rmse_z}")

        # --------------------------------
        plt.subplot(4, 2, 6)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        yaw_ref = np.array(data_euler_ref["yaw"])
        plt.plot(t_ref, yaw_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_euler_cog["__time"]) - t_bias
        yaw_cog = np.array(data_euler_cog["yaw"])
        plt.plot(t_cog, yaw_cog * 180 / np.pi, label="cog", linestyle="-.", color=color_cog)

        plt.ylabel("Yaw [$^\\circ$]", fontsize=label_size)

        # calculate RMSE
        rmse_yaw = calculate_rmse(t_cog, yaw_cog, t_ref, yaw_ref, is_yaw=True)
        print(f"RMSE Yaw [rad]: {rmse_yaw}")
        print(f"RMSE Yaw [deg]: {rmse_yaw * 180 / np.pi}")

        # --------------------------------
        plt.subplot(4, 2, 7)
        t = np.array(data_thrust_cmd["__time"]) - t_bias
        thrust1 = np.array(data_thrust_cmd["/ball1/four_axes/command/base_thrust[0]"])
        plt.plot(t, thrust1, label="$f_{c1}$", linestyle="-")
        thrust2 = np.array(data_thrust_cmd["/ball1/four_axes/command/base_thrust[1]"])
        plt.plot(t, thrust2, label="$f_{c2}$", linestyle="--")
        plt.ylabel("Thrust Cmd [N]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="lower left", ncol=2)

        # --------------------------------
        plt.subplot(4, 2, 8)
        t = np.array(data_servo_angle_cmd["__time"]) - t_bias
        servo1 = np.array(data_servo_angle_cmd["/ball1/gimbals_ctrl/gimbal1/position"]) * 180 / np.pi
        plt.plot(t, servo1, label="$\\alpha_{c1}$", linestyle="-")
        servo2 = np.array(data_servo_angle_cmd["/ball1/gimbals_ctrl/gimbal2/position"]) * 180 / np.pi
        plt.plot(t, servo2, label="$\\alpha_{c2}$", linestyle="--")

        plt.ylabel("Servo Cmd [$^\\circ$]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="center left", ncol=2)

        # --------------------------------
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
    parser.add_argument("-o", "--hand_teleop", action="store_true", help="Whether the trajectory is from hand teleop")

    args = parser.parse_args()

    main(args.file_path, args.type, args.hand_teleop)
