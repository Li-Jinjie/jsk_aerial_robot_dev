"""
Created by li-jinjie on 25-1-4.
"""

import os
import sys
import argparse

import rospy
import rospkg
import smach
import smach_ros
import numpy as np
import yaml
import inspect

from aerial_robot_planning.pub_mpc_joint_traj import MPCTrajPtPub, MPCSinglePtPub
from aerial_robot_planning.pub_mpc_pred_xu import MPCPubCSVPredXU
from geometry_msgs.msg import Pose, Quaternion, Vector3
from aerial_robot_planning.util import TrajRegister, read_csv_traj, pub_0066_wall_rviz, pub_hand_markers_rviz

# ======== load smach config from the ROS package ========
try:
    ros_pack = rospkg.RosPack()
    pkg_path = ros_pack.get_path("aerial_robot_planning")
except rospkg.ResourceNotFound:
    raise RuntimeError("Package 'aerial_robot_planning' not found! Make sure your ROS workspace is sourced.")

config_path = os.path.join(pkg_path, "config", "Smach.yaml")
with open(config_path, "r") as f:
    smach_config = yaml.load(f, Loader=yaml.FullLoader)

# ======== collect all trajectory classes ========
traj_register = TrajRegister()

# === analytical trajectory ===
from aerial_robot_planning import trajs

anal_traj_list = [
    cls
    for name, cls in inspect.getmembers(trajs, inspect.isclass)
    # optionally ensure the class is defined in trajs and not an imported library
    if cls.__module__ == "aerial_robot_planning.trajs"
    and name not in {"BaseTraj", "BaseTrajwFixedRotor", "PitchContinuousRotationTraj"}
]
traj_register.register_anal_traj_list("trajs", anal_traj_list)

from aerial_robot_planning.voice import sound_trajs

anal_sound_traj_list = [
    cls
    for name, cls in inspect.getmembers(sound_trajs, inspect.isclass)
    if cls.__module__ == "aerial_robot_planning.voice.sound_trajs" and name not in {"BaseTrajwSound"}
]
traj_register.register_anal_traj_list("sound_trajs", anal_sound_traj_list)

# === CSV trajectory ===
# read all CSV files inside the folder
csv_folder_path = os.path.join(pkg_path, "data", "csv_trajs", "tilt_qd")
csv_files = sorted([f for f in os.listdir(csv_folder_path) if f.endswith(".csv")])
traj_register.register_csv_traj_list("csv_files", csv_files)

# === teleoperation ===
from aerial_robot_planning.teleoperation.teleop_smach import create_teleop_state_machine


###############################################
# SMACH States
###############################################
class IdleState(smach.State):
    """
    IDLE State:
    - Prompt user for robot_name, traj_type, loop_num.
    - On valid input, go INIT; otherwise, stay in IDLE.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_init", "stay_idle", "shutdown", "go_teleop"],
            input_keys=["robot_name"],
            output_keys=["robot_name", "traj_type", "loop_num"],
        )

    def execute(self, userdata):
        rospy.loginfo("State: IDLE -- Waiting for user input...")

        try:
            is_beetle = userdata.robot_name == "beetle1"

            traj_register.index_all_traj_and_print(has_csv=is_beetle)

            if is_beetle:
                print("\n===== Other Choices =====")
                print("t: Teleoperation Mode")

            traj_type_str = input(f"\nEnter trajectory number/letter above to select or 'q' to quit: ")
            if traj_type_str.lower() == "q":
                return "shutdown"

            if traj_type_str.lower() == "t":
                return "go_teleop"

            traj_index = int(traj_type_str)
            if not traj_register.is_traj_index_valid(traj_index):
                rospy.logwarn("Invalid trajectory type!")
                return "stay_idle"

            loop_num = 1.0  # TODO: add loop support for CSV trajectories
            if traj_register.is_analytic_traj_index(traj_index):
                loop_str = input("Enter loop number (or press Enter for infinite): ")
                if loop_str.strip() == "":
                    loop_num = np.inf
                else:
                    loop_num = float(loop_str)

            # Set user data
            userdata.traj_type = traj_index
            userdata.loop_num = loop_num

            return "go_init"

        except ValueError:
            rospy.logwarn("Invalid input. Please enter a valid number!")
            return "stay_idle"
        except EOFError:
            rospy.logwarn("No input detected (EOF). Staying in IDLE.")
            return "stay_idle"


class InitState(smach.State):
    """
    INIT State:
    - Sleep for 5 seconds to simulate “initialization”.
    - Then go to TRACK.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_track"],
            input_keys=["robot_name", "traj_type", "loop_num"],
            output_keys=[],
        )
        self.rate = rospy.Rate(20)  # 20 Hz loop checking if finished

    def execute(self, userdata):
        rospy.loginfo("State: INIT -- Start to reach the first point of the trajectory.")

        if traj_register.is_analytic_traj_index(userdata.traj_type):
            traj_cls = traj_register.lookup_analytic_traj_cls_by_index(userdata.traj_type)
            traj = traj_cls(userdata.loop_num)

            frame_id = traj.get_frame_id()
            child_frame_id = traj.get_child_frame_id()

            x, y, z, vx, vy, vz, ax, ay, az = traj.get_3d_pt(0.0)

            try:
                (qw, qx, qy, qz, r_rate, p_rate, y_rate, r_acc, p_acc, y_acc) = traj.get_3d_orientation(0.0)
            except AttributeError:
                qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        else:
            csv_file = traj_register.lookup_csv_traj_file_by_index(userdata.traj_type)

            traj_robot, frame_id, child_frame_id, traj_data_df = read_csv_traj(
                os.path.join(csv_folder_path, csv_file), nrows=1
            )
            if traj_robot != userdata.robot_name:
                rospy.logwarn(
                    f"Warning: The robot name in the CSV file ({traj_robot}) does not match the selected robot ({userdata.robot_name})."
                )

            x, y, z = traj_data_df.iloc[0][["px", "py", "pz"]]
            qw, qx, qy, qz = traj_data_df.iloc[0][["qw", "qx", "qy", "qz"]]

        init_pose = Pose(
            position=Vector3(x, y, z),
            orientation=Quaternion(qx, qy, qz, qw),
        )

        # Create the node instance
        mpc_node = MPCSinglePtPub(userdata.robot_name, frame_id, child_frame_id, init_pose)

        # Wait here until the node signals it is finished or ROS shuts down
        while not rospy.is_shutdown():
            if mpc_node.is_finished:
                rospy.loginfo("INIT: MPCSinglePtPub says the init pose is reached.")
                break
            self.rate.sleep()

        rospy.loginfo("Initialization done. Going to TRACK state.")
        return "go_track"


class TrackState(smach.State):
    """
    TRACK State:
    - Create an MPCPtPubNode with the user’s chosen parameters.
    - Keep running until MPCPtPubNode says it's finished or ROS is shutdown.
    - Then return "done_track".
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["done_track"],
            input_keys=["robot_name", "traj_type", "loop_num"],
            output_keys=[],
        )
        self.rate = rospy.Rate(20)  # 20 Hz loop check if finished

    def execute(self, userdata):
        rospy.loginfo(
            f"State: TRACK -- Starting for robot={userdata.robot_name}, "
            f"traj_type={userdata.traj_type}, loop_num={userdata.loop_num}"
        )

        if traj_register.is_analytic_traj_index(userdata.traj_type):
            traj_cls = traj_register.lookup_analytic_traj_cls_by_index(userdata.traj_type)
            traj = traj_cls(userdata.loop_num)
            mpc_node = MPCTrajPtPub(robot_name=userdata.robot_name, traj=traj)
        else:
            csv_file = traj_register.lookup_csv_traj_file_by_index(userdata.traj_type)
            mpc_node = MPCPubCSVPredXU(
                userdata.robot_name, os.path.join(csv_folder_path, csv_file), u_mode=smach_config["xu_traj"]["u_mode"]
            )

        # Wait here until the node signals it is finished or ROS shuts down
        while not rospy.is_shutdown():
            if mpc_node.is_finished:
                rospy.loginfo("TRACK: MPCPtPubNode says the trajectory is finished.")
                break
            self.rate.sleep()

        rospy.sleep(0.5)  # wait for the tracking error to be calculated and printed

        rospy.loginfo("TRACK: Done tracking. Going back to IDLE.")
        return "done_track"


###############################################
# Main SMACH Entry Point
###############################################
def main(args):
    # Initialize a single ROS node for the entire SMACH-based system
    rospy.init_node("mpc_smach_node")

    # check if the robot_name exists in ROS
    if not rospy.has_param(args.robot_name):
        rospy.logerr(f"Robot name '{args.robot_name}' not found in ROS parameters! Make sure the robot is running.")
        return

    # visualize in RViz
    pub_0066_wall_rviz(not smach_config["viz"]["has_0066_viz"])
    pub_hand_markers_rviz(smach_config["viz"]["hand_markers_viz_type"])

    # Create a top-level SMACH state machine
    sm = smach.StateMachine(outcomes=["DONE"])

    # Declare user data fields
    sm.userdata.robot_name = args.robot_name
    sm.userdata.traj_type = None
    sm.userdata.loop_num = np.inf

    # Open the container
    with sm:
        # IDLE
        smach.StateMachine.add(
            "IDLE",
            IdleState(),
            transitions={
                "go_init": "INIT",
                "stay_idle": "IDLE",
                "shutdown": "DONE",
                "go_teleop": "TELEOP",
            },
        )

        # INIT
        smach.StateMachine.add("INIT", InitState(), transitions={"go_track": "TRACK"})

        # TRACK
        smach.StateMachine.add("TRACK", TrackState(), transitions={"done_track": "IDLE"})

        # TELEOP
        smach.StateMachine.add(
            "TELEOP",
            create_teleop_state_machine(),
            transitions={"DONE_TELEOP": "IDLE"},
            remapping={"robot_name": "robot_name"},
        )

    # (Optional) Start an introspection server to visualize SMACH in smach_viewer
    sis = smach_ros.IntrospectionServer("mpc_smach_introspection", sm, "/MPC_SMACH")
    sis.start()

    # Execute the state machine
    sm.execute()

    sis.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SMACH-based MPC Trajectory Publisher")
    parser.add_argument("robot_name", type=str, help="Robot name, e.g., beetle1, gimbalrotors")

    args = parser.parse_args()
    main(args)
