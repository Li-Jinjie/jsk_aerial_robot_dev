"""
Created by li-jinjie on 24-1-3.
"""

# !/usr/bin/env python3

import sys
import os
import numpy as np
import rospy
from abc import ABC, abstractmethod

from std_msgs.msg import MultiArrayDimension
from aerial_robot_msgs.msg import PredXU
from nav_msgs.msg import Path

# Insert current folder into path so we can import from "trajs" or other local files
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)

from pub_mpc_joint_traj import MPCPubBase
from util import read_csv_traj, check_traj_info


##########################################
# Derived Class: MPCPubPredXU
##########################################
class MPCPubPredXU(MPCPubBase, ABC):
    def __init__(self, robot_name: str, node_name: str, odom_frame_id: str, is_calc_rmse=True):
        super().__init__(robot_name, node_name, odom_frame_id, is_calc_rmse)
        # Publisher for PredXU
        self.pub_ref_xu = rospy.Publisher(f"/{robot_name}/set_ref_x_u", PredXU, queue_size=3)

    def pub_trajectory_points(self, pred_xu_msg: PredXU):
        """Publish the PredXU message."""
        pred_xu_msg.header.stamp = rospy.Time.now()
        self.pub_ref_xu.publish(pred_xu_msg)


##########################################
# Derived Class #1: MPCPubCSVPredXU
##########################################
class MPCPubCSVPredXU(MPCPubPredXU):
    """
    Derived from MPCPubPredXU, which already inherits from MPCPubBase.
    This class loads a trajectory from CSV, interpolates it, and publishes
    PredXU messages at the rate defined by the base class (~50Hz).
    """

    def __init__(self, robot_name: str, file_path: str) -> None:
        # Load trajectory from a CSV
        traj_robot, traj_frame_id, traj_child_frame_id, traj_data_df = read_csv_traj(file_path)

        # Initialize parent classes
        super().__init__(robot_name=robot_name, node_name="mpc_xu_pub_node", odom_frame_id=traj_child_frame_id)

        if traj_robot != self.robot_name:
            rospy.logwarn(
                f"Warning: The robot name in the CSV file ({traj_robot}) does not match the selected robot ({robot_name})."
            )

        self.frame_id = traj_frame_id
        self.child_frame_id = traj_child_frame_id

        time_id = traj_data_df.columns.get_loc("time")
        effort_id = traj_data_df.columns.get_loc("control effort")

        self.time_traj = traj_data_df.iloc[:, time_id].to_numpy()
        self.effort_traj = traj_data_df.iloc[:, effort_id].to_numpy()

        # Get all columns between "time" and "control effort" (exclusive)
        self.x_traj = traj_data_df.iloc[:, time_id + 1 : effort_id].to_numpy()

        # Get all columns after "control effort"
        self.u_traj = traj_data_df.iloc[:, effort_id + 1 :].to_numpy()

        if self.x_traj.shape[1] != self.nx - 6 or self.u_traj.shape[1] != self.nu:  # 6 for external wrench
            raise ValueError(
                f"Trajectory dimensions do not match NMPC settings! "
                f"Expected {self.nx} states and {self.nu} controls, "
                f"but got {self.x_traj.shape[1]} states and {self.u_traj.shape[1]} controls."
            )

        # Check trajectory information and visualize it
        traj_path_msg = check_traj_info(self.time_traj, self.x_traj, if_return_path=True)
        self.traj_path_pub = rospy.Publisher(f"/{self.robot_name}/traj_path", Path, queue_size=1, latch=True)
        self.traj_path_pub.publish(traj_path_msg)

        input_str = input(
            "Please check the traj info and Rviz visualization. Press 'Enter' to continue or 'q' to quit..."
        )
        while True:
            if input_str.lower() == "q":
                self.cleanup_traj_viz()
                self.is_finished = True
                return
            elif input_str.lower() == "":
                break
            else:
                input_str = input("Invalid input. Please press 'Enter' to continue or 'q' to quit...")

        # Adjust your control inputs if needed
        self.u_traj[:, 4:8] = self.x_traj[:, 13:17]

        self.start_timer()

    def fill_trajectory_points(self, t_elapsed: float) -> PredXU:
        """
        Construct and return a PredXU message for the current time `t_elapsed`.
        This method is called automatically by the base class timer (~50 Hz).
        """
        ref_xu_msg = PredXU()
        ref_xu_msg.x.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        ref_xu_msg.u.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        ref_xu_msg.x.layout.dim[1].stride = self.nx
        ref_xu_msg.x.layout.dim[0].size = self.N_nmpc + 1
        ref_xu_msg.u.layout.dim[1].stride = self.nu
        ref_xu_msg.u.layout.dim[0].size = self.N_nmpc

        # set frame
        ref_xu_msg.header.frame_id = self.frame_id
        ref_xu_msg.child_frame_id = self.child_frame_id

        # set data
        # Create time nodes for interpolation
        t_nodes = np.linspace(0, self.T_horizon, self.N_nmpc + 1)
        t_nodes += t_elapsed

        # Allocate storage for interpolation results
        x_traj = np.zeros((self.N_nmpc + 1, self.nx))
        u_traj = np.zeros((self.N_nmpc, self.nu))

        # Interpolate each state dimension using NumPy's interp in a vectorized way.
        # Although a Python loop is still used internally (via list comprehension), this runs much faster than
        # a regular for-loop because the interpolation itself is performed in compiled C code.
        x_traj[:, : self.nx - 6] = np.array(
            [np.interp(t_nodes, self.time_traj, self.x_traj[:, i]) for i in range(self.nx - 6)]
        ).T

        u_traj[:, :] = np.array([np.interp(t_nodes[:-1], self.time_traj, self.u_traj[:, i]) for i in range(self.nu)]).T

        # Populate the PredXU message
        ref_xu_msg.x.data = x_traj.flatten().tolist()
        ref_xu_msg.u.data = u_traj.flatten().tolist()

        # Return the reference message (used by pub_trajectory_points in the parent)
        return ref_xu_msg

    def check_finished(self, t_elapsed: float) -> bool:
        """
        Return True if we have exceeded the final trajectory time.
        This will cause the base class timer to shut down automatically.
        """
        if t_elapsed > self.time_traj[-1]:
            self.cleanup_traj_viz()
            rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory time finished!")
            return True

        return False

    def cleanup_traj_viz(self):
        """
        Clean up the trajectory visualization in Rviz.
        """
        empty = Path()
        empty.header.frame_id = "world"
        self.traj_path_pub.publish(empty)
