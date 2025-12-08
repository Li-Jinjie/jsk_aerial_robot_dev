"""
 Created by li-jinjie on 2025/12/7.
"""
from .trajs import BaseTraj
from typing import Type
import rospy
import pandas as pd


class TrajRegister:
    def __init__(self):
        self.num_analytic_traj = 0
        self.analytic_traj_list_dict = {}

        self.num_csv_traj = 0
        self.csv_traj_file_list_dict = {}

        self.index_trajs_list = []

    def register_anal_traj_list(self, name: str, anal_traj_list: list):
        self.analytic_traj_list_dict[name] = anal_traj_list
        print(f"Registered trajectory list '{name}' with {len(anal_traj_list)} trajectories.")
        self.num_analytic_traj += len(anal_traj_list)

    def register_csv_traj_list(self, name: str, csv_traj_file_list: list):
        self.csv_traj_file_list_dict[name] = csv_traj_file_list
        print(f"Registered CSV trajectory list '{name}' with {len(csv_traj_file_list)} trajectories.")
        self.num_csv_traj += len(csv_traj_file_list)

    def index_all_traj_and_print(self, has_csv: bool = True):
        self._index_all_analytic_traj_and_print()

        if has_csv:
            self._index_all_csv_traj_and_print()

    def get_max_traj_index(self) -> int:
        return len(self.index_trajs_list) - 1

    def is_traj_index_valid(self, index: int) -> bool:
        return 0 <= index <= self.get_max_traj_index()

    def is_analytic_traj_index(self, index: int) -> bool:
        return index < self.num_analytic_traj

    def lookup_analytic_traj_cls_by_index(self, index: int) -> Type[BaseTraj]:
        if not self.is_traj_index_valid(index):
            raise IndexError(f"Trajectory index {index} is out of range.")

        traj = self.index_trajs_list[index]

        if isinstance(traj, str):
            raise TypeError(f"Trajectory index {index} corresponds to a CSV file, not an Analytic trajectory.")

        rospy.loginfo(f"Selected Analytic trajectory: {traj.__name__}")

        return traj

    def lookup_csv_traj_file_by_index(self, index: int) -> str:
        if not self.is_traj_index_valid(index):
            raise IndexError(f"Trajectory index {index} is out of range.")

        traj = self.index_trajs_list[index]

        if not isinstance(traj, str):
            raise TypeError(f"Trajectory index {index} corresponds to an Analytic trajectory, not a CSV file.")

        rospy.loginfo(f"Selected CSV trajectory file: {traj}")

        return traj

    def _index_all_analytic_traj_and_print(self):
        index = 0

        print("\n=== Analytic Trajectory Lists ===")
        for list_name, traj_list in self.analytic_traj_list_dict.items():
            print(f"\nList '{list_name}':")
            for traj in traj_list:
                print(f"{index}: {traj.__name__}")
                index += 1
                self.index_trajs_list.append(traj)

    def _index_all_csv_traj_and_print(self):
        index = self.num_analytic_traj  # the CSV trajs are always printed after analytic trajs

        print("\n=== CSV Trajectory File Lists ===")
        for list_name, file_list in self.csv_traj_file_list_dict.items():
            print(f"\nList '{list_name}':")
            for file_path in file_list:
                print(f"{index}: {file_path}")
                index += 1
                self.index_trajs_list.append(file_path)


def read_csv_traj(path, nrows=None):
    # please read READEME.md in tilt_qd_csv_trajs for the csv file format
    non_data_row_num = 0
    with open(path, "r") as f:
        robot_line = f.readline().strip().split(",")
        frame_id_line = f.readline().strip().split(",")
        child_frame_id_line = f.readline().strip().split(",")
        non_data_row_num += 3

        if robot_line[0] != "robot" or frame_id_line[0] != "frame_id" or child_frame_id_line[0] != "child_frame_id":
            raise ValueError("CSV file format error: first three lines must be 'robot', 'frame_id', 'child_frame_id'")
        robot = robot_line[1] if len(robot_line) > 1 else None
        frame_id = frame_id_line[1] if len(frame_id_line) > 1 else None
        child_frame_id = child_frame_id_line[1] if len(child_frame_id_line) > 1 else None

    if nrows is not None:
        df = pd.read_csv(path, skiprows=non_data_row_num, nrows=nrows)
    else:
        df = pd.read_csv(path, skiprows=non_data_row_num)

    # check each column has a header
    for col in df.columns:
        if col.strip() == "":
            raise ValueError("CSV file format error: all columns must have a header")

    return robot, frame_id, child_frame_id, df
