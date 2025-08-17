"""
 Created by li-jinjie on 2025/8/17.
"""
import math

import numpy as np
import rospy
import tf2_ros
import tf_conversions
from typing import List, Dict, Tuple, Optional
from geometry_msgs.msg import TransformStamped, Transform

from pid_controller import PIDController


class GimbalUnit:
    """
    Encapsulates one gimbal (index 1..4).
    - Reads TF of gimbal{i}_pitch_module w.r.t. world
    - Reads joint states gimbal{i}_pitch / gimbal{i}_roll
    - Two PIDs (pitch, roll)
    - Produces commanded angles (position targets) for pitch/roll
    """

    def __init__(
        self, ns: str, idx: int, tf_buffer: tf2_ros.Buffer, world_frame: str = "world", z_force_offset: float = 0.0
    ):
        self.idx = idx
        self.tf_buffer = tf_buffer
        self.world = world_frame

        # --- state estimation ---
        # Frame names
        self.gimbal_pitch_module_frame = ns + f"/gimbal{idx}_pitch_module"
        self.link_frame = ns + f"/link{idx}"

        # Joint names
        self.gimbal_pitch_name = f"gimbal{idx}_pitch"
        self.gimbal_roll_name = f"gimbal{idx}_roll"
        self.gimbal_states = {self.gimbal_pitch_name: 0.0, self.gimbal_roll_name: 0.0}

        # Current measurements
        self.gimbal_pitch_in_w_transform: Optional[Transform] = None
        self.link_in_w_transform: Optional[Transform] = None

        # --- commands ---
        # Commanded positions (to publish to /dragon/gimbals_ctrl and /dragon/four_axes/command)
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_thrust = 0.0

        # --- controllers ---
        self.pid_x = PIDController(1.0, 0.0, 0.0, i_limit=1.0, out_limit=1.0)
        self.pid_y = PIDController(1.0, 0.0, 0.0, i_limit=1.0, out_limit=1.0)
        self.pid_z = PIDController(0.5, 0.1, 0.0, i_limit=1.0, out_limit=1.0, offset=z_force_offset)

        # Saturation for commanded joint positions
        self.force_min = -5.0  # N
        self.force_max = 5.0  # N

    def update_from_tf(self, timeout: float = 0.05) -> bool:
        """Query TF for gimbal{i}_pitch_module -> world; update self.rpy_world."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.world, self.gimbal_pitch_module_frame, rospy.Time(0), rospy.Duration(timeout)
            )
            self.gimbal_pitch_in_w_transform = t.transform

            t = self.tf_buffer.lookup_transform(self.world, self.link_frame, rospy.Time(0), rospy.Duration(timeout))
            self.link_in_w_transform = t.transform

            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def update_joint_state(self, name_to_pos: Dict[str, float]):
        """Called by the main subscriber to refresh joint positions for this gimbal."""
        if self.gimbal_pitch_name in name_to_pos:
            self.gimbal_states[self.gimbal_pitch_name] = name_to_pos[self.gimbal_pitch_name]
        if self.gimbal_roll_name in name_to_pos:
            self.gimbal_states[self.gimbal_roll_name] = name_to_pos[self.gimbal_roll_name]

    # -- Control --

    def compute_command(self, dt: float, ref_pos: Optional[Tuple[float, float, float]]):
        pos_w = self.gimbal_pitch_in_w_transform.translation
        fx_w = -np.clip(self.pid_x.step(pos_w.x - ref_pos[0], dt), self.force_min, self.force_max)
        fy_w = -np.clip(self.pid_y.step(pos_w.y - ref_pos[1], dt), self.force_min, self.force_max)
        fz_w = -np.clip(self.pid_z.step(pos_w.z - ref_pos[2], dt), self.force_min, self.force_max) + self.pid_z.offset

        # convert from world frame to link frame
        q = self.link_in_w_transform.rotation
        link_in_w_mtx = tf_conversions.transformations.quaternion_matrix((q.x, q.y, q.z, q.w))[0:3, 0:3]
        f_link = link_in_w_mtx.transpose() @ np.array([[fx_w, fy_w, fz_w]]).T

        self.cmd_roll = math.atan2(-f_link[1], f_link[2])
        self.cmd_pitch = math.atan2(
            f_link[0], -f_link[1] * math.sin(self.cmd_roll) + f_link[2] * math.cos(self.cmd_roll)
        )
        self.cmd_thrust = math.sqrt(f_link[0] ** 2 + f_link[1] ** 2 + f_link[2] ** 2)

    def get_commands(self) -> Tuple[float, float, float]:
        return self.cmd_pitch, self.cmd_roll, self.cmd_thrust
