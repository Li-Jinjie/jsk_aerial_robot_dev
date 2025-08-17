"""
 Created by li-jinjie on 2025/8/17.
"""
import math
import rospy
import tf2_ros
import tf_conversions
from typing import List, Dict, Tuple, Optional
from geometry_msgs.msg import TransformStamped

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

        # Current measurements
        self.pos_w = (0.0, 0.0, 0.0)  # Position of pitch_module_frame in world
        self.quat_w = (0.0, 0.0, 0.0, 1.0)  # Quaternion of pitch_module_frame in world
        self.rpy_w = (0.0, 0.0, 0.0)  # (roll, pitch, yaw) of pitch_module_frame in world
        self.gimbal_pos = {self.gimbal_pitch_name: 0.0, self.gimbal_roll_name: 0.0}

        # --- commands ---
        # Commanded positions (to publish to /dragon/gimbals_ctrl and /dragon/four_axes/command)
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_thrust = 0.0

        # --- controllers ---
        self.pid_x = PIDController(2.0, 0.0, 0.0, i_limit=1.0, out_limit=1.0)
        self.pid_y = PIDController(2.0, 0.0, 0.0, i_limit=1.0, out_limit=1.0)
        self.pid_z = PIDController(2.0, 0.0, 0.0, i_limit=1.0, out_limit=1.0, offset=z_force_offset)

        # Saturation for commanded joint positions
        self.force_min = -5.0  # N
        self.force_max = 5.0  # N

    def update_from_tf(self, timeout: float = 0.05) -> bool:
        """Query TF for gimbal{i}_pitch_module -> world; update self.rpy_world."""
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                self.world, self.gimbal_pitch_module_frame, rospy.Time(0), rospy.Duration(timeout)
            )
            self.pos_w = (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
            self.quat_w = (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )
            self.rpy_w = tf_conversions.transformations.euler_from_quaternion(
                (self.quat_w[0], self.quat_w[1], self.quat_w[2], self.quat_w[3])
            )
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def update_joint_state(self, name_to_pos: Dict[str, float]):
        """Called by the main subscriber to refresh joint positions for this gimbal."""
        if self.gimbal_pitch_name in name_to_pos:
            self.gimbal_pos[self.gimbal_pitch_name] = name_to_pos[self.gimbal_pitch_name]
        if self.gimbal_roll_name in name_to_pos:
            self.gimbal_pos[self.gimbal_roll_name] = name_to_pos[self.gimbal_roll_name]

    # -- Control --

    def compute_command(self, dt: float):
        """
        Compute commands for pitch & roll. By default, apply PID on measured world R/P.
        Outputs absolute position targets (cmd_pitch/roll), clamped to joint limits.
        """
        # Measurement: use TF angles (world R,P) as approximate actual link attitude
        # print(f"[GimbalUnit {self.idx}] pos_w={self.pos_w}, quat_w={self.quat_w}, rpy_w={self.rpy_w}")
        # print(f"[GimbalUnit {self.idx}] gimbal_pos={self.gimbal_pos}")

        # meas_roll, meas_pitch, _ = self.rpy_w
        #
        # if use_pid:
        #     # Error in angle space: setpoint - measurement
        #     u_pitch = self.pid_pitch.step(self.pitch_sp - meas_pitch, dt)
        #     u_roll = self.pid_roll.step(self.roll_sp - meas_roll, dt)
        #     # Command as current joint pos + control increment
        #     self.cmd_pitch = self.gimbal_pos[self.gimbal_pitch_name] + u_pitch
        #     self.cmd_roll = self.gimbal_pos[self.gimbal_roll_name] + u_roll
        # else:
        #     # Directly command setpoints as joint targets
        #     self.cmd_pitch = self.pitch_sp
        #     self.cmd_roll = self.roll_sp
        #
        # # Clamp to limits
        # self.cmd_pitch = max(self.pitch_min, min(self.pitch_max, self.cmd_pitch))
        # self.cmd_roll = max(self.roll_min, min(self.roll_max, self.cmd_roll))

        self.cmd_thrust = self.pid_z.step(0.1 - self.pos_w[2], dt)

    def get_commands(self) -> Tuple[float, float, float]:
        return self.cmd_pitch, self.cmd_roll, self.cmd_thrust
