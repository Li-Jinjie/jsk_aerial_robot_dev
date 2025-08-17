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

    def __init__(self, idx: int, tf_buffer: tf2_ros.Buffer, world_frame: str = "world"):
        self.idx = idx
        self.tf_buffer = tf_buffer
        self.world = world_frame

        # Frame names
        self.pitch_module_frame = f"gimbal{idx}_pitch_module"
        self.link_frame = f"link{idx}"

        # Joint names
        self.pitch_joint = f"gimbal{idx}_pitch"
        self.roll_joint = f"gimbal{idx}_roll"

        # Current measurements
        self.rpy_world = (0.0, 0.0, 0.0)  # (roll, pitch, yaw) of pitch_module_frame in world
        self.joint_pos = {self.pitch_joint: 0.0, self.roll_joint: 0.0}

        # Commanded positions (to publish to /dragon/gimbals_ctrl)
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0

        # Setpoints (can be changed via ROS params)
        ns = f"/gimbal{idx}"
        self.pitch_sp = rospy.get_param(f"{ns}/pitch_setpoint", 0.0)
        self.roll_sp = rospy.get_param(f"{ns}/roll_setpoint", 0.0)

        # PID gains from params
        kp_p = rospy.get_param(f"{ns}/pid_pitch/kp", 2.0)
        ki_p = rospy.get_param(f"{ns}/pid_pitch/ki", 0.0)
        kd_p = rospy.get_param(f"{ns}/pid_pitch/kd", 0.1)
        kp_r = rospy.get_param(f"{ns}/pid_roll/kp", 2.0)
        ki_r = rospy.get_param(f"{ns}/pid_roll/ki", 0.0)
        kd_r = rospy.get_param(f"{ns}/pid_roll/kd", 0.1)
        i_lim = rospy.get_param(f"{ns}/pid/i_limit", 1.0)
        out_lim = rospy.get_param(f"{ns}/pid/out_limit", math.radians(30.0))  # max step per cycle

        self.pid_pitch = PIDController(kp_p, ki_p, kd_p, i_limit=i_lim, out_limit=out_lim)
        self.pid_roll = PIDController(kp_r, ki_r, kd_r, i_limit=i_lim, out_limit=out_lim)

        # Saturation for commanded joint positions
        self.pitch_min = rospy.get_param(f"{ns}/limits/pitch_min", -math.pi)
        self.pitch_max = rospy.get_param(f"{ns}/limits/pitch_max", math.pi)
        self.roll_min = rospy.get_param(f"{ns}/limits/roll_min", -math.pi)
        self.roll_max = rospy.get_param(f"{ns}/limits/roll_max", math.pi)

    # -- Inputs --

    def update_from_tf(self, timeout: float = 0.05) -> bool:
        """Query TF for gimbal{i}_pitch_module -> world; update self.rpy_world."""
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                self.world, self.pitch_module_frame, rospy.Time(0), rospy.Duration(timeout)
            )
            q = t.transform.rotation
            rpy = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.rpy_world = rpy  # (roll, pitch, yaw)
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def update_joint_state(self, name_to_pos: Dict[str, float]):
        """Called by the main subscriber to refresh joint positions for this gimbal."""
        if self.pitch_joint in name_to_pos:
            self.joint_pos[self.pitch_joint] = name_to_pos[self.pitch_joint]
        if self.roll_joint in name_to_pos:
            self.joint_pos[self.roll_joint] = name_to_pos[self.roll_joint]

    # -- Control --

    def compute_command(self, dt: float, use_pid: bool = True):
        """
        Compute commands for pitch & roll. By default, apply PID on measured world R/P.
        Outputs absolute position targets (cmd_pitch/roll), clamped to joint limits.
        """
        # Measurement: use TF angles (world R,P) as approximate actual link attitude
        meas_roll, meas_pitch, _ = self.rpy_world

        if use_pid:
            # Error in angle space: setpoint - measurement
            u_pitch = self.pid_pitch.step(self.pitch_sp - meas_pitch, dt)
            u_roll = self.pid_roll.step(self.roll_sp - meas_roll, dt)
            # Command as current joint pos + control increment
            self.cmd_pitch = self.joint_pos[self.pitch_joint] + u_pitch
            self.cmd_roll = self.joint_pos[self.roll_joint] + u_roll
        else:
            # Directly command setpoints as joint targets
            self.cmd_pitch = self.pitch_sp
            self.cmd_roll = self.roll_sp

        # Clamp to limits
        self.cmd_pitch = max(self.pitch_min, min(self.pitch_max, self.cmd_pitch))
        self.cmd_roll = max(self.roll_min, min(self.roll_max, self.cmd_roll))

    def get_joint_commands(self) -> Tuple[float, float]:
        return self.cmd_pitch, self.cmd_roll
