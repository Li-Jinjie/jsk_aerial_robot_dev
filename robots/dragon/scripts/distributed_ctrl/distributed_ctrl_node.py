"""
 Created by li-jinjie on 2025/8/17.
"""
import rospy
import tf2_ros
from typing import List, Dict, Tuple, Optional
from sensor_msgs.msg import JointState
from spinal.msg import FourAxisCommand  # uses your uploaded message

from gimbal_unit import GimbalUnit


class DistributedPIDController:
    """Main node that owns four GimbalUnit and handles I/O."""

    def __init__(self):
        rospy.init_node("distributed_pid_controller", anonymous=False)

        # Params
        self.ns_robot = rospy.get_param("~robot_ns", "dragon")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.rate_hz = rospy.get_param("~rate", 100.0)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create 4 symmetric gimbals
        fz_offset = [17.3, 22.3, 21.9, 16.9]
        self.gimbals: List[GimbalUnit] = [
            GimbalUnit(self.ns_robot, i, self.tf_buffer, self.world_frame, fz_offset[i - 1]) for i in range(1, 5)
        ]

        # Subscribe joint states
        self.joint_state_sub = rospy.Subscriber(
            f"/{self.ns_robot}/joint_states", JointState, self.joint_state_cb, queue_size=10
        )
        print(f"/{self.ns_robot}/joint_states")

        # Publishers
        self.gimbals_pub = rospy.Publisher(f"/{self.ns_robot}/gimbals_ctrl", JointState, queue_size=10)
        self.four_axes_pub = rospy.Publisher(f"/{self.ns_robot}/four_axes/command", FourAxisCommand, queue_size=10)

        # Predefine JointState name order (exactly as you requested)
        self.gimbal_names = [
            "gimbal1_pitch",
            "gimbal1_roll",
            "gimbal2_pitch",
            "gimbal2_roll",
            "gimbal3_pitch",
            "gimbal3_roll",
            "gimbal4_pitch",
            "gimbal4_roll",
        ]

        # Internal state
        self._name_to_pos: Dict[str, float] = {}  # latest joint positions
        self._last_time = rospy.Time.now()

        # Control loop timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._on_timer)

        rospy.loginfo(
            "[DistributedPID] node initialized. rate=%.1f Hz, world='%s', robot='%s'",
            self.rate_hz,
            self.world_frame,
            self.ns_robot,
        )

    # -- Subscribers --

    def joint_state_cb(self, msg: JointState):
        """Update name->position map and forward to each gimbal unit."""
        name_to_pos: Dict[str, float] = {}
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                name_to_pos[n] = msg.position[i]
        self._name_to_pos = name_to_pos
        for g in self.gimbals:
            g.update_joint_state(name_to_pos)

    # -- Timer loop --

    def _on_timer(self, event: rospy.timer.TimerEvent):
        now = rospy.Time.now()
        dt = (now - self._last_time).to_sec()
        if dt <= 0.0:
            dt = 1.0 / self.rate_hz
        self._last_time = now

        # reference:
        takeoff_height = rospy.get_param(self.ns_robot + "/navigation/takeoff_height", 0.5)
        ref_pos_link1 = [0.21, 0.01, takeoff_height]
        ref_pos_link2 = [0.48, 0.22, takeoff_height]
        ref_pos_link3 = [0.26, 0.48, takeoff_height]
        ref_pos_link4 = [0.00, 0.27, takeoff_height]
        ref_pos = [ref_pos_link1, ref_pos_link2, ref_pos_link3, ref_pos_link4]

        # 1) Update TFs and compute commands for each gimbal
        for i, g in enumerate(self.gimbals):
            ok = g.update_from_tf(timeout=0.05)
            if not ok:
                rospy.logwarn_throttle(2.0, "[DistributedPID] TF not ready for %s", g.gimbal_pitch_module_frame)
            g.compute_command(dt, ref_pos[i])

        # 2) Publish /dragon/gimbals_ctrl (JointState) and /dragon/four_axes/command (FourAxisCommand)
        js = JointState()
        cmd = FourAxisCommand()

        js.name = self.gimbal_names
        # Gather commanded positions in the same order
        positions: List[float] = []
        base_thrust: List[float] = []
        for idx in range(1, 5):
            g = self.gimbals[idx - 1]
            cmd_p, cmd_r, cmd_th = g.get_commands()
            positions.extend([cmd_p, cmd_r])
            base_thrust.append(cmd_th)
        js.position = positions
        cmd.base_thrust = base_thrust

        js.header.stamp = rospy.Time.now()
        self.gimbals_pub.publish(js)
        self.four_axes_pub.publish(cmd)

    # -- Helpers --

    @staticmethod
    def _get_param_vec(key: str, n: int, default_val: float) -> List[float]:
        v = rospy.get_param(key, None)
        if isinstance(v, list) and len(v) >= n:
            return [float(v[i]) for i in range(n)]
        return [default_val] * n

    @staticmethod
    def _get_param_list(key: str, default_list: List[float]) -> List[float]:
        v = rospy.get_param(key, None)
        if isinstance(v, list):
            return [float(x) for x in v]
        return list(default_list)


# ------------------------------- main ---------------------------------

if __name__ == "__main__":
    try:
        node = DistributedPIDController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
