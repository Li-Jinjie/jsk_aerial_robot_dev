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
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.use_pid = rospy.get_param("~use_pid", True)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create 4 symmetric gimbals
        self.gimbals: List[GimbalUnit] = [GimbalUnit(i, self.tf_buffer, self.world_frame) for i in range(1, 5)]

        # Subscribe joint states
        self.joint_state_sub = rospy.Subscriber(
            f"/{self.ns_robot}/joint_states", JointState, self.joint_state_cb, queue_size=10
        )

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
        # self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._on_timer)

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

        # 1) Update TFs and compute commands for each gimbal
        for g in self.gimbals:
            ok = g.update_from_tf(timeout=0.05)
            if not ok:
                rospy.logwarn_throttle(2.0, "[DistributedPID] TF not ready for %s", g.pitch_module_frame)
            g.compute_command(dt, use_pid=self.use_pid)

        # 2) Publish /dragon/gimbals_ctrl (JointState)
        js = JointState()
        js.header.stamp = now
        js.name = self.gimbal_names
        # Gather commanded positions in the same order
        positions: List[float] = []
        for idx in range(1, 5):
            g = self.gimbals[idx - 1]
            cmd_p, cmd_r = g.get_joint_commands()
            positions.extend([cmd_p, cmd_r])
        js.position = positions
        self.gimbals_pub.publish(js)

        # 3) Publish /dragon/four_axes/command (FourAxisCommand)
        #    Use params so you can set them dynamically via rosparam/launch.
        cmd = FourAxisCommand()
        cmd.angles = self._get_param_vec("~four_axes/target_angles", 3, 0.0)  # [roll, pitch, yaw] (rad)
        cmd.body_rates = self._get_param_vec("~four_axes/target_body_rates", 3, 0.0)  # [p, q, r] (rad/s)
        cmd.base_thrust = self._get_param_list("~four_axes/base_thrust", [0.0, 0.0, 0.0, 0.0])  # N (len may vary)
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
