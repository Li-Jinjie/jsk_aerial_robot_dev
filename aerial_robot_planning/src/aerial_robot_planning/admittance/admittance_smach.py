"""
 Created by jinjie on 2026/04/03.
"""
import numpy as np
import rospy
import smach
from nav_msgs.msg import Odometry

from ..util import topic_ready

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from scipy.spatial.transform import Rotation as R


def skew(v):
    v = np.asarray(v).reshape(
        3,
    )
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def quat_multiply(qx1, qy1, qz1, qw1, qx2, qy2, qz2, qw2):
    """
    Multiply two quaternions.
    """
    return np.array(
        [
            qw1 * qx2 + qx1 * qw2 + qy1 * qz2 - qz1 * qy2,
            qw1 * qy2 - qx1 * qz2 + qy1 * qw2 + qz1 * qx2,
            qw1 * qz2 + qx1 * qy2 - qy1 * qx2 + qz1 * qw2,
            qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2,
        ]
    )


class AdmittanceState(smach.State):
    def __init__(self, frame_type="ee"):
        smach.State.__init__(self, outcomes=["done_admittance"], input_keys=["robot_name"], output_keys=["robot_name"])

        assert frame_type in ["ee", "cog"]
        self.frame_type = frame_type

        self.dt = 0.02
        self.rate = rospy.Rate(1 / self.dt)

        self.odom_sub = None
        self.odom_msg = Odometry()

        self.ext_wrench_body_frame_sub = None
        self.ext_wrench_body_frame_msg = WrenchStamped()

        # ROS parameters
        self.N_nmpc = None

        self.start_time = None

        # Physical parameters for the end-effector mode
        # TODO: make this part automatically update with different end-effectors
        self.p_ee_in_body = np.array([[0.0, 0.0029, 0.2644]]).T
        self.q_ee_in_body = np.array([[0, 0, 0, 1]]).T  # xyzw, same order as tf
        self.rot_body_ee = R.from_quat(self.q_ee_in_body.flatten()).as_matrix()

        # ===== param load server =====
        self.param_update_rate = 2.0
        self.param_ns_admittance = None
        self.param_timer = None
        self._last_param_signature = None

        # ====== threshold to avoid too small wrench =====
        # 0.5N, 0.1 Nm are based on the rosbag before.
        self.force_thresh = 0.5  # [N]
        self.torque_thresh = 0.1  # [N*m]

        # ====== M D K ======
        # Note: these parameters will be overridden by the function _update_params_if_needed(self, event=None)
        # Please check _read_param_dict() for the default parameters
        self.Mp = np.diag([3, 3, 3])  # [kg]
        self.Mp_rev = np.linalg.inv(self.Mp)
        self.Dp = np.diag([10, 10, 10])  # [N*s/m]
        self.Kp = np.diag([20, 20, 20])  # [N/m]    # 20 for rigid

        self.Mq = np.diag([0.5, 0.5, 0.5])  # [kg*m^2]
        self.Mq_rev = np.linalg.inv(self.Mq)
        self.Dq = np.diag([5.0, 5.0, 5.0])  # [N*m*s/rad]
        self.Kq = np.diag([10, 10, 10])  # [N*m/rad]    # 10 for rigid

        # ====== admittance state ========
        self.p_a = np.zeros(3)  # [m]
        self.v_a = np.zeros(3)  # [m/s]
        self.q_a = np.array([0, 0, 0, 1])  # xyzw
        self.w_a = np.zeros(3)  # [rad/s]

        self.p_r = np.zeros(3)
        self.q_r = np.array([0, 0, 0, 1])  # xyzw

        # ====== publisher ======
        self.admittance_wrench_pub = None
        self.pub_ref_traj = None

    def execute(self, userdata):
        rospy.loginfo("Executing state AdmittanceState (%s)", self.frame_type)
        robot_name = userdata.robot_name

        # ====== get M, D, K params =====
        self.param_ns_admittance = f"/admittance"
        self._update_params_if_needed()

        if self.param_timer is not None:
            self.param_timer.shutdown()
        self.param_timer = rospy.Timer(
            rospy.Duration(1.0 / self.param_update_rate),
            self._update_params_if_needed,
        )
        # ===============================

        frame_name = "ee" if self.frame_type == "ee" else "cog"
        odom_topic = f"/{robot_name}/uav/ee_contact/odom" if self.frame_type == "ee" else f"/{robot_name}/uav/cog/odom"
        wrench_pub_topic = f"/{robot_name}/admittance_{frame_name}/f_w_tq_t"

        self.admittance_wrench_pub = rospy.Publisher(wrench_pub_topic, WrenchStamped, queue_size=10)
        self.pub_ref_traj = rospy.Publisher(f"/{robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=3)

        self.N_nmpc = rospy.get_param(f"{robot_name}/controller/nmpc/NN")

        try:
            if not topic_ready(odom_topic, Odometry, timeout=5):
                rospy.logerr(f"Topic {odom_topic} not available. Exiting AdmittanceState.")
                return "done_admittance"

            self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self._sub_odom_callback)

            ext_wrench_body_frame_topic = f"/{robot_name}/ext_wrench_est/value"
            if not topic_ready(ext_wrench_body_frame_topic, WrenchStamped, timeout=5):
                rospy.logerr(f"Topic {ext_wrench_body_frame_topic} not available. Exiting AdmittanceState.")
                return "done_admittance"

            self.ext_wrench_body_frame_sub = rospy.Subscriber(
                ext_wrench_body_frame_topic, WrenchStamped, self._sub_ext_wrench_body_frame_callback
            )

        except Exception as e:
            rospy.logerr(f"Initialization error in AdmittanceState: {e}")
            return "done_admittance"

        self.start_time = rospy.Time.now().to_sec()

        self.p_a = np.array(
            [
                self.odom_msg.pose.pose.position.x,
                self.odom_msg.pose.pose.position.y,
                self.odom_msg.pose.pose.position.z,
            ]
        )
        self.q_a = np.array(
            [
                self.odom_msg.pose.pose.orientation.x,
                self.odom_msg.pose.pose.orientation.y,
                self.odom_msg.pose.pose.orientation.z,
                self.odom_msg.pose.pose.orientation.w,
            ]
        )

        # # modify q_a to let it rotate around pitch for -90 degree.
        # rot_old = R.from_quat(self.q_a)
        # rot_pitch_minus_90 = R.from_euler("y", -90, degrees=True)
        # rot_new = rot_old * rot_pitch_minus_90
        # self.q_a = rot_new.as_quat()

        self.p_r = self.p_a.copy()
        self.p_r[0] = 1.1
        self.q_r = self.q_a.copy()

        while not rospy.is_shutdown():
            force_w, torque_local = self.compute_wrench()

            if np.linalg.norm(force_w) < self.force_thresh:
                force_w = np.zeros(3)

            if np.linalg.norm(torque_local) < self.torque_thresh:
                torque_local = np.zeros(3)

            self.publish_wrench(force_w, torque_local)

            a_lin_a = self.Mp_rev @ (force_w - self.Dp @ self.v_a - self.Kp @ (self.p_a - self.p_r))
            self.v_a = self.v_a + a_lin_a * self.dt
            self.p_a = self.p_a + self.v_a * self.dt

            # fmt: off
            q_error = quat_multiply(
                -self.q_r[0], -self.q_r[1], -self.q_r[2], self.q_r[3],
                self.q_a[0], self.q_a[1], self.q_a[2], self.q_a[3],
            )

            a_ang_a = self.Mq_rev @ (torque_local - self.Dq @ self.w_a - self.Kq @ q_error[0:3])
            self.w_a = self.w_a + a_ang_a * self.dt

            q_a_dot = 0.5 * quat_multiply(
                self.q_a[0], self.q_a[1], self.q_a[2], self.q_a[3],
                self.w_a[0], self.w_a[1], self.w_a[2], 0,
            )
            self.q_a = self.q_a + q_a_dot * self.dt
            self.q_a = self.q_a / np.linalg.norm(self.q_a)
            # fmt: on

            multi_dof_joint_traj = MultiDOFJointTrajectory()
            multi_dof_joint_traj.header.frame_id = "world"
            multi_dof_joint_traj.header.stamp = rospy.Time.now()
            multi_dof_joint_traj.joint_names.append(frame_name)

            time_from_start = rospy.Time.now().to_sec() - self.start_time
            for _ in range(self.N_nmpc + 1):
                traj_pt = MultiDOFJointTrajectoryPoint()
                traj_pt.transforms.append(
                    Transform(
                        translation=Vector3(self.p_a[0], self.p_a[1], self.p_a[2]),
                        rotation=Quaternion(self.q_a[0], self.q_a[1], self.q_a[2], self.q_a[3]),
                    )
                )
                traj_pt.velocities.append(
                    Twist(
                        linear=Vector3(self.v_a[0], self.v_a[1], self.v_a[2]),
                        angular=Vector3(self.w_a[0], self.w_a[1], self.w_a[2]),
                    )
                )
                traj_pt.accelerations.append(
                    Twist(
                        linear=Vector3(a_lin_a[0], a_lin_a[1], a_lin_a[2]),
                        angular=Vector3(a_ang_a[0], a_ang_a[1], a_ang_a[2]),
                    )
                )
                traj_pt.time_from_start = rospy.Duration.from_sec(time_from_start)
                multi_dof_joint_traj.points.append(traj_pt)

            self.pub_ref_traj.publish(multi_dof_joint_traj)
            self.rate.sleep()

        # shutdown the timer
        if self.param_timer is not None:
            self.param_timer.shutdown()
            self.param_timer = None

        return "done_admittance"

    def _sub_odom_callback(self, msg):
        self.odom_msg = msg

    def _sub_ext_wrench_body_frame_callback(self, msg):
        self.ext_wrench_body_frame_msg = msg

    def compute_wrench(self):
        ext_force_b = np.array(
            [
                self.ext_wrench_body_frame_msg.wrench.force.x,
                self.ext_wrench_body_frame_msg.wrench.force.y,
                self.ext_wrench_body_frame_msg.wrench.force.z,
            ]
        )

        ext_torque_b = np.array(
            [
                self.ext_wrench_body_frame_msg.wrench.torque.x,
                self.ext_wrench_body_frame_msg.wrench.torque.y,
                self.ext_wrench_body_frame_msg.wrench.torque.z,
            ]
        )

        rot_w_b = R.from_quat(
            [
                self.odom_msg.pose.pose.orientation.x,
                self.odom_msg.pose.pose.orientation.y,
                self.odom_msg.pose.pose.orientation.z,
                self.odom_msg.pose.pose.orientation.w,
            ]
        ).as_matrix()

        force_w = rot_w_b @ ext_force_b

        if self.frame_type == "ee":
            torque_local = self.rot_body_ee.T @ (-skew(self.p_ee_in_body) @ ext_force_b + ext_torque_b)
        else:
            torque_local = ext_torque_b

        return force_w, torque_local

    def publish_wrench(self, force_w, torque_local):
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()

        wrench_msg.wrench.force.x = force_w[0]
        wrench_msg.wrench.force.y = force_w[1]
        wrench_msg.wrench.force.z = force_w[2]

        wrench_msg.wrench.torque.x = torque_local[0]
        wrench_msg.wrench.torque.y = torque_local[1]
        wrench_msg.wrench.torque.z = torque_local[2]

        self.admittance_wrench_pub.publish(wrench_msg)

    @staticmethod
    def _safe_get_param(name, default):
        try:
            return rospy.get_param(name, default)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Failed to get param {name}: {e}")
            return default

    def _read_param_dict(self):
        ns = self.param_ns_admittance

        param_dict = {
            "force_thresh": float(self._safe_get_param(f"{ns}/force_thresh", 0.5)),
            "torque_thresh": float(self._safe_get_param(f"{ns}/torque_thresh", 0.1)),
            "Mp": list(self._safe_get_param(f"{ns}/Mp", [3.0, 3.0, 3.0])),
            "Dp": list(self._safe_get_param(f"{ns}/Dp", [10.0, 10.0, 10.0])),
            "Kp": list(self._safe_get_param(f"{ns}/Kp", [20.0, 20.0, 20.0])),
            "Mq": list(self._safe_get_param(f"{ns}/Mq", [0.5, 0.5, 0.5])),
            "Dq": list(self._safe_get_param(f"{ns}/Dq", [10.0, 10.0, 10.0])),
            "Kq": list(self._safe_get_param(f"{ns}/Kq", [15.0, 15.0, 15.0])),
        }

        return param_dict

    def _validate_and_apply_params(self, p):
        def _check_vec(name, vec, positive=False, nonnegative=False):
            if not isinstance(vec, (list, tuple)) or len(vec) != 3:
                raise ValueError(f"{name} must be a length-3 list, got {vec}")
            vec = np.array(vec, dtype=float)
            if positive and np.any(vec <= 0.0):
                raise ValueError(f"{name} must be > 0, got {vec}")
            if nonnegative and np.any(vec < 0.0):
                raise ValueError(f"{name} must be >= 0, got {vec}")
            return vec

        force_thresh = float(p["force_thresh"])
        torque_thresh = float(p["torque_thresh"])

        if force_thresh < 0.0:
            raise ValueError(f"force_thresh must be >= 0, got {force_thresh}")
        if torque_thresh < 0.0:
            raise ValueError(f"torque_thresh must be >= 0, got {torque_thresh}")

        Mp = _check_vec("Mp", p["Mp"], positive=True)
        Dp = _check_vec("Dp", p["Dp"], nonnegative=True)
        Kp = _check_vec("Kp", p["Kp"], nonnegative=True)

        Mq = _check_vec("Mq", p["Mq"], positive=True)
        Dq = _check_vec("Dq", p["Dq"], nonnegative=True)
        Kq = _check_vec("Kq", p["Kq"], nonnegative=True)

        self.force_thresh = force_thresh
        self.torque_thresh = torque_thresh

        self.Mp = np.diag(Mp)
        self.Mp_rev = np.linalg.inv(self.Mp)
        self.Dp = np.diag(Dp)
        self.Kp = np.diag(Kp)

        self.Mq = np.diag(Mq)
        self.Mq_rev = np.linalg.inv(self.Mq)
        self.Dq = np.diag(Dq)
        self.Kq = np.diag(Kq)

    def _update_params_if_needed(self, event=None):
        try:
            p = self._read_param_dict()
            signature = repr(p)  # return the str format of p

            if signature == self._last_param_signature:
                return

            # === safety concern ===
            # the position will not jump to the original position, but the orientation will.
            self.p_r = np.array(
                [
                    self.odom_msg.pose.pose.position.x,
                    self.odom_msg.pose.pose.position.y,
                    self.odom_msg.pose.pose.position.z,
                ]
            )
            # ======================

            self._validate_and_apply_params(p)
            self._last_param_signature = signature

            rospy.loginfo(
                f"[Admittance] Parameters updated from {self.param_ns_admittance}: "
                f"Mp={p['Mp']}, Dp={p['Dp']}, Kp={p['Kp']}, "
                f"Mq={p['Mq']}, Dq={p['Dq']}, Kq={p['Kq']}, "
                f"force_thresh={p['force_thresh']}, torque_thresh={p['torque_thresh']}"
            )

        except Exception as e:
            rospy.logerr_throttle(2.0, f"[Admittance] Invalid parameter update ignored: {e}")


def create_admittance_state_machine():
    sm_sub = smach.StateMachine(outcomes=["DONE_ADMITTANCE"], input_keys=["robot_name"])

    with sm_sub:
        smach.StateMachine.add(
            "ADMITTANCE", AdmittanceState(frame_type="ee"), transitions={"done_admittance": "DONE_ADMITTANCE"}
        )

    return sm_sub
