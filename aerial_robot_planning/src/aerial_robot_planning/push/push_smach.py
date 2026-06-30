"""
 Created by jinjie on 2026/06/30.

 Push sub-state-machine: repeatedly apply a desired contact force at a list of
 contact points on a (possibly curved) surface.

 This is the reusable, looping refactor of the one-shot ``PushWallTraj`` demo in
 ``aerial_robot_planning/trajs.py``. The original timeline (align -> calibrate ->
 approach -> accumulate contact force -> apply extra force -> retreat) is split
 into fine-grained smach states that publish references directly (like
 ``AdmittanceState``) and loop over a hard-coded list of ``PushTarget`` s.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional

import numpy as np
import rospy
import smach
import tf_conversions as tf
from scipy.spatial.transform import Rotation as R

from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, Transform, Twist, Quaternion, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from ..util import topic_ready


# ======================================================================================
# Target definition (hard-coded list)
# ======================================================================================
@dataclass
class PushTarget:
    """A single contact point to push against.

    ``approach_dir`` (the surface inward normal followed during the push) is derived
    as the unit vector ``contact_pos - standoff_pos``; the over-shoot used to generate
    the force is applied along this direction. ``orientation`` should make the
    end-effector face the local surface normal.
    """

    standoff_pos: Tuple[float, float, float]  # (x, y, z) off-contact approach point [m]
    contact_pos: Tuple[float, float, float]  # (x, y, z) nominal contact point [m]
    orientation: Tuple[float, float, float]  # (roll, pitch, yaw) [rad], axes="rxyz"
    desired_force: float  # [N]
    k_p: float = 20.0  # must match the controller's effective stiffness (trajs.py:847 note)


# The first entry reproduces the original PushWallTraj demo. Add more entries to push
# repeatedly across a curved surface / large plane.
DEFAULT_PUSH_TARGETS: List[PushTarget] = [
    PushTarget(
        standoff_pos=(0.0, 1.0, 1.2),
        contact_pos=(1.1, 1.0, 1.2),
        orientation=(0.0, np.pi / 2, np.pi / 4),
        desired_force=10.0,
        k_p=20.0,
    ),
]


# ======================================================================================
# Shared IO / helpers (created once, shared between states via userdata)
# ======================================================================================
class PushIO:
    """Holds the ROS pub/sub and reference-building helpers shared by all push states."""

    def __init__(self, robot_name: str, topic_timeout: float = 5.0):
        self.robot_name = robot_name

        self.N_nmpc = rospy.get_param(f"{robot_name}/controller/nmpc/NN")
        self.T_step = rospy.get_param(f"{robot_name}/controller/nmpc/T_step")

        self.odom_msg = Odometry()
        self.ext_wrench_msg = WrenchStamped()

        self.pub_ref_traj = rospy.Publisher(f"/{robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=3)

        odom_topic = f"/{robot_name}/uav/ee_contact/odom"
        if not topic_ready(odom_topic, Odometry, timeout=topic_timeout):
            raise RuntimeError(f"Topic {odom_topic} not available.")
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self._odom_cb)

        ext_topic = f"/{robot_name}/ext_wrench_est/value"
        if not topic_ready(ext_topic, WrenchStamped, timeout=topic_timeout):
            raise RuntimeError(f"Topic {ext_topic} not available.")
        self.ext_wrench_sub = rospy.Subscriber(ext_topic, WrenchStamped, self._wrench_cb)

    def _odom_cb(self, msg: Odometry):
        self.odom_msg = msg

    def _wrench_cb(self, msg: WrenchStamped):
        self.ext_wrench_msg = msg

    # ---- reference publishing ----
    def publish_pose_ref(self, p_xyz: np.ndarray, q_xyzw: np.ndarray) -> None:
        """Publish a constant pose reference over the whole NMPC horizon (ee frame)."""
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        traj.joint_names.append("ee")

        for i in range(self.N_nmpc + 1):
            pt = MultiDOFJointTrajectoryPoint()
            pt.transforms.append(
                Transform(
                    translation=Vector3(p_xyz[0], p_xyz[1], p_xyz[2]),
                    rotation=Quaternion(q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3]),
                )
            )
            pt.velocities.append(Twist())
            pt.accelerations.append(Twist())
            pt.time_from_start = rospy.Duration.from_sec(i * self.T_step)
            traj.points.append(pt)

        self.pub_ref_traj.publish(traj)

    # ---- state readout ----
    def get_position(self) -> np.ndarray:
        p = self.odom_msg.pose.pose.position
        return np.array([p.x, p.y, p.z])

    def get_orientation_xyzw(self) -> np.ndarray:
        q = self.odom_msg.pose.pose.orientation
        return np.array([q.x, q.y, q.z, q.w])

    def get_linear_velocity(self) -> np.ndarray:
        v = self.odom_msg.twist.twist.linear
        return np.array([v.x, v.y, v.z])

    def contact_force_along(self, approach_dir: np.ndarray) -> float:
        """Contact force magnitude along the push direction (positive when in contact).

        The body-frame external force is rotated to world (like AdmittanceState) and
        projected onto ``approach_dir``. The measured reaction opposes the push, so we
        negate to make contact a positive value (matching PushWallTraj's sign).
        """
        f = self.ext_wrench_msg.wrench.force
        ext_force_b = np.array([f.x, f.y, f.z])
        rot_w_b = R.from_quat(self.get_orientation_xyzw()).as_matrix()
        force_w = rot_w_b @ ext_force_b
        return float(-np.dot(force_w, approach_dir))

    def reached_pose(
        self, p_xyz: np.ndarray, q_xyzw: np.ndarray, pos_tol: float, ang_tol: float, vel_tol: float
    ) -> bool:
        pos_err = float(np.linalg.norm(self.get_position() - p_xyz))
        vel_err = float(np.linalg.norm(self.get_linear_velocity()))

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.get_orientation_xyzw())
        roll_r, pitch_r, yaw_r = tf.transformations.euler_from_quaternion(q_xyzw)
        ang_err = float(np.linalg.norm([roll - roll_r, pitch - pitch_r, yaw - yaw_r]))

        return pos_err < pos_tol and ang_err < ang_tol and vel_err < vel_tol


def _quat_of(orientation_rpy: Tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = orientation_rpy
    qx, qy, qz, qw = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes="rxyz")
    return np.array([qx, qy, qz, qw])


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


# ======================================================================================
# Shared context (NOT stored in userdata, so the introspection server can pickle userdata)
# ======================================================================================
class PushContext:
    """Mutable state shared across all push states.

    Kept out of smach userdata on purpose: ``io`` holds ROS pub/sub objects (with
    ``_thread.RLock``) that the smach_ros introspection server cannot pickle. Looping
    state (``target_idx``) also lives here and is reset by ``InitPushState`` on every
    entry to the sub-state-machine.
    """

    def __init__(self, push_targets: List[PushTarget]):
        self.push_targets = push_targets
        self.robot_name: Optional[str] = None
        self.io: Optional[PushIO] = None
        self.target_idx = 0
        self.contact_p: Optional[np.ndarray] = None
        self.release_p: Optional[np.ndarray] = None


# ======================================================================================
# Base state: shared parameters, seeded from the original PushWallTraj timeline.
# ======================================================================================
class PushBaseState(smach.State):
    dt = 0.02  # 50 Hz reference loop

    # durations [s]
    T_CALIBRATE_SETTLE = 3.0
    T_MOVE_TO_WALL = 3.0
    T_ACCUM_FORCE = 2.5
    T_GRADUAL = 3.0  # ramp-up of the extra force (included in T_APPLY)
    T_APPLY = 10.0  # total time at/above the wall applying force
    T_MOVE_BACK = 3.0
    T_ALIGN_TIMEOUT = 20.0

    # thresholds
    FORCE_THRESH = 0.5  # [N] minimum force counted as contact
    ALIGN_POS_TOL = 0.1  # [m]
    ALIGN_ANG_TOL = 0.2  # [rad]
    ALIGN_VEL_TOL = 0.1  # [m/s]

    def __init__(self, ctx: PushContext, outcomes, input_keys=None, output_keys=None):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        self.ctx = ctx
        self.rate = rospy.Rate(1.0 / self.dt)


# ======================================================================================
# States
# ======================================================================================
class InitPushState(PushBaseState):
    """Initial state: (re)set the loop and lazily create the shared IO.

    Because a nested smach container always restarts at its initial state when entered,
    this resets ``target_idx`` every time the user re-enters PUSH from IDLE.
    """

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["start", "failed"], input_keys=["robot_name"])

    def execute(self, userdata):
        self.ctx.target_idx = 0
        self.ctx.contact_p = None
        self.ctx.release_p = None
        self.ctx.robot_name = userdata.robot_name

        if self.ctx.io is None:
            rospy.loginfo("PUSH/INIT: initializing IO ...")
            try:
                self.ctx.io = PushIO(userdata.robot_name)
            except Exception as e:
                rospy.logerr(f"PUSH/INIT: failed to initialize IO: {e}")
                return "failed"

        return "start"


class SelectState(PushBaseState):
    """Decide whether another target remains."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["next", "finished"])

    def execute(self, userdata):
        idx = self.ctx.target_idx
        n = len(self.ctx.push_targets)
        if idx >= n:
            rospy.loginfo("PUSH/SELECT: all %d target(s) done.", n)
            return "finished"

        rospy.loginfo("PUSH/SELECT: starting target %d / %d.", idx + 1, n)
        return "next"


class AlignState(PushBaseState):
    """Move to the standoff pose with the end-effector facing the surface normal."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["aligned"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        p = np.array(target.standoff_pos, dtype=float)
        q = _quat_of(target.orientation)

        rospy.loginfo("PUSH/ALIGN: moving to standoff %s ...", np.round(p, 3).tolist())
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            io.publish_pose_ref(p, q)
            if io.reached_pose(p, q, self.ALIGN_POS_TOL, self.ALIGN_ANG_TOL, self.ALIGN_VEL_TOL):
                rospy.loginfo("PUSH/ALIGN: standoff pose reached.")
                break
            if rospy.Time.now().to_sec() - t_start > self.T_ALIGN_TIMEOUT:
                rospy.logwarn("PUSH/ALIGN: timeout, proceeding anyway.")
                break
            self.rate.sleep()

        return "aligned"


class CalibrateState(PushBaseState):
    """Calibrate the wrench estimator while off-contact (runs before every push)."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["calibrated"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        p = np.array(target.standoff_pos, dtype=float)
        q = _quat_of(target.orientation)

        service_name = f"/{self.ctx.robot_name}/controller/wrench_est/calibrate"
        try:
            rospy.loginfo("PUSH/CALIBRATE: waiting for %s ...", service_name)
            rospy.wait_for_service(service_name, timeout=3.0)
            rospy.ServiceProxy(service_name, Trigger)()
            rospy.loginfo("PUSH/CALIBRATE: calibration succeeded.")
        except rospy.ROSException as e:
            rospy.logwarn(f"PUSH/CALIBRATE: service wait timeout for {service_name}: {e}")
        except rospy.ServiceException as e:
            rospy.logwarn(f"PUSH/CALIBRATE: failed to call {service_name}: {e}")

        # hold the standoff pose while the estimate settles
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t_start < self.T_CALIBRATE_SETTLE:
            io.publish_pose_ref(p, q)
            self.rate.sleep()

        return "calibrated"


class ApproachState(PushBaseState):
    """Ramp the reference from standoff to the contact pose until contact is sensed."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["contact"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        p0 = np.array(target.standoff_pos, dtype=float)
        p1 = np.array(target.contact_pos, dtype=float)
        approach_dir = _unit(p1 - p0)
        q = _quat_of(target.orientation)

        rospy.loginfo("PUSH/APPROACH: moving toward contact %s ...", np.round(p1, 3).tolist())
        contact_p = p1.copy()  # fall back to the nominal contact point
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t_start
            ratio = min(t / self.T_MOVE_TO_WALL, 1.0)
            p = p0 + (p1 - p0) * ratio
            io.publish_pose_ref(p, q)

            # early contact (e.g. surface closer than nominal on a curved surface)
            if io.contact_force_along(approach_dir) > self.FORCE_THRESH:
                contact_p = p.copy()
                rospy.loginfo("PUSH/APPROACH: contact detected at %s.", np.round(contact_p, 3).tolist())
                break

            if ratio >= 1.0:
                rospy.loginfo("PUSH/APPROACH: reached nominal contact point.")
                break
            self.rate.sleep()

        self.ctx.contact_p = contact_p
        return "contact"


class ApplyForceState(PushBaseState):
    """Accumulate the initial contact force, then over-shoot to reach desired_force."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["applied"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        approach_dir = _unit(np.array(target.contact_pos, dtype=float) - np.array(target.standoff_pos, dtype=float))
        q = _quat_of(target.orientation)
        p_contact = np.array(self.ctx.contact_p, dtype=float)

        # 1) accumulate the initial contact force while holding p_contact
        rospy.loginfo("PUSH/APPLY: accumulating initial contact force ...")
        force_sum, force_num = 0.0, 0
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t_start < self.T_ACCUM_FORCE:
            io.publish_pose_ref(p_contact, q)
            f = io.contact_force_along(approach_dir)
            if abs(f) > self.FORCE_THRESH:
                force_sum += f
                force_num += 1
            self.rate.sleep()

        init_force = force_sum / force_num if force_num > 0 else 0.0
        delta = (target.desired_force - init_force) / target.k_p
        p_final = p_contact + delta * approach_dir
        self.ctx.release_p = p_final
        rospy.loginfo(
            "PUSH/APPLY: init_force=%.2f N, delta=%.3f m, applying desired_force=%.1f N.",
            init_force,
            delta,
            target.desired_force,
        )

        # 2) ramp to p_final, then 3) hold for the rest of T_APPLY
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t_start
            if t >= self.T_APPLY:
                break
            if t < self.T_GRADUAL:
                p = p_contact + (p_final - p_contact) * (t / self.T_GRADUAL)
            else:
                p = p_final
            io.publish_pose_ref(p, q)
            self.rate.sleep()

        return "applied"


class RetreatState(PushBaseState):
    """Unload the force by ramping back to the standoff pose, then advance the index."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["retreated"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        p_standoff = np.array(target.standoff_pos, dtype=float)
        q = _quat_of(target.orientation)
        p_start = np.array(self.ctx.release_p, dtype=float) if self.ctx.release_p is not None else p_standoff

        rospy.loginfo("PUSH/RETREAT: unloading and returning to standoff ...")
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t_start
            ratio = min(t / self.T_MOVE_BACK, 1.0)
            p = p_start + (p_standoff - p_start) * ratio
            io.publish_pose_ref(p, q)
            if ratio >= 1.0:
                break
            self.rate.sleep()

        self.ctx.target_idx += 1
        self.ctx.contact_p = None
        self.ctx.release_p = None
        return "retreated"


# ======================================================================================
# Factory
# ======================================================================================
def create_push_state_machine(push_targets: Optional[List[PushTarget]] = None):
    # Only robot_name (picklable) is kept in userdata so the smach_ros introspection
    # server can pickle it; all shared/non-picklable/loop state lives in PushContext.
    sm_sub = smach.StateMachine(outcomes=["DONE_PUSH"], input_keys=["robot_name"])

    ctx = PushContext(push_targets if push_targets is not None else DEFAULT_PUSH_TARGETS)

    with sm_sub:
        smach.StateMachine.add("INIT_PUSH", InitPushState(ctx), transitions={"start": "SELECT", "failed": "DONE_PUSH"})
        smach.StateMachine.add("SELECT", SelectState(ctx), transitions={"next": "ALIGN", "finished": "DONE_PUSH"})
        smach.StateMachine.add("ALIGN", AlignState(ctx), transitions={"aligned": "CALIBRATE"})
        smach.StateMachine.add("CALIBRATE", CalibrateState(ctx), transitions={"calibrated": "APPROACH"})
        smach.StateMachine.add("APPROACH", ApproachState(ctx), transitions={"contact": "APPLY"})
        smach.StateMachine.add("APPLY", ApplyForceState(ctx), transitions={"applied": "RETREAT"})
        smach.StateMachine.add("RETREAT", RetreatState(ctx), transitions={"retreated": "SELECT"})

    return sm_sub
