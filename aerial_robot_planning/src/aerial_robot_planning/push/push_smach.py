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

from dataclasses import dataclass
from typing import List, Tuple, Optional, Sequence

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
# Target / task definition (hard-coded list)
# ======================================================================================
DEFAULT_BODY_Z_ROTATION = -np.pi / 4


@dataclass
class PushTarget:
    """A single contact point to push against.

    ``approach_dir`` (the surface inward normal followed during the push) is derived
    as the unit vector ``contact_pos - standoff_pos``; the over-shoot used to generate
    the force is applied along this direction. The desired attitude is generated from
    this same line, with Body-Z aligned to ``approach_dir`` and ``body_z_rotation``
    applied around Body-Z.
    """

    standoff_pos: Tuple[float, float, float]  # (x, y, z) off-contact approach point [m]
    contact_pos: Tuple[float, float, float]  # (x, y, z) nominal contact point [m]
    desired_force: float  # [N]
    body_z_rotation: float = DEFAULT_BODY_Z_ROTATION  # [rad], right-hand rotation about Body-Z
    k_p: float = 20.0  # must match the controller's effective stiffness (trajs.py:847 note)


@dataclass
class PushTask:
    """A selectable push task, chosen after entering Push Mode."""

    name: str
    targets: Sequence[PushTarget]


# The first target reproduces the original PushWallTraj attitude because the default
# Body-Z rotation is -pi/4 for a +X approach. Add more PushTask entries for new scenes.
THREE_PUSH_TARGETS: Tuple[PushTarget, ...] = (
    PushTarget(
        standoff_pos=(0.0, 1.0, 1.2),
        contact_pos=(1.1, 1.0, 1.2),
        desired_force=30.0,
        k_p=20.0,
    ),
    PushTarget(
        standoff_pos=(0.0, 1.0, 1.5),
        contact_pos=(1.1, 1.0, 1.5),
        desired_force=20.0,
        k_p=20.0,
    ),
    PushTarget(
        standoff_pos=(0.0, 1.2, 1.5),
        contact_pos=(1.1, 1.2, 1.5),
        desired_force=25.0,
        k_p=20.0,
    ),
)

DEFAULT_PUSH_TASKS: Tuple[PushTask, ...] = (
    PushTask("single target", (THREE_PUSH_TARGETS[0],)),
    PushTask("three targets", THREE_PUSH_TARGETS),
)


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


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def _approach_dir_for(target: PushTarget) -> np.ndarray:
    approach_dir = np.array(target.contact_pos, dtype=float) - np.array(target.standoff_pos, dtype=float)
    n = np.linalg.norm(approach_dir)
    if n <= 1e-9:
        raise ValueError("PushTarget contact_pos and standoff_pos are identical; cannot derive Body-Z direction.")
    return approach_dir / n


def _quat_for_target(target: PushTarget, body_z_rotation_override: Optional[float] = None) -> np.ndarray:
    """Build world<-body quaternion with Body-Z along standoff->contact."""
    body_z = _approach_dir_for(target)
    ref_axis = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(ref_axis, body_z))) > 0.95:
        ref_axis = np.array([1.0, 0.0, 0.0])

    body_x = _unit(np.cross(ref_axis, body_z))
    body_y = np.cross(body_z, body_x)
    rot_w_b = np.column_stack((body_x, body_y, body_z))

    body_z_rotation = target.body_z_rotation if body_z_rotation_override is None else body_z_rotation_override
    rot_w_b = rot_w_b @ R.from_euler("z", body_z_rotation).as_matrix()
    return R.from_matrix(rot_w_b).as_quat()


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

    def __init__(
        self,
        push_tasks: Sequence[PushTask],
        fixed_push_targets: Optional[Sequence[PushTarget]] = None,
    ):
        self.push_tasks = list(push_tasks)
        self.fixed_push_targets = list(fixed_push_targets) if fixed_push_targets is not None else None
        self.push_targets: List[PushTarget] = list(fixed_push_targets) if fixed_push_targets is not None else []
        self.robot_name: Optional[str] = None
        self.io: Optional[PushIO] = None
        self.target_idx = 0
        self.contact_p: Optional[np.ndarray] = None
        self.body_z_rotation_override: Optional[float] = None

    def select_task(self, task_idx: int) -> None:
        task = self.push_tasks[task_idx]
        self.push_targets = list(task.targets)


# ======================================================================================
# Base state: shared parameters, seeded from the original PushWallTraj timeline.
# ======================================================================================
class PushBaseState(smach.State):
    dt = 0.02  # 50 Hz reference loop

    # alignment / calibration [s]
    T_ALIGN_TIMEOUT = 20.0
    T_CALIBRATE_SETTLE = 3.0

    # approach / retreat: the reference advances at a constant speed (not a fixed time)
    APPROACH_SPEED = 0.2  # [m/s]
    APPROACH_TIMEOUT = 15.0  # [s]
    RETREAT_SPEED = 0.2  # [m/s]
    RETREAT_TIMEOUT = 15.0  # [s]

    # force application [s]
    T_ACCUM_FORCE = 2.5  # measure the initial contact force
    T_FORCE_RAMP = 3.0  # same duration for ramp-up and ramp-down
    T_FORCE_HOLD = 5.0  # hold at desired force between ramp-up and ramp-down

    # thresholds
    FORCE_THRESH = 0.5  # [N] minimum force counted as contact
    ALIGN_POS_TOL = 0.1  # [m]
    ALIGN_ANG_TOL = 0.2  # [rad]
    ALIGN_VEL_TOL = 0.1  # [m/s]

    def __init__(self, ctx: PushContext, outcomes, input_keys=None, output_keys=None):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        self.ctx = ctx
        self.rate = rospy.Rate(1.0 / self.dt)

    def _move_at_speed(self, io, p_from, p_to, q, speed, timeout, detect_contact=False, approach_dir=None):
        """Advance the position reference from p_from to p_to at a constant speed.

        Returns the position where the move stopped. Breaks when the segment end is
        reached, on timeout, or -- if ``detect_contact`` -- when the contact force along
        ``approach_dir`` exceeds ``FORCE_THRESH`` (returns the early-contact position).
        """
        p_from = np.array(p_from, dtype=float)
        p_to = np.array(p_to, dtype=float)
        seg = p_to - p_from
        total = float(np.linalg.norm(seg))
        direction = seg / total if total > 1e-9 else np.zeros(3)

        p = p_from.copy()
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t_start
            dist = min(speed * t, total)
            p = p_from + direction * dist
            io.publish_pose_ref(p, q)

            if detect_contact and io.contact_force_along(approach_dir) > self.FORCE_THRESH:
                rospy.loginfo("PUSH: contact detected at %s.", np.round(p, 3).tolist())
                return p

            if dist >= total:
                return p
            if t > timeout:
                rospy.logwarn("PUSH: move timeout (%.1fs) before reaching target, stopping here.", timeout)
                return p
            self.rate.sleep()

        return p


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
        self.ctx.robot_name = userdata.robot_name
        self.ctx.body_z_rotation_override = rospy.get_param("~push_body_z_rotation", None)
        self.ctx.body_z_rotation_override = rospy.get_param(
            f"/{userdata.robot_name}/push/body_z_rotation", self.ctx.body_z_rotation_override
        )
        if self.ctx.body_z_rotation_override is not None:
            try:
                self.ctx.body_z_rotation_override = float(self.ctx.body_z_rotation_override)
                rospy.loginfo(
                    "PUSH/INIT: overriding all target body_z_rotation with %.3f rad.",
                    self.ctx.body_z_rotation_override,
                )
            except (TypeError, ValueError):
                rospy.logwarn("PUSH/INIT: invalid body_z_rotation parameter; using per-target values.")
                self.ctx.body_z_rotation_override = None

        if self.ctx.fixed_push_targets is None:
            print("\n===== Push Tasks =====")
            for i, task in enumerate(self.ctx.push_tasks, start=1):
                print(f"{i}: {task.name} ({len(task.targets)} target(s))")
            try:
                task_str = input("\nEnter push task number: ")
                task_idx = int(task_str) - 1
            except (ValueError, EOFError):
                rospy.logwarn("PUSH/INIT: invalid push task input.")
                return "failed"

            if task_idx < 0 or task_idx >= len(self.ctx.push_tasks):
                rospy.logwarn("PUSH/INIT: push task index out of range.")
                return "failed"
            self.ctx.select_task(task_idx)
            rospy.loginfo(
                "PUSH/INIT: selected task %d (%s), %d target(s).",
                task_idx + 1,
                self.ctx.push_tasks[task_idx].name,
                len(self.ctx.push_targets),
            )

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
        q = _quat_for_target(target, self.ctx.body_z_rotation_override)

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
        q = _quat_for_target(target, self.ctx.body_z_rotation_override)

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
        approach_dir = _approach_dir_for(target)
        q = _quat_for_target(target, self.ctx.body_z_rotation_override)

        rospy.loginfo(
            "PUSH/APPROACH: moving toward contact %s at %.2f m/s ...", np.round(p1, 3).tolist(), self.APPROACH_SPEED
        )
        contact_p = self._move_at_speed(
            io, p0, p1, q, self.APPROACH_SPEED, self.APPROACH_TIMEOUT, detect_contact=True, approach_dir=approach_dir
        )

        self.ctx.contact_p = contact_p
        return "contact"


class ApplyForceState(PushBaseState):
    """Accumulate the initial contact force, then over-shoot to reach desired_force."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["applied"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        approach_dir = _approach_dir_for(target)
        q = _quat_for_target(target, self.ctx.body_z_rotation_override)
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
        rospy.loginfo(
            "PUSH/APPLY: init_force=%.2f N, delta=%.3f m, applying desired_force=%.1f N.",
            init_force,
            delta,
            target.desired_force,
        )

        # 2) ramp-up p_contact -> p_final, 3) hold, 4) ramp-down p_final -> p_contact.
        # ramp-up and ramp-down use the same duration (T_FORCE_RAMP). Ends back at p_contact.
        t_ramp_down_start = self.T_FORCE_RAMP + self.T_FORCE_HOLD
        t_total = t_ramp_down_start + self.T_FORCE_RAMP
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t_start
            if t >= t_total:
                break
            if t < self.T_FORCE_RAMP:  # ramp-up
                p = p_contact + (p_final - p_contact) * (t / self.T_FORCE_RAMP)
            elif t < t_ramp_down_start:  # hold
                p = p_final
            else:  # ramp-down
                tau = t - t_ramp_down_start
                p = p_final + (p_contact - p_final) * (tau / self.T_FORCE_RAMP)
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
        q = _quat_for_target(target, self.ctx.body_z_rotation_override)
        p_start = np.array(self.ctx.contact_p, dtype=float) if self.ctx.contact_p is not None else p_standoff

        rospy.loginfo(
            "PUSH/RETREAT: returning to standoff %s at %.2f m/s ...",
            np.round(p_standoff, 3).tolist(),
            self.RETREAT_SPEED,
        )
        self._move_at_speed(io, p_start, p_standoff, q, self.RETREAT_SPEED, self.RETREAT_TIMEOUT)

        self.ctx.target_idx += 1
        self.ctx.contact_p = None
        return "retreated"


# ======================================================================================
# Factory
# ======================================================================================
def create_push_state_machine(
    push_targets: Optional[Sequence[PushTarget]] = None,
    push_tasks: Optional[Sequence[PushTask]] = None,
):
    # Only robot_name (picklable) is kept in userdata so the smach_ros introspection
    # server can pickle it; all shared/non-picklable/loop state lives in PushContext.
    sm_sub = smach.StateMachine(outcomes=["DONE_PUSH"], input_keys=["robot_name"])

    ctx = PushContext(push_tasks if push_tasks is not None else DEFAULT_PUSH_TASKS, fixed_push_targets=push_targets)

    with sm_sub:
        smach.StateMachine.add("INIT_PUSH", InitPushState(ctx), transitions={"start": "SELECT", "failed": "DONE_PUSH"})
        smach.StateMachine.add("SELECT", SelectState(ctx), transitions={"next": "ALIGN", "finished": "DONE_PUSH"})
        smach.StateMachine.add("ALIGN", AlignState(ctx), transitions={"aligned": "CALIBRATE"})
        smach.StateMachine.add("CALIBRATE", CalibrateState(ctx), transitions={"calibrated": "APPROACH"})
        smach.StateMachine.add("APPROACH", ApproachState(ctx), transitions={"contact": "APPLY"})
        smach.StateMachine.add("APPLY", ApplyForceState(ctx), transitions={"applied": "RETREAT"})
        smach.StateMachine.add("RETREAT", RetreatState(ctx), transitions={"retreated": "SELECT"})

    return sm_sub
