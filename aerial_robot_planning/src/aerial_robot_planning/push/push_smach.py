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

from collections import deque
from dataclasses import dataclass
import os
import select
import sys
import termios
import threading
import time
from typing import List, Tuple, Optional, Sequence
import tty

import numpy as np
import rospy
import smach
from scipy.spatial.transform import Rotation as R, Slerp

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


@dataclass
class PushTask:
    """A selectable push task, chosen after entering Push Mode."""

    name: str
    targets: Sequence[PushTarget]
    slide_direction_world: Optional[Tuple[float, float, float]] = None
    fixed_attitude_rpy: Optional[Tuple[float, float, float]] = None
    return_to_initial_standoff: bool = False


@dataclass(frozen=True)
class MoveResult:
    """Result of publishing a point-to-point pose ramp."""

    position: np.ndarray
    reason: str


class TerminalBackMonitor:
    """Watch a terminal for one non-blocking retreat key press."""

    def __init__(
        self,
        key: str,
        back_requested: threading.Event,
        input_stream=None,
        poll_period: float = 0.02,
    ):
        self.key = key.lower()
        self.back_requested = back_requested
        self.input_stream = input_stream if input_stream is not None else sys.stdin
        self.poll_period = poll_period
        self._fd = None
        self._saved_termios = None
        self._stop_requested = threading.Event()
        self._thread = None

    def start(self) -> bool:
        """Enter cbreak mode and start monitoring. Return false for non-TTY input."""
        if self._thread is not None and self._thread.is_alive():
            return True

        try:
            self._fd = self.input_stream.fileno()
            if not os.isatty(self._fd):
                rospy.logwarn(
                    "PUSH/INIT: stdin is not a terminal; the '%s' retreat shortcut is unavailable.",
                    self.key,
                )
                self._fd = None
                return False
            self._saved_termios = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd, termios.TCSANOW)
        except (AttributeError, OSError, termios.error) as error:
            rospy.logwarn("PUSH/INIT: failed to enable the retreat shortcut: %s", error)
            self._restore_terminal()
            return False

        self._stop_requested.clear()
        self._thread = threading.Thread(target=self._run, name="push_back_key", daemon=True)
        self._thread.start()
        rospy.loginfo("PUSH: press '%s' at any time to retreat and return to IDLE.", self.key)
        return True

    def _run(self) -> None:
        while not self._stop_requested.is_set() and not rospy.is_shutdown():
            try:
                readable, _, _ = select.select([self._fd], [], [], self.poll_period)
                if not readable:
                    continue
                key = os.read(self._fd, 1).decode(errors="ignore").lower()
            except (OSError, ValueError):
                break

            if key == self.key and not self.back_requested.is_set():
                self.back_requested.set()
                rospy.logwarn("PUSH: operator requested retreat with '%s'.", self.key)

    def stop(self) -> None:
        """Stop monitoring, discard pending keystrokes, and restore terminal settings."""
        self._stop_requested.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=max(0.1, 2.0 * self.poll_period))
        self._thread = None

        if self._fd is not None:
            try:
                while select.select([self._fd], [], [], 0.0)[0]:
                    os.read(self._fd, 1024)
            except (OSError, ValueError):
                pass
        self._restore_terminal()

    def _restore_terminal(self) -> None:
        if self._fd is not None and self._saved_termios is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved_termios)
            except (OSError, termios.error):
                pass
        self._saved_termios = None
        self._fd = None


# The first target reproduces the original PushWallTraj attitude because the default
# Body-Z rotation is -pi/4 for a +X approach. Add more PushTask entries for new scenes.
THREE_PUSH_TARGETS: Tuple[PushTarget, ...] = (
    PushTarget(
        standoff_pos=(0.0, 1.0, 1.2),
        contact_pos=(1.1, 1.0, 1.2),
        desired_force=30.0,
    ),
    PushTarget(
        standoff_pos=(0.0, 1.0, 1.5),
        contact_pos=(1.1, 1.0, 1.5),
        desired_force=20.0,
    ),
    PushTarget(
        standoff_pos=(0.0, 1.2, 1.5),
        contact_pos=(1.1, 1.2, 1.5),
        desired_force=25.0,
    ),
)

PUSH_AND_SLIDE_TARGET = PushTarget(
    standoff_pos=(0.0, 1.0, 1.2),
    contact_pos=(1.1, 1.0, 1.2),
    desired_force=5.0,
)

DEFAULT_PUSH_TASKS: Tuple[PushTask, ...] = (
    PushTask("single target", (THREE_PUSH_TARGETS[0],)),
    PushTask("three targets", THREE_PUSH_TARGETS),
    PushTask(
        "push and slide",
        (PUSH_AND_SLIDE_TARGET,),
        slide_direction_world=(0.0, 0.0, 1.0),
        fixed_attitude_rpy=(0.0, np.pi / 2, 0.0),
        return_to_initial_standoff=True,
    ),
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
        self.wrench_history = deque()
        self.wrench_history_lock = threading.Lock()
        self.wrench_history_duration = rospy.Duration.from_sec(1.0)

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
        force = msg.wrench.force
        force_b = np.array([force.x, force.y, force.z])
        q_xyzw = self.get_orientation_xyzw()
        if np.linalg.norm(q_xyzw) <= 1e-9:
            return
        now = rospy.Time.now()

        with self.wrench_history_lock:
            self.wrench_history.append((now, force_b, q_xyzw))
            cutoff = now - self.wrench_history_duration
            while self.wrench_history and self.wrench_history[0][0] < cutoff:
                self.wrench_history.popleft()

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

    def clear_wrench_history(self) -> None:
        with self.wrench_history_lock:
            self.wrench_history.clear()

    def peak_contact_force_along(self, approach_dir: np.ndarray, window: float) -> float:
        """Return the largest projected contact force received in a recent window."""
        cutoff = rospy.Time.now() - rospy.Duration.from_sec(window)
        with self.wrench_history_lock:
            samples = [
                (force_b.copy(), q_xyzw.copy()) for stamp, force_b, q_xyzw in self.wrench_history if stamp >= cutoff
            ]

        if not samples:
            return self.contact_force_along(approach_dir)

        return max(
            float(-np.dot(R.from_quat(q_xyzw).as_matrix() @ force_b, approach_dir)) for force_b, q_xyzw in samples
        )

    def reached_pose(
        self,
        p_xyz: np.ndarray,
        q_xyzw: np.ndarray,
        pos_tol: float,
        ang_tol: float,
        vel_tol: float,
    ) -> bool:
        pos_err = float(np.linalg.norm(self.get_position() - p_xyz))
        vel_err = float(np.linalg.norm(self.get_linear_velocity()))
        ang_err = _quaternion_angular_distance(self.get_orientation_xyzw(), q_xyzw)

        return pos_err < pos_tol and ang_err < ang_tol and vel_err < vel_tol


def _quaternion_angular_distance(q_a: np.ndarray, q_b: np.ndarray) -> float:
    """Return the shortest rotation angle between two xyzw quaternions."""
    q_a = np.asarray(q_a, dtype=float)
    q_b = np.asarray(q_b, dtype=float)
    norm_a = float(np.linalg.norm(q_a))
    norm_b = float(np.linalg.norm(q_b))
    if norm_a <= 1e-9 or norm_b <= 1e-9:
        return np.inf

    cos_half_angle = abs(float(np.dot(q_a, q_b))) / (norm_a * norm_b)
    return float(2.0 * np.arccos(np.clip(cos_half_angle, 0.0, 1.0)))


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


def _force_reference_at_time(
    elapsed: float,
    p_contact: np.ndarray,
    p_final: np.ndarray,
    slide_velocity_world: np.ndarray,
    ramp_duration: float,
    hold_duration: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return the force-phase position and accumulated slide offset."""
    elapsed = max(float(elapsed), 0.0)
    slide_elapsed = np.clip(elapsed - ramp_duration, 0.0, hold_duration)
    slide_offset = np.asarray(slide_velocity_world, dtype=float) * slide_elapsed

    if elapsed < ramp_duration:
        alpha = elapsed / ramp_duration
        p = p_contact + (p_final - p_contact) * alpha
    elif elapsed < ramp_duration + hold_duration:
        p = p_final + slide_offset
    else:
        alpha = min((elapsed - ramp_duration - hold_duration) / ramp_duration, 1.0)
        p = p_final + (p_contact - p_final) * alpha + slide_offset
    return p, slide_offset


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
        self.selected_task: Optional[PushTask] = (
            PushTask("fixed targets", self.push_targets) if fixed_push_targets is not None else None
        )
        self.robot_name: Optional[str] = None
        self.io: Optional[PushIO] = None
        self.target_idx = 0
        self.contact_p: Optional[np.ndarray] = None
        self.retreat_start_p: Optional[np.ndarray] = None
        self.retreat_standoff_p: Optional[np.ndarray] = None
        self.return_standoff_p: Optional[np.ndarray] = None
        self.finish_after_retreat = False
        self.body_z_rotation_override: Optional[float] = None
        self.max_force_displacement = 0.10
        self.min_force_samples = 20
        self.contact_peak_window = 0.10
        self.contact_confirm_timeout = 0.50
        self.slide_speed = 0.10
        self.back_key = "b"
        self.back_requested = threading.Event()
        self.back_monitor: Optional[TerminalBackMonitor] = None
        rospy.on_shutdown(self.stop_back_monitor)

    def select_task(self, task_idx: int) -> None:
        task = self.push_tasks[task_idx]
        self.selected_task = task
        self.push_targets = list(task.targets)

    def target_quaternion(self, target: PushTarget) -> np.ndarray:
        if self.selected_task is not None and self.selected_task.fixed_attitude_rpy is not None:
            return R.from_euler("xyz", self.selected_task.fixed_attitude_rpy).as_quat()
        return _quat_for_target(target, self.body_z_rotation_override)

    def slide_velocity_world(self) -> np.ndarray:
        if self.selected_task is None or self.selected_task.slide_direction_world is None:
            return np.zeros(3)
        direction = _unit(np.asarray(self.selected_task.slide_direction_world, dtype=float))
        return self.slide_speed * direction

    def prepare_target_retreat(self, target: PushTarget) -> None:
        initial_standoff = np.asarray(target.standoff_pos, dtype=float)
        self.retreat_standoff_p = initial_standoff.copy()
        self.return_standoff_p = (
            initial_standoff.copy()
            if self.selected_task is not None and self.selected_task.return_to_initial_standoff
            else None
        )

    def start_back_monitor(self) -> None:
        self.stop_back_monitor()
        self.back_requested.clear()
        self.back_monitor = TerminalBackMonitor(self.back_key, self.back_requested)
        self.back_monitor.start()

    def stop_back_monitor(self) -> None:
        if self.back_monitor is not None:
            self.back_monitor.stop()
            self.back_monitor = None
        self.back_requested.clear()

    def is_back_requested(self) -> bool:
        return self.back_requested.is_set()

    def consume_back_request(self) -> None:
        self.back_requested.clear()


# ======================================================================================
# Base state: shared parameters, seeded from the original PushWallTraj timeline.
# ======================================================================================
class PushBaseState(smach.State):
    dt = 0.02  # 50 Hz reference loop

    # alignment / calibration [s]
    T_ALIGN_TIMEOUT = 20.0
    T_CALIBRATE_SETTLE = 3.0

    # align: fly-to-standoff ramp from the current pose (position lerp + attitude slerp)
    ALIGN_SPEED = 0.5  # [m/s]
    ALIGN_ANG_SPEED = 0.5  # [rad/s]

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
        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=input_keys or [],
            output_keys=output_keys or [],
        )
        self.ctx = ctx
        self.rate = rospy.Rate(1.0 / self.dt)

    def _move_at_speed(
        self,
        io,
        p_from,
        p_to,
        q,
        speed,
        timeout,
        detect_contact=False,
        approach_dir=None,
        contact_peak_window=0.0,
        q_from=None,
        ang_speed=None,
        allow_back=True,
    ) -> MoveResult:
        """Advance the pose reference from p_from to p_to at a constant speed.

        Returns both the last commanded position and the stop reason. The reason is one
        of ``reached``, ``contact``, ``back_requested``, ``timeout``, ``preempted``,
        or ``shutdown``.

        By default the orientation is held constant at ``q``. If ``q_from`` is given, the
        orientation is slerped from ``q_from`` to ``q`` over the ramp, and progress is
        driven so the ramp also honors ``ang_speed`` -- robust when the position move is
        tiny but the attitude change is large (or vice-versa).
        """
        p_from = np.array(p_from, dtype=float)
        p_to = np.array(p_to, dtype=float)
        seg = p_to - p_from
        total = float(np.linalg.norm(seg))
        direction = seg / total if total > 1e-9 else np.zeros(3)

        slerp = None
        T = total / speed if speed > 1e-9 else 0.0
        if q_from is not None:
            slerp = Slerp(
                [0.0, 1.0],
                R.from_quat([np.asarray(q_from, dtype=float), np.asarray(q, dtype=float)]),
            )
            rel = R.from_quat(q_from).inv() * R.from_quat(q)
            ang_total = float(np.linalg.norm(rel.as_rotvec()))
            T_ang = ang_total / ang_speed if (ang_speed is not None and ang_speed > 1e-9) else 0.0
            T = max(T, T_ang)

        p = p_from.copy()
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                rospy.logwarn("PUSH: move preempted.")
                return MoveResult(p, "preempted")
            if allow_back and self.ctx.is_back_requested():
                return MoveResult(p, "back_requested")

            t = rospy.Time.now().to_sec() - t_start
            s = min(t / T, 1.0) if T > 1e-9 else 1.0
            dist = s * total
            p = p_from + direction * dist
            q_cur = slerp(s).as_quat() if slerp is not None else q
            io.publish_pose_ref(p, q_cur)

            contact_force = io.peak_contact_force_along(approach_dir, contact_peak_window) if detect_contact else 0.0
            if detect_contact and contact_force > self.FORCE_THRESH:
                rospy.loginfo(
                    "PUSH: contact detected at %s (recent peak %.2f N).",
                    np.round(p, 3).tolist(),
                    contact_force,
                )
                return MoveResult(p, "contact")

            if s >= 1.0:
                return MoveResult(p, "reached")
            if t > timeout:
                rospy.logwarn(
                    "PUSH: move timeout (%.1fs) before reaching target, stopping here.",
                    timeout,
                )
                return MoveResult(p, "timeout")
            self.rate.sleep()

        return MoveResult(p, "shutdown")

    def _prepare_operator_retreat(self, position: np.ndarray) -> None:
        self.ctx.consume_back_request()
        self.ctx.retreat_start_p = np.array(position, dtype=float)
        self.ctx.finish_after_retreat = True
        rospy.logwarn("PUSH: entering RETREAT at the operator's request.")


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
        self.ctx.stop_back_monitor()
        self.ctx.target_idx = 0
        self.ctx.contact_p = None
        self.ctx.retreat_start_p = None
        self.ctx.retreat_standoff_p = None
        self.ctx.return_standoff_p = None
        self.ctx.finish_after_retreat = False
        self.ctx.robot_name = userdata.robot_name
        self.ctx.body_z_rotation_override = rospy.get_param("~push_body_z_rotation", None)
        self.ctx.body_z_rotation_override = rospy.get_param(
            f"/{userdata.robot_name}/push/body_z_rotation",
            self.ctx.body_z_rotation_override,
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

        push_param_ns = f"/{userdata.robot_name}/push"
        try:
            self.ctx.max_force_displacement = float(rospy.get_param(f"{push_param_ns}/max_force_displacement", 1.50))
            self.ctx.min_force_samples = int(rospy.get_param(f"{push_param_ns}/min_force_samples", 20))
            self.ctx.contact_peak_window = float(rospy.get_param(f"{push_param_ns}/contact_peak_window", 0.10))
            self.ctx.contact_confirm_timeout = float(rospy.get_param(f"{push_param_ns}/contact_confirm_timeout", 0.50))
            self.ctx.back_key = str(rospy.get_param(f"{push_param_ns}/back_key", "b")).lower()
            slide_speed_param = rospy.get_param(f"{push_param_ns}/slide_speed", 0.10)
        except (TypeError, ValueError):
            rospy.logerr("PUSH/INIT: invalid force safety parameter type.")
            return "failed"

        if not np.isfinite(self.ctx.max_force_displacement) or self.ctx.max_force_displacement <= 0.0:
            rospy.logerr("PUSH/INIT: max_force_displacement must be finite and positive.")
            return "failed"
        if self.ctx.min_force_samples <= 0:
            rospy.logerr("PUSH/INIT: min_force_samples must be positive.")
            return "failed"
        if not np.isfinite(self.ctx.contact_peak_window) or self.ctx.contact_peak_window <= 0.0:
            rospy.logerr("PUSH/INIT: contact_peak_window must be finite and positive.")
            return "failed"
        if not np.isfinite(self.ctx.contact_confirm_timeout) or self.ctx.contact_confirm_timeout <= 0.0:
            rospy.logerr("PUSH/INIT: contact_confirm_timeout must be finite and positive.")
            return "failed"
        if len(self.ctx.back_key) != 1:
            rospy.logerr("PUSH/INIT: back_key must contain exactly one character.")
            return "failed"

        rospy.loginfo(
            "PUSH/INIT: max_force_displacement=%.3f m, min_force_samples=%d, "
            "contact_peak_window=%.3f s, contact_confirm_timeout=%.3f s.",
            self.ctx.max_force_displacement,
            self.ctx.min_force_samples,
            self.ctx.contact_peak_window,
            self.ctx.contact_confirm_timeout,
        )

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

        if self.ctx.selected_task is not None and self.ctx.selected_task.slide_direction_world is not None:
            try:
                self.ctx.slide_speed = float(slide_speed_param)
            except (TypeError, ValueError):
                rospy.logerr("PUSH/INIT: slide_speed must be a finite positive number.")
                return "failed"
            if not np.isfinite(self.ctx.slide_speed) or self.ctx.slide_speed <= 0.0:
                rospy.logerr("PUSH/INIT: slide_speed must be finite and positive.")
                return "failed"
            rospy.loginfo(
                "PUSH/INIT: slide speed=%.3f m/s in world direction %s.",
                self.ctx.slide_speed,
                self.ctx.selected_task.slide_direction_world,
            )

        if self.ctx.io is None:
            rospy.loginfo("PUSH/INIT: initializing IO ...")
            try:
                self.ctx.io = PushIO(userdata.robot_name)
            except Exception as e:
                rospy.logerr(f"PUSH/INIT: failed to initialize IO: {e}")
                return "failed"

        self.ctx.start_back_monitor()
        return "start"


class SelectState(PushBaseState):
    """Decide whether another target remains."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["next", "finished", "retreat", "aborted"])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return "aborted"

        idx = self.ctx.target_idx
        n = len(self.ctx.push_targets)
        if idx >= n:
            rospy.loginfo("PUSH/SELECT: all %d target(s) done.", n)
            return "finished"
        target = self.ctx.push_targets[idx]
        self.ctx.prepare_target_retreat(target)
        if self.ctx.is_back_requested():
            self._prepare_operator_retreat(self.ctx.io.get_position())
            return "retreat"

        rospy.loginfo("PUSH/SELECT: starting target %d / %d.", idx + 1, n)
        return "next"


class AlignState(PushBaseState):
    """Move to the standoff pose with the end-effector facing the surface normal."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["aligned", "retreat", "aborted"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        p = np.array(target.standoff_pos, dtype=float)
        q = self.ctx.target_quaternion(target)

        # Ramp from the current pose to the standoff so the first reference has ~zero
        # tracking error (a far constant setpoint can otherwise diverge the NMPC solver).
        p0 = io.get_position()
        q0 = io.get_orientation_xyzw()
        rospy.loginfo("PUSH/ALIGN: ramping to standoff %s ...", np.round(p, 3).tolist())
        move_result = self._move_at_speed(
            io,
            p0,
            p,
            q,
            self.ALIGN_SPEED,
            self.T_ALIGN_TIMEOUT,
            q_from=q0,
            ang_speed=self.ALIGN_ANG_SPEED,
        )
        if move_result.reason == "back_requested":
            self._prepare_operator_retreat(move_result.position)
            return "retreat"
        if move_result.reason != "reached":
            rospy.logerr("PUSH/ALIGN: failed to reach the standoff (%s).", move_result.reason)
            return "aborted"

        # Settle at the standoff pose so Calibrate/Approach start from a converged state.
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return "aborted"
            if self.ctx.is_back_requested():
                self._prepare_operator_retreat(p)
                return "retreat"

            io.publish_pose_ref(p, q)
            if io.reached_pose(p, q, self.ALIGN_POS_TOL, self.ALIGN_ANG_TOL, self.ALIGN_VEL_TOL):
                rospy.loginfo("PUSH/ALIGN: standoff pose reached.")
                return "aligned"
            if rospy.Time.now().to_sec() - t_start > self.T_ALIGN_TIMEOUT:
                rospy.logerr("PUSH/ALIGN: settle timeout; aborting push.")
                return "aborted"
            self.rate.sleep()

        return "aborted"


class CalibrateState(PushBaseState):
    """Calibrate the wrench estimator while off-contact (runs before every push)."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["calibrated", "retreat", "aborted"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        p = np.array(target.standoff_pos, dtype=float)
        q = self.ctx.target_quaternion(target)

        service_name = f"/{self.ctx.robot_name}/controller/wrench_est/calibrate"
        service_deadline = time.monotonic() + 3.0
        try:
            rospy.loginfo("PUSH/CALIBRATE: waiting for %s ...", service_name)
            while not rospy.is_shutdown():
                if self.ctx.is_back_requested():
                    self._prepare_operator_retreat(p)
                    return "retreat"
                remaining = service_deadline - time.monotonic()
                if remaining <= 0.0:
                    rospy.logerr("PUSH/CALIBRATE: service wait timeout for %s.", service_name)
                    return "aborted"
                try:
                    rospy.wait_for_service(service_name, timeout=min(0.1, remaining))
                    break
                except rospy.ROSException:
                    continue

            if rospy.is_shutdown():
                return "aborted"
            if self.ctx.is_back_requested():
                self._prepare_operator_retreat(p)
                return "retreat"
            response = rospy.ServiceProxy(service_name, Trigger)()
            if self.ctx.is_back_requested():
                self._prepare_operator_retreat(p)
                return "retreat"
            if not response.success:
                rospy.logerr("PUSH/CALIBRATE: calibration rejected: %s", response.message)
                return "aborted"
            rospy.loginfo("PUSH/CALIBRATE: calibration succeeded: %s", response.message)
        except rospy.ServiceException as e:
            rospy.logerr(f"PUSH/CALIBRATE: failed to call {service_name}: {e}")
            return "aborted"

        # hold the standoff pose while the estimate settles
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t_start < self.T_CALIBRATE_SETTLE:
            if self.preempt_requested():
                self.service_preempt()
                return "aborted"
            if self.ctx.is_back_requested():
                self._prepare_operator_retreat(p)
                return "retreat"
            io.publish_pose_ref(p, q)
            self.rate.sleep()

        return "calibrated" if not rospy.is_shutdown() else "aborted"


class ApproachState(PushBaseState):
    """Ramp the reference from standoff to the contact pose until contact is sensed."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["contact", "retreat", "aborted"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        p0 = np.array(target.standoff_pos, dtype=float)
        p1 = np.array(target.contact_pos, dtype=float)
        approach_dir = _approach_dir_for(target)
        q = self.ctx.target_quaternion(target)

        rospy.loginfo(
            "PUSH/APPROACH: moving toward contact %s at %.2f m/s ...",
            np.round(p1, 3).tolist(),
            self.APPROACH_SPEED,
        )
        io.clear_wrench_history()
        move_result = self._move_at_speed(
            io,
            p0,
            p1,
            q,
            self.APPROACH_SPEED,
            self.APPROACH_TIMEOUT,
            detect_contact=True,
            approach_dir=approach_dir,
            contact_peak_window=self.ctx.contact_peak_window,
        )

        self.ctx.retreat_start_p = move_result.position
        if move_result.reason == "back_requested":
            self._prepare_operator_retreat(move_result.position)
            return "retreat"
        if move_result.reason == "reached":
            rospy.loginfo(
                "PUSH/APPROACH: nominal point reached; holding for %.2f s to confirm contact.",
                self.ctx.contact_confirm_timeout,
            )
            t_start = rospy.Time.now().to_sec()
            while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t_start < self.ctx.contact_confirm_timeout:
                if self.preempt_requested():
                    self.service_preempt()
                    self.ctx.finish_after_retreat = True
                    return "aborted"
                if self.ctx.is_back_requested():
                    self._prepare_operator_retreat(move_result.position)
                    return "retreat"

                io.publish_pose_ref(move_result.position, q)
                contact_force = io.peak_contact_force_along(approach_dir, self.ctx.contact_peak_window)
                if contact_force > self.FORCE_THRESH:
                    rospy.loginfo(
                        "PUSH/APPROACH: contact confirmed while holding nominal point " "(recent peak %.2f N).",
                        contact_force,
                    )
                    self.ctx.contact_p = move_result.position
                    return "contact"
                self.rate.sleep()

            current_force = io.contact_force_along(approach_dir)
            recent_peak = io.peak_contact_force_along(approach_dir, self.ctx.contact_peak_window)
            rospy.logerr(
                "PUSH/APPROACH: no contact after confirmation hold "
                "(current %.2f N, recent peak %.2f N); retreating.",
                current_force,
                recent_peak,
            )
            self.ctx.finish_after_retreat = True
            return "aborted"

        if move_result.reason != "contact":
            rospy.logerr(
                "PUSH/APPROACH: stopped without confirmed contact (%s); retreating.",
                move_result.reason,
            )
            self.ctx.finish_after_retreat = True
            return "aborted"

        self.ctx.contact_p = move_result.position
        return "contact"


class ApplyForceState(PushBaseState):
    """Accumulate the initial contact force, then over-shoot to reach desired_force."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["applied", "retreat", "aborted"])

    def execute(self, userdata):
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        approach_dir = _approach_dir_for(target)
        q = self.ctx.target_quaternion(target)
        p_contact = np.array(self.ctx.contact_p, dtype=float)
        p_standoff = np.array(target.standoff_pos, dtype=float)
        slide_velocity_world = self.ctx.slide_velocity_world()

        # 1) accumulate the initial contact force while holding p_contact
        rospy.loginfo("PUSH/APPLY: accumulating initial contact force ...")
        force_sum, force_num = 0.0, 0
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t_start < self.T_ACCUM_FORCE:
            if self.preempt_requested():
                self.service_preempt()
                self.ctx.retreat_start_p = p_contact
                self.ctx.finish_after_retreat = True
                return "aborted"
            if self.ctx.is_back_requested():
                self._prepare_operator_retreat(p_contact)
                return "retreat"

            io.publish_pose_ref(p_contact, q)
            f = io.contact_force_along(approach_dir)
            if f > self.FORCE_THRESH:
                force_sum += f
                force_num += 1
            self.rate.sleep()

        if rospy.is_shutdown():
            return "aborted"
        if self.ctx.is_back_requested():
            self._prepare_operator_retreat(p_contact)
            return "retreat"
        if force_num < self.ctx.min_force_samples:
            rospy.logerr(
                "PUSH/APPLY: only %d valid force samples (minimum %d); retreating.",
                force_num,
                self.ctx.min_force_samples,
            )
            self.ctx.retreat_start_p = p_contact
            self.ctx.finish_after_retreat = True
            return "aborted"

        stiffness_param = f"/{self.ctx.robot_name}/controller/nmpc/Qp_xy"
        try:
            k_p = float(rospy.get_param(stiffness_param))
        except (KeyError, TypeError, ValueError):
            rospy.logerr(
                "PUSH/APPLY: invalid or missing stiffness parameter %s.",
                stiffness_param,
            )
            self.ctx.retreat_start_p = p_contact
            self.ctx.finish_after_retreat = True
            return "aborted"

        if not np.isfinite(k_p) or k_p <= 0.0:
            rospy.logerr("PUSH/APPLY: stiffness must be finite and positive, got %.3f.", k_p)
            self.ctx.retreat_start_p = p_contact
            self.ctx.finish_after_retreat = True
            return "aborted"
        if not np.isfinite(target.desired_force) or target.desired_force < 0.0:
            rospy.logerr("PUSH/APPLY: desired force must be finite and non-negative.")
            self.ctx.retreat_start_p = p_contact
            self.ctx.finish_after_retreat = True
            return "aborted"

        init_force = force_sum / force_num
        requested_delta = (target.desired_force - init_force) / k_p
        delta = float(
            np.clip(
                requested_delta,
                -self.ctx.max_force_displacement,
                self.ctx.max_force_displacement,
            )
        )
        if not np.isclose(delta, requested_delta):
            rospy.logwarn(
                "PUSH/APPLY: requested displacement %.3f m exceeds the %.3f m safety limit; clamped to %.3f m.",
                requested_delta,
                self.ctx.max_force_displacement,
                delta,
            )

        p_final = p_contact + delta * approach_dir
        rospy.loginfo(
            "PUSH/APPLY: init_force=%.2f N, Kp=%.2f N/m, delta=%.3f m, desired_force=%.1f N.",
            init_force,
            k_p,
            delta,
            target.desired_force,
        )
        if np.linalg.norm(slide_velocity_world) > 0.0:
            rospy.loginfo(
                "PUSH/APPLY: sliding at world velocity %s m/s for %.1f s during force hold.",
                np.round(slide_velocity_world, 3).tolist(),
                self.T_FORCE_HOLD,
            )

        # 2) ramp up the normal force, 3) hold it while optionally sliding,
        # 4) ramp down at the final slide position.
        t_ramp_down_start = self.T_FORCE_RAMP + self.T_FORCE_HOLD
        t_total = t_ramp_down_start + self.T_FORCE_RAMP
        t_start = rospy.Time.now().to_sec()
        p = p_contact.copy()
        slide_offset = np.zeros(3)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                self.ctx.retreat_start_p = p
                self.ctx.finish_after_retreat = True
                return "aborted"
            if self.ctx.is_back_requested():
                self._prepare_operator_retreat(p)
                return "retreat"

            t = rospy.Time.now().to_sec() - t_start
            if t >= t_total:
                break
            p, slide_offset = _force_reference_at_time(
                t,
                p_contact,
                p_final,
                slide_velocity_world,
                self.T_FORCE_RAMP,
                self.T_FORCE_HOLD,
            )
            self.ctx.retreat_standoff_p = p_standoff + slide_offset
            io.publish_pose_ref(p, q)
            self.rate.sleep()

        if rospy.is_shutdown():
            return "aborted"

        p_contact_end, slide_offset = _force_reference_at_time(
            t_total,
            p_contact,
            p_final,
            slide_velocity_world,
            self.T_FORCE_RAMP,
            self.T_FORCE_HOLD,
        )
        self.ctx.retreat_standoff_p = p_standoff + slide_offset
        io.publish_pose_ref(p_contact_end, q)
        self.ctx.retreat_start_p = p_contact_end
        return "applied"


class RetreatState(PushBaseState):
    """Unload the force by ramping back to the standoff pose, then advance the index."""

    def __init__(self, ctx):
        super().__init__(ctx, outcomes=["retreated", "aborted"])

    def execute(self, userdata):
        # The initiating request has already been consumed. Any repeated key press
        # during RETREAT is deliberately ignored so the safe motion can complete.
        self.ctx.consume_back_request()
        io = self.ctx.io
        target = self.ctx.push_targets[self.ctx.target_idx]
        p_standoff = (
            np.array(self.ctx.retreat_standoff_p, dtype=float)
            if self.ctx.retreat_standoff_p is not None
            else np.array(target.standoff_pos, dtype=float)
        )
        p_return = np.array(self.ctx.return_standoff_p, dtype=float) if self.ctx.return_standoff_p is not None else None
        q = self.ctx.target_quaternion(target)
        if self.ctx.retreat_start_p is not None:
            p_start = np.array(self.ctx.retreat_start_p, dtype=float)
        elif self.ctx.contact_p is not None:
            p_start = np.array(self.ctx.contact_p, dtype=float)
        else:
            p_start = p_standoff

        rospy.loginfo(
            "PUSH/RETREAT: returning to standoff %s at %.2f m/s ...",
            np.round(p_standoff, 3).tolist(),
            self.RETREAT_SPEED,
        )
        move_result = self._move_at_speed(
            io,
            p_start,
            p_standoff,
            q,
            self.RETREAT_SPEED,
            self.RETREAT_TIMEOUT,
            q_from=io.get_orientation_xyzw(),
            ang_speed=self.ALIGN_ANG_SPEED,
            allow_back=False,
        )
        self.ctx.consume_back_request()
        if move_result.reason != "reached":
            rospy.logerr("PUSH/RETREAT: failed to reach standoff (%s).", move_result.reason)
            return "aborted"

        if p_return is not None and not np.allclose(p_standoff, p_return):
            rospy.loginfo(
                "PUSH/RETREAT: clear of wall; returning to initial standoff %s ...",
                np.round(p_return, 3).tolist(),
            )
            move_result = self._move_at_speed(
                io,
                p_standoff,
                p_return,
                q,
                self.RETREAT_SPEED,
                self.RETREAT_TIMEOUT,
                allow_back=False,
            )
            self.ctx.consume_back_request()
            if move_result.reason != "reached":
                rospy.logerr("PUSH/RETREAT: failed to return to initial standoff (%s).", move_result.reason)
                return "aborted"

        should_finish = self.ctx.finish_after_retreat
        self.ctx.contact_p = None
        self.ctx.retreat_start_p = None
        self.ctx.retreat_standoff_p = None
        self.ctx.return_standoff_p = None
        self.ctx.finish_after_retreat = False
        if should_finish:
            rospy.logwarn("PUSH/RETREAT: safe retreat completed; terminating the push task.")
            return "aborted"

        self.ctx.target_idx += 1
        return "retreated"


class FinishPushState(smach.State):
    """Restore terminal input before handing control back to the parent IDLE state."""

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=["done"])
        self.ctx = ctx

    def execute(self, userdata):
        self.ctx.stop_back_monitor()
        return "done"


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

    ctx = PushContext(
        push_tasks if push_tasks is not None else DEFAULT_PUSH_TASKS,
        fixed_push_targets=push_targets,
    )

    with sm_sub:
        smach.StateMachine.add(
            "INIT_PUSH",
            InitPushState(ctx),
            transitions={"start": "SELECT", "failed": "FINISH_PUSH"},
        )
        smach.StateMachine.add(
            "SELECT",
            SelectState(ctx),
            transitions={
                "next": "ALIGN",
                "finished": "FINISH_PUSH",
                "retreat": "RETREAT",
                "aborted": "FINISH_PUSH",
            },
        )
        smach.StateMachine.add(
            "ALIGN",
            AlignState(ctx),
            transitions={
                "aligned": "CALIBRATE",
                "retreat": "RETREAT",
                "aborted": "FINISH_PUSH",
            },
        )
        smach.StateMachine.add(
            "CALIBRATE",
            CalibrateState(ctx),
            transitions={
                "calibrated": "APPROACH",
                "retreat": "RETREAT",
                "aborted": "FINISH_PUSH",
            },
        )
        smach.StateMachine.add(
            "APPROACH",
            ApproachState(ctx),
            transitions={
                "contact": "APPLY",
                "retreat": "RETREAT",
                "aborted": "RETREAT",
            },
        )
        smach.StateMachine.add(
            "APPLY",
            ApplyForceState(ctx),
            transitions={
                "applied": "RETREAT",
                "retreat": "RETREAT",
                "aborted": "RETREAT",
            },
        )
        smach.StateMachine.add(
            "RETREAT",
            RetreatState(ctx),
            transitions={"retreated": "SELECT", "aborted": "FINISH_PUSH"},
        )
        smach.StateMachine.add(
            "FINISH_PUSH",
            FinishPushState(ctx),
            transitions={"done": "DONE_PUSH"},
        )

    return sm_sub
