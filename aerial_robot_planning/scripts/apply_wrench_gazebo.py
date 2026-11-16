#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import threading

import rospy
from geometry_msgs.msg import Point, Wrench
from gazebo_msgs.srv import (
    ApplyBodyWrench,
    ApplyBodyWrenchRequest,
    BodyRequest,
    BodyRequestRequest,
)
from std_srvs.srv import Empty, EmptyResponse


def _clear_private_params_and_restore_cli():
    """
    Clear all private parameters under this node's namespace, then restore
    command-line private parameter overrides such as _force_x:=1.
    Must be called after rospy.init_node() and before rospy.get_param("~...").
    """
    # Parse command-line private params from sys.argv, e.g. _force_x:=1
    cli_private_params = rospy.client.load_command_line_node_params(sys.argv)

    node_name = rospy.get_name()  # e.g. /constant_force_applier

    # Clear the whole private namespace of this node
    if rospy.has_param(node_name):
        rospy.delete_param(node_name)
        rospy.loginfo("Cleared private parameter namespace: %s", node_name)

    # Restore CLI private params into this node's private namespace
    for key, value in cli_private_params.items():
        full_key = node_name + "/" + key
        rospy.set_param(full_key, value)
        rospy.loginfo("Restored CLI private param: %s = %s", full_key, value)


class ConstantForceApplier:
    """
    Continuously apply a wrench through Gazebo.

    The target can be changed at runtime through the private ``~wrench`` topic:

      rostopic pub --once /constant_force_applier/wrench geometry_msgs/Wrench \
        '{force: {x: 5.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'

    Call ``rosservice call /constant_force_applier/clear`` to stop applying the
    wrench immediately.
    """

    def __init__(self):
        rospy.init_node("constant_force_applier")

        # Clear stale private params from previous runs, but preserve current
        # command-line overrides such as _force_x:=1.
        _clear_private_params_and_restore_cli()

        # ===== Parameters =====
        # Note: body_name must be a link name, not a model name
        # The format is usually "model_name::link_name"
        self.robot_name = rospy.get_param("~robot_name", "ball2")
        self.body_name = rospy.get_param("~body_name", self.robot_name + "::root")
        self.rate_hz = rospy.get_param("~rate", 50.0)

        # Force application mode:
        # - "body_offset": apply the wrench at a point offset from body_name
        #   and express both force and point in the body frame.
        # - otherwise, use the provided reference_frame directly.
        self.point_mode = rospy.get_param("~point_mode", "body_offset")
        self.reference_frame = rospy.get_param("~reference_frame", "world")

        # Offset of the application point from root (or body_name),
        # expressed in the body frame.
        self.offset_x = rospy.get_param("~offset_x", 0.0)
        self.offset_y = rospy.get_param("~offset_y", 0.0)
        self.offset_z = rospy.get_param("~offset_z", 0.0)  # 0.35

        # Fallback application point, expressed in the reference_frame
        self.point_x = rospy.get_param("~point_x", 0.0)
        self.point_y = rospy.get_param("~point_y", 0.0)
        self.point_z = rospy.get_param("~point_z", 0.0)

        # Target constant force (N)
        self.force_x = rospy.get_param("~force_x", 0.0)
        self.force_y = rospy.get_param("~force_y", 0.0)
        self.force_z = rospy.get_param("~force_z", 0.0)

        # Target constant torque (N*m)
        self.torque_x = rospy.get_param("~torque_x", 0.0)
        self.torque_y = rospy.get_param("~torque_y", 0.0)
        self.torque_z = rospy.get_param("~torque_z", 0.0)

        # Duration of each wrench application
        # It is recommended to set this slightly larger than 1 / rate_hz
        # to avoid gaps between consecutive applications
        self.duration = rospy.get_param("~duration", 0.05)

        # Ramp duration in seconds
        # The applied wrench increases gradually from zero to the target value
        self.ramp_duration = rospy.get_param("~ramp_duration", 2.0)

        self.command_lock = threading.RLock()
        self.target_force = [self.force_x, self.force_y, self.force_z]
        self.target_torque = [self.torque_x, self.torque_y, self.torque_z]
        self.ramp_start_force = [0.0, 0.0, 0.0]
        self.ramp_start_torque = [0.0, 0.0, 0.0]
        self.ramp_start_time = rospy.Time.now()

        rospy.loginfo("Waiting for /gazebo/apply_body_wrench service...")
        rospy.wait_for_service("/gazebo/apply_body_wrench")
        self.apply_wrench_srv = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)

        # Optional: clear any remaining wrench when shutting down
        self.clear_available = False
        try:
            rospy.wait_for_service("/gazebo/clear_body_wrenches", timeout=1.0)
            self.clear_wrenches_srv = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)
            self.clear_available = True
        except rospy.ROSException:
            rospy.logwarn("/gazebo/clear_body_wrenches not available.")

        self.wrench_sub = rospy.Subscriber("~wrench", Wrench, self.wrench_callback, queue_size=1)
        self.clear_srv = rospy.Service("~clear", Empty, self.clear_callback)

        rospy.on_shutdown(self.on_shutdown)

    def get_ramp_scale(self, now=None):
        """
        Compute a scale factor in [0, 1] for gradual ramp-up.
        The scale increases linearly with time until it reaches 1.
        """
        if self.ramp_duration <= 0.0:
            return 1.0

        if now is None:
            now = rospy.Time.now()
        elapsed = (now - self.ramp_start_time).to_sec()
        scale = max(0.0, min(1.0, elapsed / self.ramp_duration))
        return scale

    def _get_applied_wrench_components(self, now=None):
        if now is None:
            now = rospy.Time.now()

        with self.command_lock:
            scale = self.get_ramp_scale(now)
            force = [
                start + scale * (target - start) for start, target in zip(self.ramp_start_force, self.target_force)
            ]
            torque = [
                start + scale * (target - start) for start, target in zip(self.ramp_start_torque, self.target_torque)
            ]
        return force, torque, scale

    def set_target_wrench(self, force, torque):
        """Set a new target wrench while preserving a continuous ramp."""
        if len(force) != 3 or len(torque) != 3:
            raise ValueError("force and torque must each contain exactly three values")

        now = rospy.Time.now()
        current_force, current_torque, _ = self._get_applied_wrench_components(now)
        with self.command_lock:
            self.ramp_start_force = current_force
            self.ramp_start_torque = current_torque
            self.target_force = [float(value) for value in force]
            self.target_torque = [float(value) for value in torque]
            self.ramp_start_time = now

        rospy.loginfo(
            "New target wrench | Force: [%.3f, %.3f, %.3f] | Torque: [%.3f, %.3f, %.3f]",
            *self.target_force,
            *self.target_torque,
        )

    def wrench_callback(self, msg):
        self.set_target_wrench(
            [msg.force.x, msg.force.y, msg.force.z],
            [msg.torque.x, msg.torque.y, msg.torque.z],
        )

    def clear_callback(self, _request):
        with self.command_lock:
            self.target_force = [0.0, 0.0, 0.0]
            self.target_torque = [0.0, 0.0, 0.0]
            self.ramp_start_force = [0.0, 0.0, 0.0]
            self.ramp_start_torque = [0.0, 0.0, 0.0]
            self.ramp_start_time = rospy.Time.now()

        self.clear_gazebo_wrenches()
        rospy.loginfo("The target wrench was cleared.")
        return EmptyResponse()

    def get_reference_frame_and_point(self):
        if self.point_mode == "body_offset":
            return self.body_name, Point(self.offset_x, self.offset_y, self.offset_z)

        return self.reference_frame, Point(self.point_x, self.point_y, self.point_z)

    def build_request(self):
        force, torque, _ = self._get_applied_wrench_components()
        reference_frame, reference_point = self.get_reference_frame_and_point()

        req = ApplyBodyWrenchRequest()
        req.body_name = self.body_name
        req.reference_frame = reference_frame
        req.reference_point = reference_point

        req.wrench = Wrench()
        req.wrench.force.x, req.wrench.force.y, req.wrench.force.z = force
        req.wrench.torque.x, req.wrench.torque.y, req.wrench.torque.z = torque

        req.start_time = rospy.Time(0)  # Apply immediately
        req.duration = rospy.Duration(self.duration)
        return req

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Applying ramped wrench to [%s]", self.body_name)

        while not rospy.is_shutdown():
            req = self.build_request()
            _, _, scale = self._get_applied_wrench_components()
            try:
                self.apply_wrench_srv(req)
                rospy.loginfo_throttle(
                    0.5,
                    "Ramp scale: %.3f | Ref frame: %s | Point: [%.3f, %.3f, %.3f] | "
                    "Force: [%.3f, %.3f, %.3f] | Torque: [%.3f, %.3f, %.3f]",
                    scale,
                    req.reference_frame,
                    req.reference_point.x,
                    req.reference_point.y,
                    req.reference_point.z,
                    req.wrench.force.x,
                    req.wrench.force.y,
                    req.wrench.force.z,
                    req.wrench.torque.x,
                    req.wrench.torque.y,
                    req.wrench.torque.z,
                )
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call /gazebo/apply_body_wrench: %s", str(e))
            rate.sleep()

    def clear_gazebo_wrenches(self):
        if not self.clear_available:
            return

        try:
            req = BodyRequestRequest()
            req.body_name = self.body_name
            self.clear_wrenches_srv(req)
            rospy.loginfo("Cleared body wrenches in Gazebo.")
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to clear body wrenches: %s", str(e))

    def on_shutdown(self):
        rospy.loginfo("Shutting down constant_force_applier...")
        self.clear_gazebo_wrenches()


if __name__ == "__main__":
    node = ConstantForceApplier()
    node.run()
