"""
Created by li-jinjie on 2026/02/7.
Interactive Marker Server for RVIZ pose input.
"""

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from geometry_msgs.msg import Pose


class InteractiveMarkerPoseServer:
    """
    A wrapper around InteractiveMarkerServer that:
    - Creates a 6DOF interactive marker at a specified pose
    - Allows users to manipulate the pose in RVIZ
    - Provides a "Confirm Pose" menu button
    - Stores the latest pose and confirmation status
    """

    def __init__(self, robot_name: str, initial_pose: Pose, frame_id: str = "world"):
        """
        Initialize the interactive marker server.

        Args:
            robot_name: Name of the robot (e.g., "beetle1", "gimbalrotors")
            initial_pose: Initial pose for the marker
            frame_id: Reference frame for the marker (default: "world")
        """
        self.robot_name = robot_name
        self.frame_id = frame_id
        self.marker_name = f"{robot_name}_goal_pose"

        # Current pose of the marker
        self.current_pose = initial_pose

        # Confirmation flag
        self.pose_confirmed = False
        self.confirmed_pose = None

        # Create interactive marker server
        self.server = InteractiveMarkerServer(f"{robot_name}_interactive_marker")

        # Create menu handler for "Confirm Pose" button
        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Confirm Pose", callback=self._confirm_callback)

        # Create the interactive marker
        self._create_6dof_marker()

        # Apply changes
        self.server.applyChanges()

        rospy.loginfo(
            f"Interactive marker server initialized for {robot_name} at pose: "
            f"x={initial_pose.position.x:.2f}, y={initial_pose.position.y:.2f}, z={initial_pose.position.z:.2f}"
        )

    def _create_6dof_marker(self):
        """Create a 6DOF interactive marker with translation and rotation controls."""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = self.marker_name
        int_marker.description = f"{self.robot_name} Goal Pose\n(Use controls to set, then right-click to confirm)"
        int_marker.pose = self.current_pose
        int_marker.scale = 0.5

        # Create a box marker for visualization
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.2
        box_marker.scale.y = 0.2
        box_marker.scale.z = 0.1
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.8
        box_marker.color.a = 0.8

        # Create control for the box (non-interactive, just for visualization)
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # Add 6DOF controls (3 translation + 3 rotation axes)
        # Move X
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Move Y
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Move Z
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Rotate X
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Rotate Y
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Rotate Z
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Insert marker and set callback for pose updates
        self.server.insert(int_marker, self._process_feedback)

        # Apply menu to marker
        self.menu_handler.apply(self.server, self.marker_name)

    def _process_feedback(self, feedback: InteractiveMarkerFeedback):
        """
        Callback for interactive marker feedback.
        Updates the current pose when user manipulates the marker.
        """
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.current_pose = feedback.pose
            rospy.logdebug(
                f"Marker pose updated: x={feedback.pose.position.x:.2f}, "
                f"y={feedback.pose.position.y:.2f}, z={feedback.pose.position.z:.2f}"
            )

    def _confirm_callback(self, feedback: InteractiveMarkerFeedback):
        """
        Callback for the "Confirm Pose" menu item.
        Sets the confirmation flag and stores the confirmed pose.
        """
        self.pose_confirmed = True
        self.confirmed_pose = self.current_pose
        rospy.loginfo(
            f"Pose confirmed! Target: x={self.current_pose.position.x:.2f}, "
            f"y={self.current_pose.position.y:.2f}, z={self.current_pose.position.z:.2f}, "
            f"qw={self.current_pose.orientation.w:.2f}, qx={self.current_pose.orientation.x:.2f}, "
            f"qy={self.current_pose.orientation.y:.2f}, qz={self.current_pose.orientation.z:.2f}"
        )

    def is_pose_confirmed(self) -> bool:
        """Check if the user has confirmed the pose."""
        return self.pose_confirmed

    def get_confirmed_pose(self) -> Pose:
        """Get the confirmed pose. Returns None if not confirmed yet."""
        return self.confirmed_pose

    def reset_confirmation(self):
        """Reset the confirmation flag for a new pose input."""
        self.pose_confirmed = False
        self.confirmed_pose = None
        rospy.loginfo("Confirmation reset. Waiting for new pose input...")

    def update_marker_pose(self, new_pose: Pose):
        """
        Update the marker to a new pose (e.g., current drone position).

        Args:
            new_pose: New pose for the marker
        """
        self.current_pose = new_pose
        self.server.setPose(self.marker_name, new_pose)
        self.server.applyChanges()
        rospy.loginfo(
            f"Marker pose updated to: x={new_pose.position.x:.2f}, "
            f"y={new_pose.position.y:.2f}, z={new_pose.position.z:.2f}"
        )

    def shutdown(self):
        """Shutdown the interactive marker server."""
        self.server.clear()
        self.server.applyChanges()
        rospy.loginfo(f"Interactive marker server for {self.robot_name} shutdown.")
