"""
Created by li-jinjie on 2026/02/7.
SMACH state machine for interactive marker mode.
"""

import rospy
import smach
from nav_msgs.msg import Odometry

from ..pub_mpc_joint_traj import MPCSinglePtPub
from ..util import topic_ready
from .interactive_marker_server import InteractiveMarkerPoseServer

# Global variable to store marker server (cannot be serialized in userdata)
shared_data = {"marker_server": None}


class InitInteractiveMarkerState(smach.State):
    """
    Initialize the interactive marker at the current drone position.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_wait_pose", "done_interactive_marker"],
            input_keys=["robot_name"],
            output_keys=["robot_name"],
        )

    def execute(self, userdata):
        rospy.loginfo("INIT_INTERACTIVE_MARKER: Initializing interactive marker...")

        try:
            # Subscribe to odometry to get current drone position
            odom_topic = f"/{userdata.robot_name}/uav/cog/odom"

            if not topic_ready(odom_topic, Odometry, timeout=2.0):
                rospy.logerr(f"INIT_INTERACTIVE_MARKER: Odometry topic {odom_topic} not available!")
                return "done_interactive_marker"

            # Get current drone pose
            odom_msg = rospy.wait_for_message(odom_topic, Odometry, timeout=5.0)
            current_pose = odom_msg.pose.pose

            rospy.loginfo(
                f"INIT_INTERACTIVE_MARKER: Current drone position: "
                f"x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}, z={current_pose.position.z:.2f}"
            )

            # Create interactive marker server at current position
            marker_server = InteractiveMarkerPoseServer(
                robot_name=userdata.robot_name, initial_pose=current_pose, frame_id="world"
            )

            # Store marker server in shared_data (not userdata, as it's not serializable)
            shared_data["marker_server"] = marker_server

            rospy.loginfo("INIT_INTERACTIVE_MARKER: Interactive marker ready in RVIZ!")
            rospy.loginfo(
                "INIT_INTERACTIVE_MARKER: Drag the marker to set target pose, then right-click and select 'Confirm Pose'."
            )

            return "go_wait_pose"

        except rospy.ROSException as e:
            rospy.logerr(f"INIT_INTERACTIVE_MARKER: Failed to get odometry: {e}")
            return "done_interactive_marker"
        except Exception as e:
            rospy.logerr(f"INIT_INTERACTIVE_MARKER: Unexpected error: {e}")
            return "done_interactive_marker"


class WaitForPoseConfirmState(smach.State):
    """
    Wait for the user to confirm the pose via the interactive marker menu.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_navigate", "done_interactive_marker"],
            input_keys=["robot_name"],
            output_keys=["robot_name", "target_pose"],
        )
        self.rate = rospy.Rate(10)  # 10 Hz check rate

    def execute(self, userdata):
        rospy.loginfo("WAIT_POSE: Waiting for pose confirmation in RVIZ...")
        rospy.loginfo(
            "WAIT_POSE: Right-click on the marker and select 'Confirm Pose' when ready, or press 'q' + Enter to quit."
        )

        marker_server = shared_data["marker_server"]

        # Start a thread to check for keyboard input
        import threading

        quit_flag = [False]

        def check_quit():
            try:
                user_input = input().strip().lower()
                if user_input == "q":
                    quit_flag[0] = True
            except:
                pass

        input_thread = threading.Thread(target=check_quit, daemon=True)
        input_thread.start()

        # Wait for confirmation
        while not rospy.is_shutdown():
            if quit_flag[0]:
                rospy.loginfo("WAIT_POSE: User requested quit.")
                marker_server.shutdown()
                return "done_interactive_marker"

            if marker_server.is_pose_confirmed():
                target_pose = marker_server.get_confirmed_pose()
                userdata.target_pose = target_pose

                rospy.loginfo(
                    f"WAIT_POSE: Pose confirmed! Target: "
                    f"x={target_pose.position.x:.2f}, y={target_pose.position.y:.2f}, z={target_pose.position.z:.2f}"
                )

                return "go_navigate"

            self.rate.sleep()

        # ROS shutdown
        marker_server.shutdown()
        return "done_interactive_marker"


class NavigateToPoseState(smach.State):
    """
    Navigate the drone to the confirmed target pose using MPCSinglePtPub.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_wait_pose", "done_interactive_marker"],
            input_keys=["robot_name", "target_pose"],
            output_keys=["robot_name"],
        )
        self.rate = rospy.Rate(20)  # 20 Hz loop check

    def execute(self, userdata):
        rospy.loginfo("NAVIGATE: Flying to target pose...")

        try:
            target_pose = userdata.target_pose
            marker_server = shared_data["marker_server"]

            # Create MPCSinglePtPub node to fly to target
            mpc_node = MPCSinglePtPub(
                robot_name=userdata.robot_name,
                frame_id="world",
                child_frame_id="cog",
                target_pose=target_pose,
                pos_tol=0.2,
                ang_tol=0.3,
                vel_tol=0.1,
                rate_tol=0.1,
            )

            # Wait until navigation is complete
            while not rospy.is_shutdown():
                if mpc_node.is_finished:
                    rospy.loginfo("NAVIGATE: Target pose reached!")
                    break
                self.rate.sleep()

            rospy.sleep(0.5)  # Brief pause

            # Ask if user wants to continue or quit
            user_choice = input("\nPress 'c' to set another pose, or 'q' to quit: ").strip().lower()

            if user_choice == "q":
                rospy.loginfo("NAVIGATE: User requested quit.")
                marker_server.shutdown()
                return "done_interactive_marker"
            else:
                # Reset confirmation and go back to waiting for new pose
                marker_server.reset_confirmation()

                # Update marker to current position
                odom_topic = f"/{userdata.robot_name}/uav/cog/odom"
                odom_msg = rospy.wait_for_message(odom_topic, Odometry, timeout=5.0)
                current_pose = odom_msg.pose.pose
                marker_server.update_marker_pose(current_pose)

                return "go_wait_pose"

        except Exception as e:
            rospy.logerr(f"NAVIGATE: Error during navigation: {e}")
            shared_data["marker_server"].shutdown()
            return "done_interactive_marker"


def create_interactive_marker_state_machine():
    """Create and return the interactive marker state machine."""
    sm_sub = smach.StateMachine(outcomes=["DONE_INTERACTIVE_MARKER"], input_keys=["robot_name"])

    with sm_sub:
        # Initialize interactive marker
        smach.StateMachine.add(
            "INIT_INTERACTIVE_MARKER",
            InitInteractiveMarkerState(),
            transitions={"go_wait_pose": "WAIT_POSE", "done_interactive_marker": "DONE_INTERACTIVE_MARKER"},
        )

        # Wait for pose confirmation
        smach.StateMachine.add(
            "WAIT_POSE",
            WaitForPoseConfirmState(),
            transitions={"go_navigate": "NAVIGATE", "done_interactive_marker": "DONE_INTERACTIVE_MARKER"},
        )

        # Navigate to confirmed pose
        smach.StateMachine.add(
            "NAVIGATE",
            NavigateToPoseState(),
            transitions={"go_wait_pose": "WAIT_POSE", "done_interactive_marker": "DONE_INTERACTIVE_MARKER"},
        )

    return sm_sub
