"""
 Created by li-jinjie on 2025/12/7.
"""

import rospy
import smach
import re
from geometry_msgs.msg import Pose, Quaternion, Vector3

from ..pub_mpc_joint_traj import MPCSinglePtPub, MPCTrajPtPub
from .sound_trajs import BaseTrajwSound, StringNoteTraj
from .tokenizer import allowed_token_list


class InitVoiceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["go_listen_input"], input_keys=["robot_name"], output_keys=["robot_name"])
        self.rate = rospy.Rate(20)  # 20 Hz loop checking if finished

    def execute(self, userdata):
        rospy.loginfo("INIT: fly to the initial position and get ready for voice commands.")

        # get initial pose from trajectory
        traj = BaseTrajwSound()
        frame_id = traj.get_frame_id()
        child_frame_id = traj.get_child_frame_id()
        x, y, z, vx, vy, vz, ax, ay, az = traj.get_3d_pt(0.0)
        qw, qx, qy, qz, r_rate, p_rate, y_rate, r_acc, p_acc, y_acc = traj.get_3d_orientation(0.0)

        # fly to it
        init_pose = Pose(
            position=Vector3(x, y, z),
            orientation=Quaternion(qx, qy, qz, qw),
        )

        # Create the node instance
        mpc_node = MPCSinglePtPub(userdata.robot_name, frame_id, child_frame_id, init_pose)

        # Wait here until the node signals it is finished or ROS shuts down
        while not rospy.is_shutdown():
            if mpc_node.is_finished:
                rospy.loginfo("INIT: MPCSinglePtPub says the init pose is reached.")
                break
            self.rate.sleep()

        rospy.loginfo("VOICE Initialization done.")

        return "go_listen_input"


class ListenInputState(smach.State):
    # wait for user input and execute the listen() method 10 seconds a time
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_play", "stay_listen_input", "done_voice"],
            input_keys=["robot_name"],
            output_keys=["robot_name", "user_input", "duration_per_note"],
        )

    def execute(self, userdata):
        rospy.loginfo("LISTEN: Waiting for voice command input...")

        try:
            pass  # TODO: xxx.start()  # Start voice recognition system here
        except Exception as e:
            rospy.logerr(f"LISTEN: Failed to start voice recognition system: {e}")
            return "done_voice"

        user_input = (
            input(f"\nPress the combination of {allowed_token_list} to play notes, or 'q' to quit: ").strip().lower()
        )

        if user_input == "q":
            rospy.loginfo("LISTEN: Quitting voice command mode.")
            pass  # TODO: xxx.stop()  # Stop voice recognition system here
            return "done_voice"

        # check if the input is the combination of allowed strings
        pattern = r"^(?:" + "|".join(allowed_token_list) + ")+$"
        if re.match(pattern, user_input):
            duration_per_note = input(f"\nEnter duration per note in seconds (e.g. 2): ").strip()

            userdata.user_input = user_input
            try:
                userdata.duration_per_note = float(duration_per_note)
            except ValueError:
                rospy.logwarn(f"LISTEN: Invalid duration '{duration_per_note}'. Using default of 2 seconds.")
                userdata.duration_per_note = 2.0

            pass  # TODO: xxx.stop()  # Stop voice recognition system here

            rospy.loginfo(
                f"LISTEN: Received command to play note '{user_input}' for {float(duration_per_note)} seconds."
            )
            return "go_play"

        rospy.logwarn("LISTEN: Unexpected reading input. Please try again.")
        return "stay_listen_input"


class PlayState(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_listen_input", "done_voice"],
            input_keys=["robot_name", "user_input", "duration_per_note"],
            output_keys=["robot_name"],
        )
        self.rate = rospy.Rate(20)  # 20 Hz loop check if finished

    def execute(self, userdata):
        rospy.loginfo(f"PLAY: Playing note '{userdata.user_input}'...")

        try:
            # Create trajectory for the specified note
            traj = StringNoteTraj(note=userdata.user_input, duration=userdata.duration_per_note)
            mpc_node = MPCTrajPtPub(robot_name=userdata.robot_name, traj=traj)

            # Wait here until the node signals it is finished or ROS shuts down
            while not rospy.is_shutdown():
                if mpc_node.is_finished:
                    rospy.loginfo("TRACK: MPCPtPubNode says the trajectory is finished.")
                    break
                self.rate.sleep()

            rospy.sleep(0.5)  # wait for the tracking error to be calculated and printed

            rospy.loginfo(f"PLAY: Finished playing note '{userdata.user_input}'.")
            return "go_listen_input"

        except Exception as e:
            rospy.logerr(f"PLAY: Failed to play note '{userdata.user_input}': {e}")
            return "done_voice"


def create_voice_state_machine():
    """VoiceStateMachine"""
    sm_sub = smach.StateMachine(outcomes=["DONE_VOICE"], input_keys=["robot_name"])

    with sm_sub:
        # InitObjectState
        smach.StateMachine.add(
            "INIT_VOICE",
            InitVoiceState(),
            transitions={"go_listen_input": "LISTEN_INPUT"},
        )

        smach.StateMachine.add(
            "LISTEN_INPUT",
            ListenInputState(),
            transitions={"go_play": "PLAY", "stay_listen_input": "LISTEN_INPUT", "done_voice": "DONE_VOICE"},
        )

        smach.StateMachine.add(
            "PLAY",
            PlayState(),
            transitions={"go_listen_input": "LISTEN_INPUT", "done_voice": "DONE_VOICE"},
        )

    return sm_sub
