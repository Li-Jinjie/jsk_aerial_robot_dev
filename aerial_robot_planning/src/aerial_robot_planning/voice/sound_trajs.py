"""
 Created by li-jinjie on 2025/12/7.
"""
import numpy as np
import rospy
from std_msgs.msg import Float32

from ..trajs import BaseTrajwFixedRotor


class BaseTrajwSound(BaseTrajwFixedRotor):
    def __init__(self, loop_num: int = np.inf):
        super().__init__(loop_num)

        # thrust of each musical note
        self.note2thrust = {
            "g4": 7.75,
            "g4sharp": 8.83,
            "a4": 9.96,
            "a4sharp": 11.20,
            "b4": 12.54,
            "c5": 13.97,
            "c5sharp": 15.56,
            "d5": 17.22,
            "d5sharp": 18.87,
            "e5": 21.05,
        }

        self.freq_pub = rospy.Publisher("sound/fixed_rotor_frequency", Float32, queue_size=10)

    def thrust_to_freq(self, f):
        a = 0.0000161
        b = 0.0327
        c = -7.54 - f
        disc = b**2 - 4 * a * c
        if disc < 0:
            rospy.logwarn(f"Invalid thrust value f={f}, cannot compute frequency")
            return 0.0
        return (-b + np.sqrt(disc)) / (2 * a)

    def get_fixed_rotor(self, t: float):
        rotor_id = 0

        ft_fixed = self.compute_thrust_at_time(t)

        freq = self.thrust_to_freq(ft_fixed)
        self.freq_pub.publish(freq)

        alpha_fixed = np.arccos(self.hover_thrust / ft_fixed)
        self.use_fix_rotor_flag = True

        return rotor_id, ft_fixed, alpha_fixed

    def compute_thrust_at_time(self, t: float) -> float:
        raise NotImplementedError


class HappyBirthdayFixedRotorTraj(BaseTrajwSound):
    def __init__(self, loop_num: int = 1):
        super().__init__(loop_num)

        self.sequence = [
            ("g4", 1.0),  # lyrics: Happy
            ("a4", 1.0),  # Birth-
            ("g4", 1.0),  # day
            ("c5", 1.0),  # to
            ("b4", 2.0),  # You
            ("g4", 1.0),  # Happy
            ("a4", 1.0),  # Birth-
            ("g4", 1.0),  # day
            ("d5", 1.0),  # to
            ("c5", 2.0),  # You
        ]

        self.beat_times = np.cumsum([0.0] + [dur for _, dur in self.sequence])
        self.T = self.beat_times[-1]
        self.period = self.T
        self.min_thrust = 0.5

    def compute_thrust_at_time(self, t: float) -> float:
        t_mod = t % self.period
        idx = np.searchsorted(self.beat_times, t_mod, side="right") - 1
        if idx >= len(self.sequence):
            return self.min_thrust

        note, _ = self.sequence[idx]
        return self.note2thrust[note]


class TestThrustFrequencyTraj(BaseTrajwSound):
    def __init__(self, loop_num: int = 1):
        super().__init__(loop_num)

        self.sequence = [
            ("g4", 3.0),
            ("a4", 3.0),
            ("b4", 3.0),
            ("c5", 3.0),
            ("d5", 3.0),
        ]

        self.beat_times = np.cumsum([0.0] + [dur for _, dur in self.sequence])
        self.T = self.beat_times[-1]
        self.period = self.T
        self.min_thrust = 0.5

    def compute_thrust_at_time(self, t: float) -> float:
        t_mod = t % self.period
        idx = np.searchsorted(self.beat_times, t_mod, side="right") - 1
        if idx >= len(self.sequence):
            return self.min_thrust

        note, _ = self.sequence[idx]
        return self.note2thrust[note]
