"""
 Created by Hirata and li-jinjie.
 NOTE: these trajectories should remain at the same pose while producing sound to execute multiple notes at one time.
"""

import numpy as np
import rospy
from std_msgs.msg import Float32

from ..trajs import BaseTrajwFixedRotor
from .tokenizer import allowed_token_list, tokenize


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

    def set_sequence(self, sequence):
        """
        sequence: list of (note, duration)
        """
        self.sequence = sequence
        self.beat_times = np.cumsum([0.0] + [dur for _, dur in sequence])
        self.period = self.beat_times[-1]
        self.T = self.period

    def compute_thrust_at_time(self, t: float) -> float:
        if self.sequence is None:
            rospy.logwarn("Sequence not set, using hover thrust")
            return self.hover_thrust

        t_mod = t % self.period
        idx = np.searchsorted(self.beat_times, t_mod, side="right") - 1

        if idx < 0 or idx >= len(self.sequence):
            return self.hover_thrust

        note, _ = self.sequence[idx]
        return self.note2thrust[note]


class BaseTrajForSoundTest(BaseTrajwFixedRotor):
    def __init__(self, loop_num: int = np.inf):
        super().__init__(loop_num)

        # thrust of each musical note
        self.note2thrust = {
            "sound1": 7.60,
            "sound2": 8.12,
            "sound3": 8.63,
            "sound4": 9.12,
            "sound5": 9.61,
            "sound6": 10.10,
            "sound7": 10.57,
            "sound8": 11.03,
            "sound9": 11.49,
            "sound10": 11.95,
            "sound11": 12.40,
            "sound12": 12.84,
            "sound13": 13.28,
            "sound14": 13.71,
            "sound15": 14.13,
            "sound16": 14.56,
            "sound17": 14.98,
            "sound18": 15.39,
            "sound19": 15.80,
            "sound20": 16.21,
            "sound21": 16.61,
            "sound22": 17.01,
            "sound23": 17.41,
            "sound24": 17.80,
            "sound25": 18.19,
            "sound26": 18.58,
            "sound27": 18.96,
            "sound28": 19.34,
            "sound29": 19.72,
            "sound30": 20.09,
            "sound31": 20.84,
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

    def set_sequence(self, sequence):
        """
        sequence: list of (note, duration)
        """
        self.sequence = sequence
        self.beat_times = np.cumsum([0.0] + [dur for _, dur in sequence])
        self.period = self.beat_times[-1]
        self.T = self.period

    def compute_thrust_at_time(self, t: float) -> float:
        if self.sequence is None:
            rospy.logwarn("Sequence not set, using hover thrust")
            return self.hover_thrust

        t_mod = t % self.period
        idx = np.searchsorted(self.beat_times, t_mod, side="right") - 1

        if idx < 0 or idx >= len(self.sequence):
            return self.hover_thrust

        note, _ = self.sequence[idx]
        return self.note2thrust[note]


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

        self.set_sequence(self.sequence)


class SoundResolutionFixedRotorTraj(BaseTrajForSoundTest):
    def __init__(self, loop_num: int = 1):
        super().__init__(loop_num)

        self.sequence = [
            ("sound1", 3.0),
            ("sound2", 3.0),
            ("sound3", 3.0),
            ("sound4", 3.0),
            ("sound5", 3.0),
            ("sound6", 3.0),
            ("sound7", 3.0),
            ("sound8", 3.0),
            ("sound9", 3.0),
            ("sound10", 3.0),
            ("sound11", 3.0),
            ("sound12", 3.0),
            ("sound13", 3.0),
            ("sound14", 3.0),
            ("sound15", 3.0),
            ("sound16", 3.0),
            ("sound17", 3.0),
            ("sound18", 3.0),
            ("sound19", 3.0),
            ("sound20", 3.0),
            ("sound21", 3.0),
            ("sound22", 3.0),
            ("sound23", 3.0),
            ("sound24", 3.0),
            ("sound25", 3.0),
            ("sound26", 3.0),
            ("sound27", 3.0),
            ("sound28", 3.0),
            ("sound29", 3.0),
            ("sound30", 3.0),
            ("sound31", 3.0),
        ]

        self.set_sequence(self.sequence)


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

        self.set_sequence(self.sequence)


class StringNoteTraj(BaseTrajwSound):
    def __init__(self, note: str = "a4", duration: float = 2.0, loop_num: int = 1):
        super().__init__(loop_num)

        note_list = tokenize(note, allowed_token_list)
        rospy.loginfo(f"Constructed note sequence: {note_list}")

        self.sequence = [(n, duration) for n in note_list]

        self.set_sequence(self.sequence)


class TulipFixedRotorTraj(BaseTrajwSound):
    def __init__(self, loop_num: int = 1):
        super().__init__(loop_num)

        self.sequence = [
            ("g4", 1.0),  # lyrics: Sa-
            ("a4", 1.0),  # i-
            ("b4", 2.0),  # ta
            ("g4", 1.0),  # Sa-
            ("a4", 1.0),  # i-
            ("b4", 2.0),  # ta
            ("d5", 1.0),  # Tu-
            ("b4", 1.0),  # li-
            ("a4", 1.0),  # p
            ("g4", 1.0),  # no
            ("a4", 1.0),  # Ha-
            ("b4", 1.0),  # na
            ("a4", 2.0),  # ga
            ("g4", 1.0),  # Na-
            ("a4", 1.0),  # ran-
            ("b4", 2.0),  # da
            ("g4", 1.0),  # Na-
            ("a4", 1.0),  # ran-
            ("b4", 2.0),  # da
            ("d5", 1.0),  # A-
            ("b4", 1.0),  # ka-
            ("a4", 1.0),  # Shi-
            ("g4", 1.0),  # ro
            ("a4", 1.0),  # Ki-
            ("b4", 1.0),  # i-
            ("a4", 2.0),  # ro. changed from g4 due to risk
            ("d5", 2.0),  # Dono
            ("b4", 1.0),  # ha-
            ("d5", 1.0),  # na
            ("e5", 2.0),  # Mite-
            ("d5", 2.0),  # mo-
            ("b4", 2.0),  # Kire-
            ("a4", 2.0),  # ida-
            ("g4", 3.0),  # na
        ]


class Canon1FixedRotorTraj(BaseTrajwSound):
    def __init__(self, loop_num: int = 1):
        super().__init__(loop_num)

        self.sequence = [
            ("a4sharp", 5.0),  # preparation for music start
            ("d5", 5.0),  # preparation for music start
            ("e5", 2.0),
            ("d5", 2.0),
            ("c5", 2.0),
            ("b4", 2.0),
            ("a4", 2.0),
            ("g4", 2.0),
            ("a4", 2.0),
            ("b4", 2.0),
        ]


class Canon2FixedRotorTraj(BaseTrajwSound):
    def __init__(self, loop_num: int = 1):
        super().__init__(loop_num)

        self.sequence = [
            ("a4sharp", 5.0),  # preparation for music start
            ("c5", 2.0),
            ("b4", 2.0),
            ("a4", 2.0),
            ("g4", 2.0),
            ("a4", 2.0),
            ("g4", 2.0),
            ("a4", 2.0),
            ("g4", 2.0),
        ]


class Canon3FixedRotorTraj(BaseTrajwSound):
    def __init__(self, loop_num: int = 1):
        super().__init__(loop_num)

        self.sequence = [
            ("c5", 0.5),
            ("g4", 1.5),
            ("b4", 0.5),  # changed from g4 due to risk
            ("d5", 1.5),
            ("a4", 0.5),
            ("c5", 0.5),  # changed from e5 due to risk
            ("e5", 1.0),
            ("e5", 0.5),
            ("b4", 1.5),
            ("a4", 0.5),
            ("c5", 1.5),
            ("a4", 0.5),
            ("g4", 1.5),
            ("a4", 0.5),
            ("c5", 1.5),
            ("c5", 0.5),  # changed from g4 due to risk
            ("d5", 1.5),
        ]


class Canon4FixedRotorTraj(BaseTrajwSound):
    def __init__(self, loop_num: int = 1):
        super().__init__(loop_num)

        self.sequence = [
            ("g4", 2.0),
            ("g4", 0.5),
            ("a4", 0.5),
            ("d5", 0.5),  # changed from g4 due to risk
            ("d5", 0.5),
            ("e5", 1.0),
            ("c5", 0.5),
            ("e5", 1.0),
            ("d5", 0.5),
            ("e5", 0.5),
            ("d5", 0.5),
            ("c5", 1.0),
            ("a4", 0.5),
            ("c5", 1.5),
            ("e5", 0.5),
            ("c5", 0.5),
            ("c5", 0.5),
            ("a4sharp", 0.5),
            ("a4", 0.5),
            ("a4sharp", 0.5),
            ("b4", 2.0),
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
