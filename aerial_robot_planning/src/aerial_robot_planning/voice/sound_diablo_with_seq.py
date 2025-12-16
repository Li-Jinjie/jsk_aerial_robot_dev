#!/usr/bin/env python3
import rospy
import numpy as np
import sounddevice as sd
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

RATE = 48000
BLOCK_DURATION = 0.1
DETECTION_TIME = 0.8
ENERGY_THRESHOLD = 1e-6

SEQ_NOTE_DURATION = 1.0  # duration of each sound of sequence
START_SEQ = ["G", "C", "A"]
STOP_SEQ = ["G", "B", "F"]

BANDS = {
    "E": [(311.13, 339.29), (622.25, 678.57)],
    "F": [(339.29, 369.99), (678.57, 739.99)],
    "G": [(369.99, 415.30), (739.99, 830.61)],
    "A": [(415.30, 466.16), (830.61, 932.33)],
    "B": [(466.16, 508.35), (932.33, 1016.71)],
    "C": [(508.35, 554.37), (1016.71, 1108.73)],
    "D": [(554.37, 622.25), (1108.73, 1244.51)],
}

NOTE_TO_TWIST = {
    "E": {"lx": 0.1, "ly": 0.0, "lz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0},
    "F": {"lx": 0.0, "ly": 0.0, "lz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0},
    "G": {"lx": 0.0, "ly": 0.0, "lz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0},
    "A": {"lx": 0.0, "ly": 0.0, "lz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0},
    "B": {"lx": 0.0, "ly": 0.0, "lz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0},
    "C": {"lx": 0.0, "ly": 0.0, "lz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0},
    "D": {"lx": 0.0, "ly": 0.0, "lz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0},
}


class BandSoundDetector:
    def __init__(self):
        rospy.init_node("sound_note_detector")

        self.note_pub = rospy.Publisher("/detected_note", String, queue_size=10)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.last_note = None
        self.detect_start_time = None

        # command state
        self.command_active = False

        # sequence tracking
        self.seq_buffer = []
        self.seq_note_start_time = None

        rospy.loginfo("Listening for notes (E–D)...")

    def check_sequence(self, note, target_seq):
        now = time.time()

        # new note
        if not self.seq_buffer or self.seq_buffer[-1] != note:
            self.seq_buffer.append(note)
            self.seq_note_start_time = now

            if len(self.seq_buffer) > len(target_seq):
                self.seq_buffer.pop(0)

        # sequence matched
        if self.seq_buffer == target_seq:
            if now - self.seq_note_start_time >= SEQ_NOTE_DURATION:
                self.seq_buffer = []
                self.seq_note_start_time = None
                return True

        return False

    def start(self):
        with sd.InputStream(
            device="hw:1,0",  # USB mic
            channels=1,
            samplerate=RATE,
            blocksize=int(RATE * BLOCK_DURATION),
            callback=self.audio_callback,
        ):
            rospy.spin()

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            rospy.logwarn(str(status))

        data = indata[:, 0]
        fft_data = np.fft.rfft(data)
        freq = np.fft.rfftfreq(len(data), 1.0 / RATE)
        mag = np.abs(fft_data)

        detected_note = None
        max_energy = 0.0

        for note, ranges in BANDS.items():
            energies = []
            for fmin, fmax in ranges:
                mask = (freq >= fmin) & (freq < fmax)
                energies.append(np.mean(mag[mask]) if np.any(mask) else 0.0)

            if all(e > ENERGY_THRESHOLD for e in energies):
                avg_energy = np.mean(energies)
                if avg_energy > max_energy:
                    max_energy = avg_energy
                    detected_note = note

        now = time.time()

        if detected_note:
            rospy.loginfo(f"Detected note: {detected_note}")
            self.note_pub.publish(detected_note)

            # ---- START sequence ----
            if not self.command_active:
                if self.check_sequence(detected_note, START_SEQ):
                    rospy.loginfo("🎉 Command STARTED (G C A)")
                    self.command_active = True
                else:
                    rospy.loginfo("command not started yet")
                return

            # ---- STOP sequence ----
            if self.command_active:
                if self.check_sequence(detected_note, STOP_SEQ):
                    rospy.loginfo("🛑 Command STOPPED (G B F)")
                    self.command_active = False
                    return

            # ---- NORMAL COMMAND ----
            if detected_note in NOTE_TO_TWIST:
                params = NOTE_TO_TWIST[detected_note]
                twist = Twist()
                twist.linear.x = params["lx"]
                twist.linear.y = params["ly"]
                twist.linear.z = params["lz"]
                twist.angular.x = params["ax"]
                twist.angular.y = params["ay"]
                twist.angular.z = params["az"]

                self.cmd_pub.publish(twist)

        else:
            rospy.loginfo_throttle(2, "searching for sound...")


if __name__ == "__main__":
    try:
        detector = BandSoundDetector()
        detector.start()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Stopped by user.")
