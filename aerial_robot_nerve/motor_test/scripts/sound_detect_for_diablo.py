#!/usr/bin/env python3
import rospy
import numpy as np
import sounddevice as sd
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

RATE = 44100
BLOCK_DURATION = 0.1
DETECTION_TIME = 0.7
ENERGY_THRESHOLD = 1e-6

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
    "A": {
        "lx": 0.0,
        "ly": 0.0,
        "lz": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 0.0,
    },
    "B": {
        "lx": 0.0,
        "ly": 0.0,
        "lz": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 0.0,
    },
    "C": {
        "lx": 0.0,
        "ly": 0.0,
        "lz": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 0.0,
    },
    "D": {
        "lx": 0.0,
        "ly": 0.0,
        "lz": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 0.0,
    },
    "E": {
        "lx": 0.1,
        "ly": 0.0,
        "lz": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 0.0,
    },
    "F": {
        "lx": 0.0,
        "ly": 0.0,
        "lz": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 0.0,
    },
    "G": {
        "lx": 0.0,
        "ly": 0.0,
        "lz": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 0.0,
    },
}


class BandSoundDetector:
    def __init__(self):
        rospy.init_node("sound_note_detector")
        self.note_pub = rospy.Publisher("/detected_note", String, queue_size=10)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.last_note = None
        self.detect_start_time = None
        rospy.loginfo("Listening for notes (E–D)...")

    def start(self):
        with sd.InputStream(
            channels=3,
            samplerate=RATE,
            callback=self.audio_callback,
            blocksize=int(RATE * BLOCK_DURATION),
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
                if np.any(mask):
                    energy = np.mean(mag[mask])
                    energies.append(energy)
                else:
                    energies.append(0.0)

            if all(e > ENERGY_THRESHOLD for e in energies):
                avg_energy = np.mean(energies)
                if avg_energy > max_energy:
                    max_energy = avg_energy
                    detected_note = note

        current_time = time.time()

        if detected_note:
            if self.last_note == detected_note:
                if current_time - self.detect_start_time >= DETECTION_TIME:
                    rospy.loginfo(f"Detected note: {detected_note}")
                    self.note_pub.publish(detected_note)

                    # Publish Twist for each node
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

                    self.last_note = None
                    self.detect_start_time = None
            else:
                self.last_note = detected_note
                self.detect_start_time = current_time
        else:
            self.last_note = None
            self.detect_start_time = None
            rospy.loginfo_throttle(2, "searching for sound...")


if __name__ == "__main__":
    try:
        detector = BandSoundDetector()
        detector.start()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Stopped by user.")
