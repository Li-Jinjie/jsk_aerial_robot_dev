#!/usr/bin/env python3
import rospy
import numpy as np
import sounddevice as sd
import time
from std_msgs.msg import String
from datetime import datetime
import soundfile as sf

RATE = 48000  # [Hz]
BLOCK_DURATION = 0.1  # [s]
DETECTION_TIME = 1.2  # [s]
ENERGY_THRESHOLD = 1e-6  # 0~1

BANDS = {
    "E": [(311.13, 339.29), (622.25, 678.57)],
    "F": [(339.29, 369.99), (678.57, 739.99)],
    "G": [(369.99, 415.30), (739.99, 830.61)],
    "A": [(415.30, 466.16), (830.61, 932.33)],
    "B": [(466.16, 508.35), (932.33, 1016.71)],
    "C": [(508.35, 554.37), (1016.71, 1108.73)],
    "D": [(554.37, 622.25), (1108.73, 1244.51)],
}


class BandSoundDetector:
    def __init__(self):
        rospy.init_node("sound_note_detector")
        self.note_pub = rospy.Publisher("/detected_note", String, queue_size=10)
        self.audio_buffer = []
        self.filename = None
        self.last_note = None
        self.detect_start_time = None
        rospy.loginfo("Listening for notes (E–D) & recording audio...")

    def start(self):
        try:
            with sd.InputStream(
                device="hw:1,0",
                # USB mic: "hw: 1,0"
                # PC mic: "hw: 0,6"
                channels=1,
                samplerate=RATE,
                callback=self.audio_callback,
                blocksize=int(RATE * BLOCK_DURATION),
            ):
                rospy.spin()

        except KeyboardInterrupt:
            rospy.loginfo("Stopped by user.")
        finally:
            self.save_audio()

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            rospy.logwarn(str(status))

        data = indata[:, 0].copy()
        self.audio_buffer.append(data)

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
                else:
                    energy = 0.0
                energies.append(energy)

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
                    self.last_note = None
                    self.detect_start_time = None
            else:
                self.last_note = detected_note
                self.detect_start_time = current_time
        else:
            self.last_note = None
            self.detect_start_time = None
            rospy.loginfo_throttle(2, "searching for sound...")

    def save_audio(self):
        """バッファの音声を WAV で保存"""
        if len(self.audio_buffer) == 0:
            rospy.logwarn("No audio data to save.")
            return

        audio_np = np.concatenate(self.audio_buffer)

        if self.filename is None:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.filename = f"record_{timestamp}.wav"

        sf.write(self.filename, audio_np, RATE)
        rospy.loginfo(f"Saved audio: {self.filename}")

        self.audio_buffer = []


if __name__ == "__main__":
    detector = BandSoundDetector()
    detector.start()
