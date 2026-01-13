#!/usr/bin/env python3
import rospy
import numpy as np
import sounddevice as sd
import time

RATE = 44100  # [Hz]
BLOCK_DURATION = 0.1  # [s]
DETECTION_TIME = 2.0  # [s]
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
        self.last_note = None
        self.detect_start_time = None
        rospy.loginfo("Listening for notes (E–D)...")

    def start(self):
        with sd.InputStream(
            channels=1,
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

        # calculate the amplitude for each range
        for note, ranges in BANDS.items():
            total_energy = 0.0
            count = 0
            for fmin, fmax in ranges:
                mask = (freq >= fmin) & (freq < fmax)
                if np.any(mask):
                    total_energy += np.mean(mag[mask])
                    count += 1
            if count > 0:
                energy = total_energy / count
                if energy > ENERGY_THRESHOLD and energy > max_energy:
                    max_energy = energy
                    detected_note = note

        current_time = time.time()

        if detected_note:
            if self.last_note == detected_note:
                if current_time - self.detect_start_time >= DETECTION_TIME:
                    rospy.loginfo(f"Detected note: {detected_note}")
                    # reset after detecting
                    self.last_note = None
                    self.detect_start_time = None
            else:
                self.last_note = detected_note
                self.detect_start_time = current_time
        else:
            self.last_note = None
            self.detect_start_time = None
            rospy.loginfo_throttle(2, "searching for sound...")  # show every 2 seconds


if __name__ == "__main__":
    try:
        detector = BandSoundDetector()
        detector.start()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Stopped by user.")
