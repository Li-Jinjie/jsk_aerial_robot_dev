#!/usr/bin/env python3
import rospy
import numpy as np
import sounddevice as sd
import time

RATE = 44100  # [Hz]
BLOCK_DURATION = 0.1  # [s]
DETECTION_TIME = 2.0  # [s]
ENERGY_THRESHOLD = 1e-6  # 0~1

# === 300〜800Hz: band by 50Hz ===
BANDS = [(f, f + 50) for f in range(300, 801, 50)]
# → [(300,350), (350,400), (400,450), (450,500), (500,550), (550,600), (600,650), (650,700), (700,750), (750,800)]


class BandSoundDetector:
    def __init__(self):
        rospy.init_node("sound_band_detector")
        self.last_band = None
        self.detect_start_time = None
        rospy.loginfo("Listening for sound between 300–800 Hz (50 Hz bands)...")

    def start(self):
        with sd.InputStream(
            channels=1, samplerate=RATE, callback=self.audio_callback, blocksize=int(RATE * BLOCK_DURATION)
        ):
            rospy.spin()

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            rospy.logwarn(str(status))

        # FFT
        data = indata[:, 0]
        fft_data = np.fft.rfft(data)
        freq = np.fft.rfftfreq(len(data), 1.0 / RATE)
        mag = np.abs(fft_data)

        detected_band = None
        max_energy = 0.0

        # energy for each range
        for fmin, fmax in BANDS:
            mask = (freq >= fmin) & (freq < fmax)
            if np.any(mask):
                energy = np.mean(mag[mask])
                if energy > ENERGY_THRESHOLD and energy > max_energy:
                    max_energy = energy
                    detected_band = (fmin, fmax)

        current_time = time.time()

        if detected_band:
            # whether the band is continuing
            if self.last_band == detected_band:
                if current_time - self.detect_start_time >= DETECTION_TIME:
                    rospy.loginfo(f"now: {detected_band[0]}-{detected_band[1]}")
                    # reset after detection
                    self.last_band = None
                    self.detect_start_time = None
            else:
                # print time of new detection
                self.last_band = detected_band
                self.detect_start_time = current_time
        else:
            # when no band is detected
            self.last_band = None
            self.detect_start_time = None
            rospy.loginfo_throttle(2, "searching for sound...")  # print every 2 seconds


if __name__ == "__main__":
    try:
        detector = BandSoundDetector()
        detector.start()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Stopped by user.")
