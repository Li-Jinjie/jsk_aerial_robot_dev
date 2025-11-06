#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
sound_detector.py

"""

import numpy as np
import sounddevice as sd
import rospy
from std_msgs.msg import String


# settings
RANGE1 = (280, 320)  # Hz
RANGE2 = (560, 640)  # Hz
THRESHOLD = 0.03  # （0〜1）
DETECTION_TIME = 1.5
SAMPLE_RATE = 44100  #  [Hz]
FRAME_DURATION = 0.1  #  [s]

frame_size = int(SAMPLE_RATE * FRAME_DURATION)
detected_start = None


rospy.init_node("sound_detector")
pub = rospy.Publisher("/sound_found", String, queue_size=1)


def detect_sound(indata):
    """
    音声入力データから周波数成分を解析し、
    対象周波数帯が一定時間続いたらトピックをPublish。
    """
    global detected_start

    # FFT
    fft_data = np.fft.rfft(indata[:, 0])
    freqs = np.fft.rfftfreq(len(indata[:, 0]), 1 / SAMPLE_RATE)
    mag = np.abs(fft_data)
    mag = mag / np.max(mag + 1e-10)  # 0〜1

    # average energy for each frequency
    mask1 = (freqs >= RANGE1[0]) & (freqs <= RANGE1[1])
    mask2 = (freqs >= RANGE2[0]) & (freqs <= RANGE2[1])
    energy1 = np.mean(mag[mask1]) if np.any(mask1) else 0
    energy2 = np.mean(mag[mask2]) if np.any(mask2) else 0

    detected = (energy1 > THRESHOLD) or (energy2 > THRESHOLD)
    now = rospy.get_time()

    # detection logic
    if detected:
        if detected_start is None:
            detected_start = now
        elif now - detected_start >= DETECTION_TIME:
            rospy.loginfo("Sound found!")
            pub.publish("sound found")
            detected_start = None  # reset after detection
    else:
        detected_start = None


def audio_callback(indata, frames, time, status):
    if status:
        rospy.logwarn(f"Audio status: {status}")
    detect_sound(indata)


if __name__ == "__main__":
    print("Listening for sound between 420–460 Hz or 820–1920 Hz ...")
    print("If detected for 3 seconds → publish '/sound_found' = 'sound found'")

    try:
        with sd.InputStream(channels=1, samplerate=SAMPLE_RATE, callback=audio_callback, blocksize=frame_size):
            rospy.spin()
    except KeyboardInterrupt:
        print("\n Stopped by user")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
