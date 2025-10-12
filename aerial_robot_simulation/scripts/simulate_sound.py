#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sounddevice as sd
from threading import Lock
from spinal.msg import ESCTelemetryArray

# ==========================================================
# Global variables
# ==========================================================
current_rpms = [0.0, 0.0, 0.0, 0.0]  # store the latest rpm values for 4 rotors
data_lock = Lock()  # thread safety for rpm updates

# Sampling frequency for audio
SAMPLE_RATE = 44100
# Default buffer duration (not strictly used anymore)
BUFFER_DURATION = 0.05

RPM_MAX = 20000.0  # Maximum RPM for normalization


# ==========================================================
# Callback function
# ==========================================================
def esc_callback(msg):
    """
    Callback function for ESC telemetry topic.
    It updates the current RPM of each rotor.
    """
    global current_rpms
    with data_lock:
        current_rpms[0] = msg.esc_telemetry_1.rpm
        current_rpms[1] = msg.esc_telemetry_2.rpm
        current_rpms[2] = msg.esc_telemetry_3.rpm
        current_rpms[3] = msg.esc_telemetry_4.rpm


# ==========================================================
# Sound synthesis function
# ==========================================================
def generate_audio_buffer(frames):
    """
    Generate an audio buffer that matches the required number of frames.
    Each rotor contributes a sine wave at frequency rpm/60 Hz.
    """
    with data_lock:
        freqs = [rpm / 60.0 for rpm in current_rpms]

    t = np.arange(frames) / SAMPLE_RATE
    wave = np.zeros_like(t)

    for f in freqs:
        if f > 0:
            wave += np.sin(2 * np.pi * f * t)

    if len(freqs) > 0:
        wave /= len(freqs)

    volume = 0.2
    wave *= volume

    return wave.astype(np.float32)


# ==========================================================
# Audio callback for sounddevice
# ==========================================================
def audio_callback(outdata, frames, time, status):
    """
    This callback is repeatedly called by the audio stream.
    It must fill exactly `frames` samples into outdata.
    """
    if status:
        rospy.logwarn(f"Audio status: {status}")
    wave = generate_audio_buffer(frames)
    outdata[:] = wave.reshape(-1, 1)


# ==========================================================
# Main node
# ==========================================================
def main():
    rospy.init_node("rotor_sound_node")
    rospy.loginfo("Rotor sound node started.")

    rospy.Subscriber("/beetle1/esc_telem", ESCTelemetryArray, esc_callback)

    with sd.OutputStream(samplerate=SAMPLE_RATE, channels=1, callback=audio_callback, dtype="float32"):
        rospy.loginfo("Audio stream opened. Generating sound...")
        rospy.spin()

    rospy.loginfo("Shutting down rotor sound node.")


# ==========================================================
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
