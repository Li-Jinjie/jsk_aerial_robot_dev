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

RPM_MAX = 11000.0  # Maximum RPM for normalization


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
    Generate an audio buffer with harmonic components and loudness scaling.
    """
    with data_lock:
        freqs = [rpm / 60.0 for rpm in current_rpms]
        rpms = current_rpms.copy()

    t = np.arange(frames) / SAMPLE_RATE
    wave = np.zeros_like(t)

    # Harmonic weights
    A1 = 1.0  # fundamental
    A2 = 0.3  # 2nd harmonic
    A3 = 0.15  # 3rd harmonic

    for f, rpm in zip(freqs, rpms):
        if f > 0:
            # Calculate volume based on RPM
            volume = 0.05 + 0.25 * min(rpm / RPM_MAX, 1.0)

            # Add fundamental and harmonics
            wave += volume * (
                A1 * np.sin(2 * np.pi * f * t) + A2 * np.sin(2 * np.pi * 2 * f * t) + A3 * np.sin(2 * np.pi * 3 * f * t)
            )

    # Normalize to avoid clipping
    if len(freqs) > 0:
        wave /= len(freqs)

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
