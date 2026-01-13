#!/usr/bin/env python3
import numpy as np
import sounddevice as sd
import soundfile as sf
import time
from datetime import datetime

RATE = 48000
BLOCK_DURATION = 0.1

DEVICE = "hw:1,0"  # USB microphone
CHANNELS = 1


class WavRecorder:
    def __init__(self):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = f"record_{timestamp}.wav"
        self.frames = []

        print(f"Recording started → {self.filename}")
        print("Press Ctrl+C to stop")

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            print(status)

        self.frames.append(indata.copy())

    def start(self):
        try:
            with sd.InputStream(
                device=DEVICE,
                channels=CHANNELS,
                samplerate=RATE,
                blocksize=int(RATE * BLOCK_DURATION),
                callback=self.audio_callback,
            ):
                while True:
                    time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        print("\nStopping recording...")
        audio = np.concatenate(self.frames, axis=0)
        sf.write(self.filename, audio, RATE)
        print(f"Saved wav file: {self.filename}")


if __name__ == "__main__":
    recorder = WavRecorder()
    recorder.start()
