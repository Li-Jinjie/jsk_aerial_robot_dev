#!/usr/bin/env python3
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
from scipy.signal import spectrogram

if len(sys.argv) != 3:
    print("Usage: python wav_transfer.py input.wav output.png")
    sys.exit(1)

wav_filename = sys.argv[1]
png_filename = sys.argv[2]

fs, data = wavfile.read(wav_filename)

if data.ndim > 1:
    data = data.mean(axis=1)

frequencies, times, Sxx = spectrogram(data, fs=fs, nperseg=1024, noverlap=512)

freq_limit = 2000
freq_mask = frequencies <= freq_limit
frequencies = frequencies[freq_mask]
Sxx = Sxx[freq_mask, :]

Sxx_dB = 10 * np.log10(Sxx + 1e-10)

plt.figure(figsize=(10, 6))
plt.imshow(
    Sxx_dB,
    aspect="auto",
    origin="lower",
    cmap="gray",
    extent=[times.min(), times.max(), frequencies.min(), frequencies.max()],
)

plt.ylim([frequencies.min(), freq_limit])
plt.colorbar(label="Intensity [dB]")
plt.ylabel("Frequency [Hz]")
plt.xlabel("Time [sec]")
plt.title("Spectrogram (0-2000 Hz)")

plt.tight_layout()
plt.savefig(png_filename)
plt.close()

print(f"Saved spectrogram to: {png_filename}")
