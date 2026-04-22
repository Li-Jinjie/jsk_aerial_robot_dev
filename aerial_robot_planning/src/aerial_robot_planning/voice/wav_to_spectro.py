#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt
import scienceplots  # noqa: F401  # registers scienceplot styles for plt.style.use
from scipy.io import wavfile
from scipy.signal import spectrogram

if len(sys.argv) != 2:
    print("Usage: python wav_to_spectro.py input.wav")
    sys.exit(1)

wav_filename = sys.argv[1]

fs, data = wavfile.read(wav_filename)

if data.ndim > 1:
    data = data.mean(axis=1)

frequencies, times, Sxx = spectrogram(data, fs=fs, nperseg=1024, noverlap=512)

time_start = 35.0
time_end = 51.0
time_mask = (times >= time_start) & (times <= time_end)
if not np.any(time_mask):
    raise ValueError(
        f"No spectrogram frames in requested range {time_start}-{time_end}s. "
        f"Audio duration is approximately {len(data) / fs:.2f}s."
    )
times = times[time_mask] - 36  # to enforce the same start with traj data
Sxx = Sxx[:, time_mask]

freq_limit = 2000
freq_mask = frequencies <= freq_limit
frequencies = frequencies[freq_mask]
Sxx = Sxx[freq_mask, :]

Sxx_dB = 10 * np.log10(Sxx + 1e-10)

plt.style.use(["science", "ieee", "no-latex"])
plt.rcParams.update(
    {
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7,
    }
)

# IEEE single-column width is about 3.5 in.
plt.figure(figsize=(3.5, 2.6), dpi=300)
plt.imshow(
    Sxx_dB,
    aspect="auto",
    origin="lower",
    cmap="gray",
    extent=[times.min(), times.max(), frequencies.min(), frequencies.max()],
)

plt.ylim([frequencies.min(), freq_limit])
plt.xlim([time_start - 36, time_end - 36])
plt.colorbar(label="Intensity [dB]")
plt.ylabel("Frequency [Hz]")
plt.xlabel("Time [sec]")
# plt.title("Spectrogram (35-50 sec, 0-2000 Hz)")

plt.tight_layout()
plt.show()
