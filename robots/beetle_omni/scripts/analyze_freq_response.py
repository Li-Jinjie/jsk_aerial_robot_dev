import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import scienceplots

# =========================
# Plot style for IEEE paper
# =========================
plt.style.use(["science"])
plt.rcParams.update({"font.size": 14})
plt.rcParams.update(
    {
        "grid.alpha": 0.3,
        "grid.linewidth": 0.5,
    }
)
label_size = 15

# =========================
# Parameters
# =========================
f_pre = 10.27  # pre LPF cutoff frequency [Hz]
f_post = 2.0  # post LPF cutoff frequency [Hz]

ts = 0.01  # sampling time [s]
m = 3.039  # mass [kg]
KI = 100  # integral gain

# Convert to rad/s
w_pre = 2 * np.pi * f_pre
w_post = 2 * np.pi * f_post

# Equivalent integral bandwidth
w_I = KI * ts**2 / (2 * m)

print(f"K_I     = {KI:.1f}")
print(f"omega_I = {w_I:.6e} rad/s")
print(f"f_I     = {w_I / (2*np.pi):.6e} Hz")

# =========================
# Transfer functions
# =========================

# H_e(s) = H_post(s) H_pre(s)
#        = w_post/(s+w_post) * w_pre/(s+w_pre)
num_He = [w_pre * w_post]
den_He = [1, w_pre + w_post, w_pre * w_post]
H_e = signal.TransferFunction(num_He, den_He)

# H_fdm_fother(s) = w_I / (s + w_I)
H_fdm_fother = signal.TransferFunction([w_I], [1, w_I])

# H_fdm_fde(s) = w_I/(s+w_I) * (1 - H_e(s))
#
# 1 - H_e(s)
# = [s^2 + (w_pre+w_post)s] /
#   [s^2 + (w_pre+w_post)s + w_pre*w_post]
num_one_minus_He = [1, w_pre + w_post, 0]
den_one_minus_He = den_He

num_Hfdm_fde = np.polymul([w_I], num_one_minus_He)
den_Hfdm_fde = np.polymul([1, w_I], den_one_minus_He)
H_fdm_fde = signal.TransferFunction(num_Hfdm_fde, den_Hfdm_fde)

# =========================
# Frequency response
# =========================
f = np.logspace(-5, 2, 2000)  # Hz
w = 2 * np.pi * f  # rad/s

_, mag_He, phase_He = signal.bode(H_e, w=w)
_, mag_fdm_fde, phase_fdm_fde = signal.bode(H_fdm_fde, w=w)
_, mag_fdm_fother, phase_fdm_fother = signal.bode(H_fdm_fother, w=w)

# =========================
# Combined magnitude and phase plot
# =========================
fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True)

ax_mag, ax_phase = axes

# Magnitude
ax_mag.semilogx(f, mag_He, label=r"$\hat{f}_{de}/f_{de}$")
ax_mag.semilogx(f, mag_fdm_fde, label=r"$f_{dm}/f_{de}$")
ax_mag.semilogx(f, mag_fdm_fother, label=r"$f_{dm}/f_{\mathrm{other}}$")

ax_mag.set_ylabel(r"Magnitude [dB]")
ax_mag.legend(
    # loc='lower left',
    frameon=True,
    handlelength=1.6,
    borderpad=0.3,
    labelspacing=0.25,
    framealpha=0.85,
)

# Phase
ax_phase.semilogx(f, phase_He)
ax_phase.semilogx(f, phase_fdm_fde)
ax_phase.semilogx(f, phase_fdm_fother)

ax_phase.set_xlabel(r"Frequency [Hz]", fontsize=label_size)
ax_phase.set_ylabel(r"Phase [deg]", fontsize=label_size)

# Axis and layout
for ax in axes:
    ax.grid(True, which="both")
    ax.set_xlim([1e-5, 1e2])

fig.tight_layout(pad=0.4)
plt.show()
