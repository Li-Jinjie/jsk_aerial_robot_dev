#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt


def compute_alpha_original(fh, fv, f_thresh):
    """
    Original heuristic:

        alpha = atan2(fh, fv),                         if f_ir >= f_thresh
              = pi/2 - arccos(fh / f_thresh),          if f_ir <  f_thresh

    where f_ir = sqrt(fh^2 + fv^2).
    """
    f_norm = np.sqrt(fh**2 + fv**2)

    # Avoid invalid arccos due to numerical round-off.
    ratio = np.clip(fh / f_thresh, -1.0, 1.0)

    alpha_large = np.arctan2(fh, fv)
    alpha_small = np.pi / 2.0 - np.arccos(ratio)

    alpha = np.where(f_norm >= f_thresh, alpha_large, alpha_small)
    return alpha, f_norm


def save_fig(fig, filename_prefix, save):
    if save:
        fig.savefig(f"{filename_prefix}.png", dpi=300)
        fig.savefig(f"{filename_prefix}.pdf")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--f_thresh", type=float, default=1.0, help="Threshold force f_{t,thresh}.")
    parser.add_argument("--f_max", type=float, default=2.0, help="Plot range: f_h, f_v in [-f_max, f_max].")
    parser.add_argument("--num", type=int, default=401, help="Grid resolution.")
    parser.add_argument("--save_prefix", type=str, default="alpha_heuristic", help="Prefix of saved figures.")
    parser.add_argument("--save", action="store_true", help="Save figures as png/pdf instead of directly showing them.")
    args = parser.parse_args()

    f_thresh = args.f_thresh
    f_max = args.f_max
    num = args.num

    fh_vec = np.linspace(-f_max, f_max, num)
    fv_vec = np.linspace(-f_max, f_max, num)
    FH, FV = np.meshgrid(fh_vec, fv_vec)

    alpha, f_norm = compute_alpha_original(FH, FV, f_thresh)
    alpha_deg = np.rad2deg(alpha)

    theta = np.linspace(0.0, 2.0 * np.pi, 500)
    circle_h = f_thresh * np.cos(theta)
    circle_v = f_thresh * np.sin(theta)

    # ============================================================
    # 1. 3D surface plot
    # ============================================================
    fig = plt.figure(figsize=(7.0, 5.4))
    ax = fig.add_subplot(111, projection="3d")

    surf = ax.plot_surface(FH, FV, alpha_deg, rstride=4, cstride=4, linewidth=0, antialiased=True, alpha=0.95)

    z_min = np.nanmin(alpha_deg)
    ax.plot(circle_h, circle_v, z_min * np.ones_like(circle_h), linestyle="--", linewidth=1.5)

    ax.set_xlabel(r"$f_{ir,h}$")
    ax.set_ylabel(r"$f_{ir,v}$")
    ax.set_zlabel(r"$\alpha_{ir}$ [deg]")
    ax.set_title("Original heuristic surface")

    cbar = fig.colorbar(surf, ax=ax, shrink=0.65, pad=0.10)
    cbar.set_label(r"$\alpha_{ir}$ [deg]")

    ax.view_init(elev=30, azim=-135)
    fig.tight_layout()

    save_fig(fig, f"{args.save_prefix}_surface", args.save)

    # ============================================================
    # 2. 2D heatmap
    # ============================================================
    fig, ax = plt.subplots(figsize=(5.8, 4.8))

    im = ax.pcolormesh(FH, FV, alpha_deg, shading="auto")
    ax.plot(circle_h, circle_v, linestyle="--", linewidth=1.5, label=r"$f_{ir}=f_{t,\mathrm{thresh}}$")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(r"$f_{ir,h}$")
    ax.set_ylabel(r"$f_{ir,v}$")
    ax.set_title("Original heuristic angle map")
    ax.legend(loc="upper right")

    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label(r"$\alpha_{ir}$ [deg]")

    fig.tight_layout()

    save_fig(fig, f"{args.save_prefix}_heatmap", args.save)

    # ============================================================
    # 3. Line cut
    # ============================================================
    fh_cut = 0.5 * f_thresh
    fv_cut = np.linspace(-f_max, f_max, num)
    alpha_cut, f_norm_cut = compute_alpha_original(fh_cut * np.ones_like(fv_cut), fv_cut, f_thresh)

    fig, ax = plt.subplots(figsize=(5.8, 3.6))
    ax.plot(fv_cut, np.rad2deg(alpha_cut), linewidth=2.0)

    if abs(fh_cut) < f_thresh:
        fv_switch = np.sqrt(f_thresh**2 - fh_cut**2)
        ax.axvline(+fv_switch, linestyle="--", linewidth=1.2)
        ax.axvline(-fv_switch, linestyle="--", linewidth=1.2)

    ax.set_xlabel(r"$f_{ir,v}$")
    ax.set_ylabel(r"$\alpha_{ir}$ [deg]")
    ax.set_title(rf"Line cut at $f_{{ir,h}}={fh_cut:.2f}$")
    ax.grid(True)

    fig.tight_layout()

    save_fig(fig, f"{args.save_prefix}_linecut", args.save)

    if args.save:
        plt.close("all")
    else:
        plt.show()


if __name__ == "__main__":
    main()
