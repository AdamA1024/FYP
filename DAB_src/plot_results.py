"""
plot_results.py  –  Plot DAB RTL simulation output

Reads results.csv produced by the Verilator testbench and generates a figure
with separate i_L and V2 panels for three time windows:
  Row 0: full simulation
  Row 1: bridge drive signals p1, p2
  Row 2: start-up transient  (first 10 switching cycles)
  Row 3: steady state        (last 10 switching cycles)

Run:  python3 plot_results.py          # saves dab_results.png
      python3 plot_results.py --show   # also opens the window
"""

import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Parameters ────────────────────────────────────────────────────────────────
V1  = 400.0        # V    primary bus voltage
N   = 2.0          #      transformer turns ratio
L   = 1e-3         # H    leakage inductance
Co  = 100e-6       # F    output filter capacitance
R   = 10.0         # Ω    load resistance
fsw = 10e3         # Hz   switching frequency
phi = math.pi / 4  # rad  phase shift (p2 lags p1)

DT          = 100e-9   # s    simulation timestep
HALF_PERIOD = 500      # clock cycles per half-period
FULL_PERIOD = 1000
PHASE_SHIFT = 125      # p2 lag in clock cycles

CSV_FILE = "results.csv"

# ── Theoretical steady state (single-phase-shift formula) ─────────────────────
#   Power balance:  N·V1·V2·φ·(π−φ) / (2π²·fsw·L) = V2²/R
#   → V2_ss = N·V1·R·φ·(π−φ) / (2π²·fsw·L)
V2_theory = N * V1 * R * phi * (math.pi - abs(phi)) / (2 * math.pi**2 * fsw * L)

#   Peak i_L (zero DC, V1 > N·V2 so current peaks at T/2):
#     Both v_L intervals are positive → current rises throughout first half-period.
#     Anti-symmetry: i_L(T/2) = −i_L(0)  →  2·IL_peak = area(v_L) / L
#     area = (V1+N·V2)·φ_T + (V1−N·V2)·(T/2−φ_T) = V1·T/2 + N·V2·(2·φ_T − T/2)
_half_T = 1 / (2 * fsw)
_phi_T  = phi / (2 * math.pi * fsw)
IL_peak = (V1 * _half_T + N * V2_theory * (2 * _phi_T - _half_T)) / (2 * L)

BLUE   = "#1f77b4"
ORANGE = "#ff7f0e"
GREEN  = "#2ca02c"
RED    = "#d62728"
GREY   = "#888888"

# ── Load data ──────────────────────────────────────────────────────────────────
data  = np.loadtxt(CSV_FILE, delimiter=",", skiprows=1)
cycle = data[:, 0].astype(int)
p1    = data[:, 1].astype(int)
p2    = data[:, 2].astype(int)
i_L   = data[:, 3]               # [A]
V2    = data[:, 4]               # [V]
t_us  = cycle * DT * 1e6         # time [µs]

p1_signed = np.where(p1, 1, -1).astype(float)
p2_signed = np.where(p2, 1, -1).astype(float)

# ── Steady-state metrics ───────────────────────────────────────────────────────
N_SW_ZOOM = 10
t_ss_lo   = t_us[-1] - N_SW_ZOOM * FULL_PERIOD * DT * 1e6
mask_e    = t_us >= t_ss_lo
v2_mean   = V2[mask_e].mean()
v2_err    = (v2_mean - V2_theory) / V2_theory * 100

# ── Masks for zoom windows ─────────────────────────────────────────────────────
mask_s = t_us <= N_SW_ZOOM * FULL_PERIOD * DT * 1e6   # start-up

# ── Figure layout: 4 rows × 2 cols ────────────────────────────────────────────
#   Row 0: full simulation   [i_L | V2]
#   Row 1: drive signals     [spans both cols]
#   Row 2: start-up          [i_L | V2]
#   Row 3: steady state      [i_L | V2]
fig = plt.figure(figsize=(14, 14))
fig.suptitle(
    "DAB DC–DC Converter  –  RTL Simulation\n"
    rf"$V_1={V1:.0f}\,$V, $N={N:.0f}$, $L={L*1e3:.0f}\,$mH,"
    rf" $C_o={Co*1e6:.0f}\,\mu$F, $R={R:.0f}\,\Omega$,"
    rf" $\phi={math.degrees(phi):.0f}^\circ$, $f_{{sw}}={fsw/1e3:.0f}\,$kHz",
    fontsize=12)

gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.55, wspace=0.35,
                       height_ratios=[1.0, 0.65, 1.0, 1.0])


def label_ax(ax, ylabel, color=None, xlabel=None):
    if xlabel:
        ax.set_xlabel(xlabel)
    if color:
        ax.set_ylabel(ylabel, color=color)
        ax.tick_params(axis="y", colors=color)
    else:
        ax.set_ylabel(ylabel)
    ax.grid(axis="y", ls=":", alpha=0.4)


# ── Row 0: full simulation ─────────────────────────────────────────────────────
ax_iL_full = fig.add_subplot(gs[0, 0])
ax_V2_full = fig.add_subplot(gs[0, 1])

ax_iL_full.plot(t_us, i_L, color=BLUE, lw=0.35, alpha=0.8)
label_ax(ax_iL_full, r"$i_L$ [A]", BLUE)
ax_iL_full.set_title(f"Full sim ({int(t_us[-1]/1e3)} ms)  –  inductor current")

ax_V2_full.plot(t_us, V2, color=ORANGE, lw=0.6)
label_ax(ax_V2_full, r"$V_2$ [V]", ORANGE)
ax_V2_full.set_title(f"Full sim ({int(t_us[-1]/1e3)} ms)  –  output voltage")

# Share x-axis between the two full-sim panels
ax_iL_full.sharex(ax_V2_full)

# ── Row 1: bridge drive signals (full width) ───────────────────────────────────
ax_drv = fig.add_subplot(gs[1, :])

N_SW_WAVES = 4
mask_w = t_us <= N_SW_WAVES * FULL_PERIOD * DT * 1e6
ax_drv.step(t_us[mask_w], p1_signed[mask_w], where="post",
            color=GREEN, lw=1.5, label=r"$p_1$")
ax_drv.step(t_us[mask_w], p2_signed[mask_w] * 0.85, where="post",
            color=RED, lw=1.5, ls="--", label=r"$p_2$ (×0.85)")
t_p2_rise = PHASE_SHIFT * DT * 1e6
ax_drv.annotate("", xy=(t_p2_rise, 0.5), xytext=(0.0, 0.5),
                arrowprops=dict(arrowstyle="<->", color=GREY, lw=1.2))
ax_drv.text(t_p2_rise / 2, 0.62, rf"$\phi={math.degrees(phi):.0f}^\circ$",
            ha="center", va="bottom", fontsize=9, color=GREY)
ax_drv.set_xlabel("Time [µs]")
ax_drv.set_ylabel("Phase  (±1)")
ax_drv.set_yticks([-1, 0, 1])
ax_drv.set_ylim(-1.4, 1.4)
ax_drv.set_title(f"Bridge drive signals (first {N_SW_WAVES} cycles)")
ax_drv.legend(loc="upper right", fontsize=9)
ax_drv.grid(axis="x", ls=":", alpha=0.5)

# ── Row 2: start-up transient ─────────────────────────────────────────────────
ax_iL_su = fig.add_subplot(gs[2, 0])
ax_V2_su = fig.add_subplot(gs[2, 1])

ax_iL_su.plot(t_us[mask_s], i_L[mask_s], color=BLUE, lw=0.9)
label_ax(ax_iL_su, r"$i_L$ [A]", BLUE, xlabel="Time [µs]")
ax_iL_su.set_title(f"Start-up (first {N_SW_ZOOM} cycles)  –  inductor current")
for k in range(N_SW_ZOOM):
    t0 = k * FULL_PERIOD * DT * 1e6
    ax_iL_su.axvspan(t0, t0 + HALF_PERIOD * DT * 1e6, alpha=0.06, color=GREEN, zorder=0)
    ax_iL_su.axvspan(t0 + HALF_PERIOD * DT * 1e6, t0 + FULL_PERIOD * DT * 1e6,
                     alpha=0.06, color=RED, zorder=0)

ax_V2_su.plot(t_us[mask_s], V2[mask_s], color=ORANGE, lw=0.9)
label_ax(ax_V2_su, r"$V_2$ [V]", ORANGE, xlabel="Time [µs]")
ax_V2_su.set_title(f"Start-up (first {N_SW_ZOOM} cycles)  –  output voltage")

ax_iL_su.sharex(ax_V2_su)

# ── Row 3: steady state ────────────────────────────────────────────────────────
ax_iL_ss = fig.add_subplot(gs[3, 0])
ax_V2_ss = fig.add_subplot(gs[3, 1])

ax_iL_ss.plot(t_us[mask_e], i_L[mask_e], color=BLUE, lw=0.9)
label_ax(ax_iL_ss, r"$i_L$ [A]", BLUE, xlabel="Time [µs]")
ax_iL_ss.set_title(f"Steady state (last {N_SW_ZOOM} cycles)  –  inductor current")
ax_iL_ss.axhline( IL_peak, color=BLUE, lw=0.9, ls=":", alpha=0.7)
ax_iL_ss.axhline(-IL_peak, color=BLUE, lw=0.9, ls=":", alpha=0.7)
ax_iL_ss.annotate(rf"$\pm I_{{pk}}^{{th}}={IL_peak:.2f}$ A",
                  xy=(t_us[mask_e][5], IL_peak),
                  xytext=(0, 5), textcoords="offset points",
                  color=BLUE, fontsize=8, alpha=0.85)

ax_V2_ss.plot(t_us[mask_e], V2[mask_e], color=ORANGE, lw=0.9)
label_ax(ax_V2_ss, r"$V_2$ [V]", ORANGE, xlabel="Time [µs]")
ax_V2_ss.set_title(f"Steady state (last {N_SW_ZOOM} cycles)  –  output voltage")
ax_V2_ss.axhline(v2_mean, color=ORANGE, lw=1.0, ls="--", alpha=0.9)
ax_V2_ss.annotate(rf"$\bar{{V}}_2={v2_mean:.1f}$ V (sim)",
                  xy=(t_us[mask_e][5], v2_mean),
                  xytext=(0, 6), textcoords="offset points",
                  color=ORANGE, fontsize=8)
ax_V2_ss.axhline(V2_theory, color=GREY, lw=1.0, ls=":")
ax_V2_ss.annotate(rf"$V_2^{{th}}={V2_theory:.1f}$ V  ({v2_err:+.1f}%)",
                  xy=(t_us[mask_e][5], V2_theory),
                  xytext=(0, -14), textcoords="offset points",
                  color=GREY, fontsize=8)

ax_iL_ss.sharex(ax_V2_ss)

# ── Save ───────────────────────────────────────────────────────────────────────
out = "dab_results.png"
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"Saved {out}")
print(f"  V2_sim={v2_mean:.3f} V,  V2_theory={V2_theory:.3f} V,  error={v2_err:+.2f}%")

if "--show" in sys.argv:
    plt.show()
