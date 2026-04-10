"""
plot_results.py  -  Plot the DAB single-cycle Verilator results.

Reads results.csv (cycle, i_L_A, V2_V) produced by tb_dab_top.cpp and writes
dab_top_results.png with three panels:

  Row 0: full transient            (V2 + i_L over the full sim)
  Row 1: start-up zoom             (first ~10 switching cycles)
  Row 2: steady-state ripple       (last ~10 switching cycles)

Run:  python3 plot_results.py            -> saves dab_top_results.png
      python3 plot_results.py --show     -> also opens a window
"""

import sys
import math
import numpy as np
import matplotlib.pyplot as plt

# Timing / operating point (must match tb_dab_top.cpp + dab_rtl.sv defaults)
DT          = 20e-9          # 50 MHz core
HALF_PERIOD = 50             # cycles  -> fsw = 500 kHz
FULL_PERIOD = 100
PHASE       = 13             # cycles  (13/50 * pi ~ 46.8 deg)
V1, N, L, R = 400.0, 2.0, 20e-6, 10.0
fsw         = 1.0 / (FULL_PERIOD * DT)
phi         = PHASE / HALF_PERIOD * math.pi

# Theoretical single-phase-shift V2:  V2 = N*V1*R*phi*(pi - phi) / (2*pi^2*fsw*L)
V2_theory = N * V1 * R * phi * (math.pi - phi) / (2 * math.pi**2 * fsw * L)

# Load CSV
data  = np.loadtxt("results.csv", delimiter=",", skiprows=1)
cycle = data[:, 0].astype(int)
iL    = data[:, 1]
V2    = data[:, 2]
t_us  = cycle * DT * 1e6

# Window masks
N_ZOOM = 10
t_su_hi = N_ZOOM * FULL_PERIOD * DT * 1e6
t_ss_lo = t_us[-1] - N_ZOOM * FULL_PERIOD * DT * 1e6
mask_su = t_us <= t_su_hi
mask_ss = t_us >= t_ss_lo

v2_mean    = V2[mask_ss].mean()
v2_ripple  = V2[mask_ss].max() - V2[mask_ss].min()
iL_peak    = iL[mask_ss].max()
iL_trough  = iL[mask_ss].min()
v2_err_pct = (v2_mean - V2_theory) / V2_theory * 100

BLUE, ORANGE, GREY = "#1f77b4", "#ff7f0e", "#888888"

fig, axs = plt.subplots(3, 2, figsize=(13, 10))
fig.suptitle(
    f"dab_top simulation - phase = {PHASE} cycles ({math.degrees(phi):.1f} deg)\n"
    rf"$V_1$={V1:.0f} V, $N$={N:.0f}, $L$={L*1e6:.0f} $\mu$H,"
    rf" $R$={R:.0f} $\Omega$, $f_{{sw}}$={fsw/1e3:.0f} kHz",
    fontsize=11)

# --- Row 0: full transient -------------------------------------------------
axs[0, 0].plot(t_us / 1e3, iL, color=BLUE, lw=0.3, alpha=0.7)
axs[0, 0].set_ylabel(r"$i_L$ [A]", color=BLUE)
axs[0, 0].set_xlabel("Time [ms]")
axs[0, 0].set_title(f"Full sim ({t_us[-1]/1e3:.1f} ms)  -  inductor current")
axs[0, 0].grid(ls=":", alpha=0.4)

axs[0, 1].plot(t_us / 1e3, V2, color=ORANGE, lw=0.6)
axs[0, 1].axhline(V2_theory, color=GREY, lw=0.8, ls="--",
                  label=rf"$V_2^{{th}}$={V2_theory:.2f} V")
axs[0, 1].set_ylabel(r"$V_2$ [V]", color=ORANGE)
axs[0, 1].set_xlabel("Time [ms]")
axs[0, 1].set_title(f"Full sim ({t_us[-1]/1e3:.1f} ms)  -  output voltage")
axs[0, 1].legend(fontsize=9, loc="lower right")
axs[0, 1].grid(ls=":", alpha=0.4)

# --- Row 1: start-up zoom (first N_ZOOM cycles) ----------------------------
axs[1, 0].plot(t_us[mask_su], iL[mask_su], color=BLUE, lw=0.9)
axs[1, 0].set_ylabel(r"$i_L$ [A]", color=BLUE)
axs[1, 0].set_xlabel(r"Time [$\mu$s]")
axs[1, 0].set_title(f"Start-up (first {N_ZOOM} sw cycles)")
axs[1, 0].grid(ls=":", alpha=0.4)
# Shade leading / trailing half-periods to visualise the bridge drive
for k in range(N_ZOOM):
    t0 = k * FULL_PERIOD * DT * 1e6
    axs[1, 0].axvspan(t0, t0 + HALF_PERIOD * DT * 1e6,
                      alpha=0.05, color="green", zorder=0)
    axs[1, 0].axvspan(t0 + HALF_PERIOD * DT * 1e6,
                      t0 + FULL_PERIOD * DT * 1e6,
                      alpha=0.05, color="red", zorder=0)

axs[1, 1].plot(t_us[mask_su], V2[mask_su], color=ORANGE, lw=0.9)
axs[1, 1].set_ylabel(r"$V_2$ [V]", color=ORANGE)
axs[1, 1].set_xlabel(r"Time [$\mu$s]")
axs[1, 1].set_title(f"Start-up (first {N_ZOOM} sw cycles)")
axs[1, 1].grid(ls=":", alpha=0.4)

# --- Row 2: steady-state ---------------------------------------------------
axs[2, 0].plot(t_us[mask_ss], iL[mask_ss], color=BLUE, lw=0.9)
axs[2, 0].axhline(iL_peak,   color=BLUE, lw=0.7, ls=":", alpha=0.7)
axs[2, 0].axhline(iL_trough, color=BLUE, lw=0.7, ls=":", alpha=0.7)
axs[2, 0].set_ylabel(r"$i_L$ [A]", color=BLUE)
axs[2, 0].set_xlabel(r"Time [$\mu$s]")
axs[2, 0].set_title(
    rf"Steady state (last {N_ZOOM} cycles)  -  peak $\pm${iL_peak:.2f} A")
axs[2, 0].grid(ls=":", alpha=0.4)

axs[2, 1].plot(t_us[mask_ss], V2[mask_ss], color=ORANGE, lw=0.9)
axs[2, 1].axhline(v2_mean,  color=ORANGE, lw=1.0, ls="--",
                  label=rf"sim mean = {v2_mean:.3f} V")
axs[2, 1].axhline(V2_theory, color=GREY,  lw=0.8, ls=":",
                  label=rf"theory = {V2_theory:.3f} V")
axs[2, 1].set_ylabel(r"$V_2$ [V]", color=ORANGE)
axs[2, 1].set_xlabel(r"Time [$\mu$s]")
axs[2, 1].set_title(
    f"Steady state ripple pp = {v2_ripple*1e3:.1f} mV  "
    f"({v2_err_pct:+.2f}% vs theory)")
axs[2, 1].legend(fontsize=8, loc="lower right")
axs[2, 1].grid(ls=":", alpha=0.4)

plt.tight_layout(rect=[0, 0, 1, 0.96])
out = "dab_top_results.png"
plt.savefig(out, dpi=150)
print(f"Saved {out}")
print(f"  V2 mean    = {v2_mean:.3f} V   (theory {V2_theory:.3f} V, {v2_err_pct:+.2f}%)")
print(f"  V2 ripple  = {v2_ripple*1e3:.2f} mV pp")
print(f"  i_L peak   = +{iL_peak:.3f} / {iL_trough:.3f} A")

if "--show" in sys.argv:
    plt.show()
