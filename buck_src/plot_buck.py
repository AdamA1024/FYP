#!/usr/bin/env python3
"""Plot buck converter load-step simulation results from buck_sim.csv."""

import csv
import sys
import os

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
except ImportError:
    print("matplotlib not found – install with:  pip install matplotlib")
    sys.exit(1)

csv_file = "buck_sim.csv"
if not os.path.exists(csv_file):
    print(f"{csv_file} not found – run 'make sim' first")
    sys.exit(1)

cycles, time_us, vin_vals, i_vals, v_vals = [], [], [], [], []
with open(csv_file) as f:
    reader = csv.DictReader(f)
    for row in reader:
        cycles.append(int(row["cycle"]))
        time_us.append(float(row["time_us"]))
        vin_vals.append(float(row["vin_V"]))
        i_vals.append(float(row["i_A"]))
        v_vals.append(float(row["v_V"]))

# Identify step times (where Vin changes)
step_times = []
for k in range(1, len(vin_vals)):
    if vin_vals[k] != vin_vals[k - 1]:
        step_times.append(time_us[k])

# Assign a colour to each phase for background shading
phase_colors = ["#e8f4e8", "#fff3e0", "#e8f0ff"]   # green / orange / blue tints
phase_label  = [f"Vin = {v:.1f} V" for v in [12.0, 9.0, 15.0]]
phase_bounds = [0.0] + step_times + [time_us[-1]]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
fig.suptitle(
    "Buck Converter Digital Twin – Input Voltage Step Test\n"
    "R=5 Ω, L=10 µH, C=100 µF, duty=50 %  |  V_out = 0.5 × Vin",
    fontsize=11
)

# --- Shade phases ---
for ax in (ax1, ax2):
    for p, (t0, t1) in enumerate(zip(phase_bounds[:-1], phase_bounds[1:])):
        ax.axvspan(t0, t1, color=phase_colors[p % len(phase_colors)], alpha=0.5, zorder=0)
    for t in step_times:
        ax.axvline(t, color="red", linestyle="--", linewidth=1.0, zorder=2)

# --- Voltage panel ---
ax1.plot(time_us, v_vals, color="tab:blue", linewidth=0.7, zorder=3)
ax1.set_ylabel("Capacitor Voltage (V)")
ax1.set_ylim(bottom=0)
ax1.grid(True, alpha=0.3, zorder=1)

# Phase legend patches
patches = [mpatches.Patch(color=phase_colors[p], label=phase_label[p])
           for p in range(len(phase_label))]
patches.append(plt.Line2D([0], [0], color="red",   linestyle="--", label="Vin step"))
ax1.legend(handles=patches, loc="lower right", fontsize=8)

# --- Current panel ---
ax2.plot(time_us, i_vals, color="tab:green", linewidth=0.7, zorder=3)
ax2.set_ylabel("Inductor Current (A)")
ax2.set_xlabel("Time (µs)")
ax2.grid(True, alpha=0.3, zorder=1)

plt.tight_layout()
out_file = "buck_sim.png"
plt.savefig(out_file, dpi=150)
print(f"Plot saved to {out_file}")
plt.show()
