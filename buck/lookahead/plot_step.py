"""Plot the Vin-step response traces written by sim_step_main.cpp.

Reads step_trace.csv and produces fig_step_response.png with two panels:
  (a) effect of step size at fixed kR
  (b) effect of damping kR at fixed step size

Each trace shows the raw v_out (faint) and the per-PWM-period mean (bold),
with v_ss2 and a 2% settling band marked, and a dot at the settling instant.
"""
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
CSV  = HERE / "step_trace.csv"

PWM_PERIOD     = 100     # clocks per PWM period (matches sim_step_main.cpp)
DT_NS          = 10.0    # clock period -> 10 ns
BAND_FRAC      = 0.02    # 2% settling band

# Cases as ordered in sim_step_main.cpp, with the pre-step clock count we
# need to know where the step happens.  Keep in sync with cases[] in C++.
CASES = [
    # tag                       pre_step  group       label
    ("step12to18 kR=0.002",      60000,  "size",      "Vin 12 → 18 V"),
    ("step12to8  kR=0.002",      60000,  "size",      "Vin 12 → 8 V"),
    ("step12to24 kR=0.002",      60000,  "size",      "Vin 12 → 24 V"),
    ("step12to18 kR=0.010",      20000,  "damping",   "kR = 0.0100 (heavy)"),
    ("step12to18 kR=0.002",      60000,  "damping",   "kR = 0.0020 (typical)"),
    ("step12to18 kR=0.0002",    200000,  "damping",   "kR = 0.0002 (light)"),
]

# ---- load ----
data = defaultdict(lambda: {"c": [], "v": [], "i": [], "s": []})
with CSV.open() as f:
    rd = csv.DictReader(f)
    for row in rd:
        d = data[row["tag"]]
        d["c"].append(int(row["cycle"]))
        d["v"].append(float(row["v_out"]))
        d["i"].append(float(row["i_out"]))
        d["s"].append(int(row["s"]))

def per_period_mean(v, P):
    n = len(v) // P
    return [sum(v[p*P:(p+1)*P]) / P for p in range(n)]

def settle_info(vmean, step_period, band):
    """Last period whose mean is outside the band; settling = next boundary."""
    v_ss2 = sum(vmean[-20:]) / 20
    band_v = band * abs(v_ss2)
    last_out = -1
    for p in range(len(vmean) - 1, step_period - 1, -1):
        if abs(vmean[p] - v_ss2) > band_v:
            last_out = p
            break
    if last_out < 0:
        return v_ss2, band_v, 0
    return v_ss2, band_v, (last_out + 1 - step_period) * PWM_PERIOD

def plot_panel(ax, group_name, members):
    for tag, pre, _, label in members:
        d = data[tag]
        if not d["c"]:
            continue
        v = d["v"]
        step_clk = pre
        # Time axis in microseconds relative to the step instant.
        t_us = [(c - step_clk) * DT_NS / 1000.0 for c in d["c"]]

        vmean = per_period_mean(v, PWM_PERIOD)
        step_period = step_clk // PWM_PERIOD
        v_ss2, band_v, ts_clk = settle_info(vmean, step_period, BAND_FRAC)
        ts_us = ts_clk * DT_NS / 1000.0
        # per-period mean time axis: place at end-of-period
        t_mean_us = [(((p + 1) * PWM_PERIOD - 1) - step_clk) * DT_NS / 1000.0
                     for p in range(len(vmean))]

        ax.plot(t_us, v, lw=0.3, alpha=0.25)
        line, = ax.plot(t_mean_us, vmean, lw=1.6, label=f"{label}  (ts={ts_us:.1f} µs)")
        c = line.get_color()
        # mark settling instant
        if ts_clk > 0:
            # find vmean value at the settling boundary
            sp = step_period + ts_clk // PWM_PERIOD
            sp = min(sp, len(vmean) - 1)
            ax.plot([ts_us], [vmean[sp]], "o", color=c, ms=6,
                    mec="k", mew=0.8, zorder=5)

        # 2% band around v_ss2 (use last group member's band for cleanliness)
        ax.axhline(v_ss2, color=c, ls=":", lw=0.7, alpha=0.6)

    ax.axvline(0.0, color="k", ls="--", lw=0.8, alpha=0.7)
    ax.set_xlabel("time relative to step [µs]")
    ax.set_ylabel("v_out [V]")
    ax.set_title(f"Vin step response — {group_name}")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)

groups = {
    "step size (kR = 0.002)":
        [m for m in CASES if m[2] == "size"],
    "damping kR (step 12 → 18 V)":
        [m for m in CASES if m[2] == "damping"],
}

fig, axes = plt.subplots(2, 1, figsize=(12, 8))
for ax, (name, members) in zip(axes, groups.items()):
    plot_panel(ax, name, members)

# Tighten x-window per panel: focus on the transient, not the long settle tail.
axes[0].set_xlim(-20, 80)    # kR=0.002 settles in ~30 µs
axes[1].set_xlim(-30, 400)   # kR sweep — lightest case needs ~325 µs

fig.tight_layout()
out = HERE / "fig_step_response.png"
fig.savefig(out, dpi=120)
plt.close(fig)
print(f"Wrote {out}")
