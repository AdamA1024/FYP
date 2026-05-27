"""Plot buck_verlet_mixed_precision convergence results from trace.csv."""
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
CSV  = HERE / "trace.csv"

# ---- load ----
data = defaultdict(lambda: {"c": [], "s": [], "v": [], "i": []})
with CSV.open() as f:
    rd = csv.DictReader(f)
    for row in rd:
        d = data[row["tag"]]
        d["c"].append(int(row["cycle"]))
        d["s"].append(int(row["s"]))
        d["v"].append(float(row["v_out"]))
        d["i"].append(float(row["i_out"]))

def sliding_mean(xs, w):
    """Moving average with window w (= PWM period), centered."""
    out = [0.0]*len(xs)
    s = 0.0
    for i, x in enumerate(xs):
        s += x
        if i >= w: s -= xs[i-w]
        out[i] = s / min(i+1, w)
    return out

# ---- Fig 1: PWM convergence at 4 kR values (P=100, 50% duty), transient ----
pwm_tags = [
    ("pwm50 kR=0.0000", "kR = 0.0000 (lossless, rings)"),
    ("pwm50 kR=0.0002", "kR = 0.0002 (very light damping)"),
    ("pwm50 kR=0.0020", "kR = 0.0020 (R ~ 1.5 ohm)"),
    ("pwm50 kR=0.0100", "kR = 0.0100 (heavy load)"),
]
fig, axes = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True)
for tag, label in pwm_tags:
    d = data[tag]
    # raw waveform (light) + PWM-period sliding mean (bold) so the
    # 6 V settle is visible through the 24 V pk-pk ripple.
    avg = sliding_mean(d["v"], 100)
    axes[0].plot(d["c"], d["v"],  lw=0.4, alpha=0.25)
    line, = axes[0].plot(d["c"], avg, lw=1.6, label=label)
    axes[1].plot(d["c"], sliding_mean(d["i"], 100),
                 lw=1.4, color=line.get_color(), label=label)
axes[0].axhline(6.0, color="k", ls="--", lw=0.8, alpha=0.7, label="target 6 V")
axes[0].set_ylabel("v_out [V]   (faint = raw, bold = 1-period mean)")
axes[0].set_title("50% PWM (period = 100 clk), Vin = 12 V — convergence vs kR  (first 3000 cycles)")
axes[0].legend(loc="lower right", fontsize=8, ncol=2)
axes[0].grid(alpha=0.3)
axes[0].set_ylim(-15, 30)
axes[1].set_ylabel("i_out [A]   (1-period mean)")
axes[1].set_xlabel("clock cycle")
axes[1].axhline(0.0, color="k", ls="--", lw=0.7, alpha=0.5)
axes[1].grid(alpha=0.3)
axes[1].set_xlim(0, 30000)
fig.tight_layout()
fig.savefig(HERE / "fig1_pwm_kR_sweep.png", dpi=120)
plt.close(fig)

# ---- Fig 2: zoom on steady-state ripple (last 500 cycles) for kR=0.02 ----
fig, ax1 = plt.subplots(figsize=(11, 5))
ax2 = ax1.twinx()
d = data["pwm50 kR=0.0020"]
n = len(d["c"]); start = n - 500
c = d["c"][start:]; v = d["v"][start:]; i = d["i"][start:]; s = d["s"][start:]
ax1.plot(c, v, color="C0", lw=1.0, label="v_out")
ax1.axhline(6.0, color="k", ls="--", lw=0.7, alpha=0.6, label="6 V target")
ax2.plot(c, i, color="C3", lw=1.0, label="i_out")
# shade switch ON
ax1.fill_between(c, -100, 100, where=[bool(x) for x in s], color="C2",
                 alpha=0.07, step="mid", label="v_in_s ON")
ax1.set_ylim(min(v)-1, max(v)+1)
ax1.set_xlabel("clock cycle")
ax1.set_ylabel("v_out [V]", color="C0")
ax2.set_ylabel("i_out [A]", color="C3")
ax1.set_title("Steady-state ripple — 50% PWM, P=100, kR=0.002 (last 500 cycles)")
ax1.legend(loc="upper left", fontsize=9)
ax2.legend(loc="upper right", fontsize=9)
ax1.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(HERE / "fig2_ripple_zoom.png", dpi=120)
plt.close(fig)

# ---- Fig 3: PWM frequency sweep (P = 10, 100, 1000) at kR=0.02 ----
period_tags = [
    ("pwm50 P=10 kR=0.020",   "PWM period = 10 clk (fast)"),
    ("pwm50 kR=0.020",        "PWM period = 100 clk (medium)"),
    ("pwm50 P=1000 kR=0.020", "PWM period = 1000 clk (slow)"),
]
# (Fig 3 retired — was a PWM-period sweep with the old plant.  The canonical
# P=100 / f_sw=1 MHz case is now well-filtered, so the multi-period sweep
# is no longer interesting.)

# ---- Fig 4: DC response (no PWM) — proves unity DC gain ----
dc_tags = [
    ("DC v_in_s=12 kR=0.0000", "kR = 0.0000  (lossless — undamped ring)"),
    ("DC v_in_s=12 kR=0.0020", "kR = 0.0020"),
    ("DC v_in_s=12 kR=0.0100", "kR = 0.0100"),
]
fig, ax = plt.subplots(figsize=(12, 6))
for tag, label in dc_tags:
    d = data[tag]
    ax.plot(d["c"], d["v"], lw=1.0, label=label)
ax.axhline(12.0, color="k", ls="--", lw=0.8, alpha=0.7, label="Vin = 12 V target")
# annotate final values
for tag, label in dc_tags:
    d = data[tag]
    ax.annotate(f"final = {d['v'][-1]:.3f} V",
                xy=(len(d["v"])-1, d["v"][-1]), fontsize=8,
                xytext=(len(d["v"])-5000, d["v"][-1] + 1.5),
                arrowprops=dict(arrowstyle="->", lw=0.6))
ax.set_xlabel("clock cycle")
ax.set_ylabel("v_out [V]")
ax.set_title("DC response (constant v_in_s = 12 V) — unity DC gain check")
ax.legend(loc="upper right", fontsize=9)
ax.grid(alpha=0.3)
ax.set_xlim(0, 20000)
ax.set_ylim(-5, 28)
fig.tight_layout()
fig.savefig(HERE / "fig4_dc_response.png", dpi=120)
plt.close(fig)

print("Wrote:")
for p in ("fig1_pwm_kR_sweep.png", "fig2_ripple_zoom.png",
          "fig3_pwm_period_sweep.png", "fig4_dc_response.png"):
    print(" ", HERE / p)
