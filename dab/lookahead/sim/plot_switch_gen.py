#!/usr/bin/env python3
# Visualise dab_switch_gen.sv (SPS bridge polarity generator).
#
# Top row : p1/p2 square waves for a sweep of phase_shift values, so you can
#           see p2 slide later relative to the fixed reference p1 as the phase
#           command grows (0/90/180/270 deg).  The shaded band marks the lag.
# Bottom  : average transferred "power" proxy  <p1*p2>  vs phase, which traces
#           the classic SPS triangle (peak at 180 deg) that the phase command
#           actually controls.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PWM_PERIOD = 200                       # clocks per switching period (matches dab_la_pkg)
F_CLK      = 20e6                      # 20 MHz solver clock (dt = 50 ns) -> f_sw = 100 kHz

d = np.genfromtxt("out_switch.csv", delimiter=",", names=True)
phases = np.unique(d["phase"]).astype(int)

# time axis in microseconds: one clock = 1/F_CLK
def t_us(t):
    return t / F_CLK * 1e6

fig = plt.figure(figsize=(12, 9))
gs  = fig.add_gridspec(len(phases) + 1, 1, height_ratios=[1] * len(phases) + [1.4])

for k, ph in enumerate(phases):
    m   = d["phase"] == ph
    t   = d["t"][m]
    p1  = d["p1"][m]
    p2  = d["p2"][m]
    deg = d["deg"][m][0]

    ax = fig.add_subplot(gs[k, 0])
    ax.step(t_us(t), p1, where="post", color="tab:blue", lw=1.6, label="p1 (primary)")
    ax.step(t_us(t), p2, where="post", color="tab:red", lw=1.6, ls="--",
            label="p2 (secondary)")
    # shade where the two bridges disagree -> this is the transfer interval
    ax.fill_between(t_us(t), -1, 1, where=(p1 != p2), step="post",
                    color="0.85", zorder=0)
    # period boundaries
    for b in range(0, int(t.max()) + 1, PWM_PERIOD):
        ax.axvline(t_us(b), color="0.6", ls=":", lw=0.8)
    ax.set_ylim(-1.6, 1.6)
    ax.set_yticks([-1, 1])
    ax.set_ylabel(f"φ={int(ph)} clk\n({deg:.0f}°)")
    ax.grid(True, axis="x", alpha=0.3)
    if k == 0:
        ax.legend(loc="upper right", ncol=2, fontsize=9)
        ax.set_title("dab_switch_gen — p1/p2 polarities vs phase_shift "
                     "(shaded = p1≠p2, the power-transfer interval)")
    if k == len(phases) - 1:
        ax.set_xlabel("time [µs]   (1 clk = 50 ns, 200 clk = one 100 kHz period)")

# ── transfer characteristic: <p1*p2> over a full period for every phase 0..100
ph_sweep = np.arange(0, PWM_PERIOD + 1)
cnt      = np.arange(PWM_PERIOD)
half     = PWM_PERIOD // 2
p1_ref   = np.where(cnt < half, 1, -1)
avg = []
for ph in ph_sweep:
    pos = (cnt - ph) % PWM_PERIOD
    p2v = np.where(pos < half, 1, -1)
    avg.append(np.mean(p1_ref * p2v))     # +1 (in phase) .. -1 (anti-phase)
avg = np.array(avg)
# power transferred to the load rises as the bridges spend more time opposed
transfer = (1 - avg) / 2                   # 0 at 0deg, 1 at 180deg

axp = fig.add_subplot(gs[len(phases), 0])
axp.plot(ph_sweep / PWM_PERIOD * 360, transfer, color="tab:green", lw=2)
for ph in phases:
    axp.axvline(ph / PWM_PERIOD * 360, color="0.6", ls=":", lw=0.8)
    axp.plot(ph / PWM_PERIOD * 360, transfer[ph], "o", color="tab:green")
axp.set_title("SPS transfer characteristic  (1−⟨p1·p2⟩)/2  vs phase  "
              "— what the phase command actually controls")
axp.set_xlabel("phase shift [degrees]")
axp.set_ylabel("normalised\npower transfer")
axp.set_xticks(np.arange(0, 361, 45))
axp.grid(True, alpha=0.3)

fig.tight_layout()
fig.savefig("switch_gen.png", dpi=110)
print("wrote switch_gen.png")
