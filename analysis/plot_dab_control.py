#!/usr/bin/env python3
"""Plot the DAB closed-loop PI phase regulator. Reads either the Verilator
verification log (analysis/dab_ctrl_sim.csv: tick,t_ms,v1,vref,v2,phase) or the
hardware dump (tick,t_us,v1,vref,v2,phase between # DAB_CTRL_BEGIN/END)."""
import sys, numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "analysis/dab_ctrl_sim.csv"
df = pd.read_csv(path, comment="#")
t = (df.t_ms.to_numpy() if "t_ms" in df else df.t_us.to_numpy()/1000.0)  # ms

fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True,
                                 gridspec_kw={"height_ratios": [3, 1.4, 1.4]})

# V2 vs Vref
a1.plot(t, df.vref, "k--", lw=1.5, label="Vref")
a1.plot(t, df.v2,  "C0", lw=1.6, label="V2 (twin)")
a1.set_ylabel("V2 / Vref (V)")
a1.set_title("DAB closed-loop PI phase regulator — line rejection + reference tracking")
a1.grid(alpha=0.3); a1.legend(loc="upper right")
a1.axvspan(t[0], 120, color="C1", alpha=0.05)
a1.axvspan(120, t[-1], color="C2", alpha=0.05)
a1.text(60, a1.get_ylim()[1]*0.96, "line disturbance (Vref fixed)",
        ha="center", va="top", fontsize=9, color="C1")
a1.text((120+t[-1])/2, a1.get_ylim()[1]*0.96, "reference tracking (V1 fixed)",
        ha="center", va="top", fontsize=9, color="C2")

# V1 line input
a2.plot(t, df.v1, "C3", lw=1.5)
a2.set_ylabel("V1 (V)"); a2.grid(alpha=0.3)

# phase command
a3.step(t, df.phase, "C4", lw=1.5, where="post")
a3.set_ylabel("phase (clk)"); a3.set_xlabel("t (ms)"); a3.grid(alpha=0.3)

fig.tight_layout()
out = "analysis/dab_control_sim.png" if "sim" in path else "analysis/dab_control_hw.png"
fig.savefig(out, dpi=140)
print("wrote", out)

# settled-error summary
for lab, (a, b) in {"V1=100": (180, 200), "V1=120": (380, 400), "V1=80": (580, 600),
                     "Vref=18": (680, 700), "Vref=8": (780, 800)}.items():
    seg = df[(df.tick >= a) & (df.tick < b)]
    if len(seg):
        print(f"  {lab:8s}: V2={seg.v2.mean():6.3f} V  Vref={seg.vref.iloc[0]:.0f}  "
              f"err={seg.vref.iloc[0]-seg.v2.mean():+.3f} V  phase~{seg.phase.mean():.1f}")
