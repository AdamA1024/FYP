#!/usr/bin/env python3
"""True steady-state V2 ripple and i_L waveform (20 ns resolution, from the twin
RTL in Verilator) at V1=100 V, R=5 ohm, phase=45 deg. The hardware runs identical
RTL; its V2 steady-state matches to <1%, but PS/AXI reads (~0.5 us) alias these
500 kHz waveforms, so the true shapes come from the cycle-accurate model."""
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

d = pd.read_csv("analysis/dab_fine_100_5.csv")
t = d.t_ns.to_numpy() / 1000.0          # us
sl = t <= 6.0                            # small slice: 3 switching periods (Tsw=2us)
tt, v2, il = t[sl], d.v2.to_numpy()[sl], d.il.to_numpy()[sl]

fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

# i_L true shape
a1.plot(tt, il, color="C1", lw=1.6)
a1.set_ylabel("i_L (A)")
a1.set_title("DAB i_L — true waveform (V1=100 V, R=5 Ω, phase=45°, steady state)")
a1.grid(alpha=0.3)
for x in np.arange(0, 6.1, 2.0):        # mark switching periods
    a1.axvline(x, color="grey", ls=":", lw=0.8)
a1.annotate("Tsw = 2 µs (500 kHz)", xy=(2, a1.get_ylim()[1]),
            xytext=(2.1, il.max()*0.9), fontsize=9, color="grey")

# V2 ripple (zoomed)
a2.plot(tt, v2*1000, color="C0", lw=1.6)
a2.set_ylabel("V2 (mV)")
a2.set_xlabel("t (µs)")
vpp = (d.v2.max() - d.v2.min()) * 1000
a2.set_title(f"V2 steady-state ripple — {vpp:.1f} mV pk-pk on {d.v2.mean():.3f} V DC")
a2.grid(alpha=0.3)
for x in np.arange(0, 6.1, 2.0):
    a2.axvline(x, color="grey", ls=":", lw=0.8)

fig.tight_layout()
fig.savefig("analysis/dab_fine_100_5.png", dpi=140)
print("wrote analysis/dab_fine_100_5.png")
print(f"i_L: {il.min():.2f} .. {il.max():.2f} A  (pk-pk {il.max()-il.min():.2f} A)")
print(f"V2 : {d.v2.mean():.4f} V DC, ripple {vpp:.2f} mV pk-pk")
