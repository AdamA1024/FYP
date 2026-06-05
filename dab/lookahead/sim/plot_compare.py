#!/usr/bin/env python3
# Overlay SV engine (out.csv) on the golden model (ref.csv): full run + zoomed window.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ref = np.genfromtxt("ref.csv", delimiter=",", names=True)
sv = np.genfromtxt("out.csv", delimiter=",", names=True)
dt = 50e-9
tr = ref["idx"] * dt * 1e3   # ms
ts = sv["idx"] * dt * 1e3

fig, ax = plt.subplots(2, 2, figsize=(12, 7))

ax[0, 0].plot(tr, ref["V2_ref"], "k-", lw=1, label="golden")
ax[0, 0].plot(ts, sv["V2_sv"], "r--", lw=0.8, label="SV engine")
ax[0, 0].set_title("V2 (full 30 ms)"); ax[0, 0].set_xlabel("t [ms]"); ax[0, 0].set_ylabel("V2 [V]"); ax[0, 0].legend()

ax[0, 1].plot(tr, ref["i_ref"], "k-", lw=1, label="golden")
ax[0, 1].plot(ts, sv["i_sv"], "r--", lw=0.8, label="SV engine")
ax[0, 1].set_title("i_L (full 30 ms)"); ax[0, 1].set_xlabel("t [ms]"); ax[0, 1].set_ylabel("i_L [A]"); ax[0, 1].legend()

# zoom: last 3 switching periods (steady state)
m = tr > (30.0 - 0.03)
ms = ts > (30.0 - 0.03)
ax[1, 0].plot(tr[m], ref["V2_ref"][m], "k-", lw=1.2, label="golden")
ax[1, 0].plot(ts[ms], sv["V2_sv"][ms], "r--", lw=1, label="SV engine")
ax[1, 0].set_title("V2 (steady-state zoom)"); ax[1, 0].set_xlabel("t [ms]"); ax[1, 0].set_ylabel("V2 [V]"); ax[1, 0].legend()

ax[1, 1].plot(tr[m], ref["i_ref"][m], "k-", lw=1.2, label="golden")
ax[1, 1].plot(ts[ms], sv["i_sv"][ms], "r--", lw=1, label="SV engine")
ax[1, 1].set_title("i_L (steady-state zoom)"); ax[1, 1].set_xlabel("t [ms]"); ax[1, 1].set_ylabel("i_L [A]"); ax[1, 1].legend()

fig.suptitle("DAB look-ahead engine vs golden model (Q8.24 / Q4.28)")
fig.tight_layout()
fig.savefig("dab_compare.png", dpi=110)
print("wrote dab_compare.png")
