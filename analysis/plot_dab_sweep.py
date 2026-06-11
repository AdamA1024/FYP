#!/usr/bin/env python3
"""Plot the DAB operating-point sweep (V1 x R, fixed phase) from resultsDAB.csv.

Produces:
  dab_sweep_v2.png     - V2(t) families: one panel per V1, a curve per R
  dab_sweep_byR.png    - V2(t) families: one panel per R, a curve per V1
  dab_sweep_il.png     - i_L(t) (aliased AC -> envelope), one panel per V1
and prints a steady-state + time-constant table.
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CSV = "analysis/resultsDAB.csv"
Co  = 470e-6   # for tau = R*Co reference

df = pd.read_csv(CSV, comment="#")
V1S = sorted(df.v1.unique())
RS  = sorted(df.R.unique())
print(f"{len(V1S)} V1 x {len(RS)} R = {df.point.nunique()} points, "
      f"{df[df.point==0].shape[0]} samples/point")

def curve(v1, R):
    s = df[(df.v1 == v1) & (df.R == R)].sort_values("sample")
    return s.t_us.to_numpy() / 1000.0, s.v2.to_numpy(), s.il.to_numpy()   # t[ms], V2, iL

# ── steady-state + 63% time constant table ───────────────────────────────────
rows = []
for v1 in V1S:
    for R in RS:
        t, v2, il = curve(v1, R)
        vss = v2[-32:].mean()                       # last ~3 ms average
        # 63.2% rise time -> tau (first crossing of 0.632*Vss)
        thr = 0.632 * vss
        i = np.argmax(v2 >= thr) if np.any(v2 >= thr) else -1
        tau = t[i] if i > 0 else np.nan
        rows.append((v1, R, vss, tau, R * Co * 1e3))
tab = pd.DataFrame(rows, columns=["V1", "R", "V2_ss", "tau_meas_ms", "RCo_ms"])
print("\n=== steady state & time constant ===")
print(tab.to_string(index=False,
      formatters={"V2_ss": "{:7.2f}".format, "tau_meas_ms": "{:6.2f}".format,
                  "RCo_ms": "{:6.2f}".format}))

# ── Fig 1: one panel per V1, curve per R ──────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)
for ax, v1 in zip(axes.ravel(), V1S):
    for R in RS:
        t, v2, _ = curve(v1, R)
        ax.plot(t, v2, lw=1.4, label=f"R={R:g} Ω")
    ax.set_title(f"V1 = {v1:g} V")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_ylabel("V2 (V)")
for ax in axes[-1]:
    ax.set_xlabel("t (ms)")
fig.suptitle("DAB twin — V2 charge transient vs load R (phase = 45°)", fontweight="bold")
fig.tight_layout()
fig.savefig("analysis/dab_sweep_v2.png", dpi=130)
print("\nwrote analysis/dab_sweep_v2.png")

# ── Fig 2: one panel per R, curve per V1 (shows V2 ∝ V1 at fixed R) ────────────
fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)
for ax, R in zip(axes.ravel(), RS):
    for v1 in V1S:
        t, v2, _ = curve(v1, R)
        ax.plot(t, v2, lw=1.4, label=f"V1={v1:g} V")
    ax.set_title(f"R = {R:g} Ω   (τ=RCo≈{R*Co*1e3:.1f} ms)")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_ylabel("V2 (V)")
for ax in axes[-1]:
    ax.set_xlabel("t (ms)")
fig.suptitle("DAB twin — V2 charge transient vs V1 (phase = 45°)", fontweight="bold")
fig.tight_layout()
fig.savefig("analysis/dab_sweep_byR.png", dpi=130)
print("wrote analysis/dab_sweep_byR.png")

# ── Fig 3: i_L (aliased AC) — show as scatter envelope, one panel per V1 ───────
fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)
for ax, v1 in zip(axes.ravel(), V1S):
    for R in RS:
        t, _, il = curve(v1, R)
        ax.plot(t, il, lw=0.5, alpha=0.6, label=f"R={R:g} Ω")
    ax.set_title(f"V1 = {v1:g} V")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_ylabel("i_L (A)  [aliased]")
for ax in axes[-1]:
    ax.set_xlabel("t (ms)")
fig.suptitle("DAB twin — i_L (instantaneous, aliased 500 kHz AC)", fontweight="bold")
fig.tight_layout()
fig.savefig("analysis/dab_sweep_il.png", dpi=130)
print("wrote analysis/dab_sweep_il.png")
