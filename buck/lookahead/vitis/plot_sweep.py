#!/usr/bin/env python3
"""Plot the buck Vin x R sweep captured over UART (putty.log.txt).

Robust to the UART-overrun data loss at the start of the dump: it parses only
well-formed CSV rows (7 fields, numeric) between the BEGIN/END markers, so
truncated / missing operating points are simply plotted with whatever samples
survived.

Outputs:
  sweep_transients.png  - Vout(t) per operating point, one subplot per load R
  sweep_dc.png          - steady-state Vout vs Vin per R (DC transfer char.)
and prints a steady-state summary table.
"""
import sys
from collections import defaultdict
import matplotlib.pyplot as plt

LOG = sys.argv[1] if len(sys.argv) > 1 else "putty.log.txt"
SS_T_US = 300            # treat samples at t >= this as steady state
EXCLUDE_R = {5.0}        # loads to drop from the figures (heavily damped, crowds plots)
RIPPLE_VIN, RIPPLE_R = 12.0, 20.0   # operating point for the steady-state ripple zoom

# point -> dict(vin, R, t[], vout[], iout[])
pts = defaultdict(lambda: {"vin": None, "R": None, "t": [], "vout": [], "iout": []})

n_ok = n_bad = 0
in_block = False
with open(LOG, errors="replace") as fh:
    for line in fh:
        s = line.strip()
        if s.startswith("# BUCK_SWEEP_BEGIN"):
            in_block = True; continue
        if s.startswith("# BUCK_SWEEP_END"):
            break
        if not in_block or s.startswith("#") or s.startswith("point,"):
            continue
        f = s.split(",")
        if len(f) != 7:
            n_bad += 1; continue
        try:
            pt = int(f[0]); vin = float(f[1]); R = float(f[2])
            t = float(f[4]); vout = float(f[5]); iout = float(f[6])
        except ValueError:
            n_bad += 1; continue
        d = pts[pt]
        d["vin"], d["R"] = vin, R
        d["t"].append(t); d["vout"].append(vout); d["iout"].append(iout)
        n_ok += 1

print(f"parsed {n_ok} good rows, skipped {n_bad} malformed")

# ---- steady-state table ------------------------------------------------------
def steady(d, key):
    vals = [v for t, v in zip(d["t"], d[key]) if t >= SS_T_US]
    return sum(vals) / len(vals) if vals else float("nan")

print(f"\n{'pt':>3} {'Vin':>5} {'R':>5} {'n':>4} {'Vout_ss':>8} {'Iout_ss':>8} {'D*Vin':>6}")
rows = sorted(pts.items())
for pt, d in rows:
    print(f"{pt:>3} {d['vin']:>5.1f} {d['R']:>5.1f} {len(d['t']):>4} "
          f"{steady(d,'vout'):>8.3f} {steady(d,'iout'):>8.3f} {0.5*d['vin']:>6.2f}")

# ---- figure 1: transients, one subplot per R --------------------------------
loads = sorted({d["R"] for d in pts.values()} - EXCLUDE_R)
fig, axes = plt.subplots(1, len(loads), figsize=(4.5 * len(loads), 4.2),
                         sharex=True, sharey=True, squeeze=False)
axes = axes[0]
cmap = plt.get_cmap("viridis")
vins = sorted({d["vin"] for d in pts.values()})
vcol = {v: cmap(i / max(1, len(vins) - 1)) for i, v in enumerate(vins)}

for ax, R in zip(axes, loads):
    for pt, d in rows:
        if d["R"] != R:
            continue
        # sort by time in case rows arrived out of order
        ts, vs = zip(*sorted(zip(d["t"], d["vout"])))
        ax.plot(ts, vs, lw=0.9, color=vcol[d["vin"]],
                label=f"Vin={d['vin']:.0f}")
    ax.set_title(f"R = {R:.0f} ohm")
    ax.set_xlabel("t (us)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=1)
axes[0].set_ylabel("Vout (V)")
fig.suptitle("Buck twin: Vout transient, 50% duty, swept Vin x R")
fig.tight_layout()
fig.savefig("sweep_transients.png", dpi=130)
print("\nwrote sweep_transients.png")

# ---- figure 2: DC transfer characteristic -----------------------------------
fig2, ax2 = plt.subplots(figsize=(6, 4.5))
by_R = defaultdict(list)
for pt, d in rows:
    if d["R"] in EXCLUDE_R:
        continue
    by_R[d["R"]].append((d["vin"], steady(d, "vout")))
for R in loads:
    series = sorted(by_R[R])
    xs = [v for v, _ in series]; ys = [vo for _, vo in series]
    ax2.plot(xs, ys, "o-", label=f"R={R:.0f}")
# ideal D*Vin reference (over the Vin range actually plotted)
xs = sorted({v for s in by_R.values() for v, _ in s})
ax2.plot(xs, [0.5 * v for v in xs], "k--", lw=1, label="ideal 0.5*Vin")
ax2.set_xlabel("Vin (V)"); ax2.set_ylabel("steady-state Vout (V)")
ax2.set_title("Buck DC transfer (50% duty)")
ax2.grid(alpha=0.3); ax2.legend()
fig2.tight_layout()
fig2.savefig("sweep_dc.png", dpi=130)
print("wrote sweep_dc.png")

# ---- figure 3: steady-state ripple zoom for one operating point -------------
# NOTE: f_sw = 1 MHz (Tsw = 1 us) but we sample every 2 us, so the true
# switching ripple is undersampled -> what we see is the *aliased* ripple. The
# pk-pk below is the sampled envelope, not the real 1 MHz ripple amplitude.
target = next((d for _, d in rows
               if d["vin"] == RIPPLE_VIN and d["R"] == RIPPLE_R), None)
if target is None:
    print(f"no data for Vin={RIPPLE_VIN}, R={RIPPLE_R}; skipping ripple plot")
else:
    ss = [(t, v) for t, v in zip(target["t"], target["vout"]) if t >= SS_T_US]
    ss.sort()
    ts = [t for t, _ in ss]; vs = [v for _, v in ss]
    mean = sum(vs) / len(vs)
    pkpk = max(vs) - min(vs)
    fig3, ax3 = plt.subplots(figsize=(8, 4))
    ax3.plot(ts, vs, "o-", ms=3, lw=0.9, color="tab:blue")
    ax3.axhline(mean, color="k", ls="--", lw=1, label=f"mean {mean:.3f} V")
    ax3.fill_between(ts, min(vs), max(vs), color="tab:blue", alpha=0.08)
    ax3.set_xlabel("t (us)"); ax3.set_ylabel("Vout (V)")
    ax3.set_title(f"Steady-state ripple  Vin={RIPPLE_VIN:.0f} V, R={RIPPLE_R:.0f} ohm  "
                  f"(pk-pk {pkpk*1e3:.0f} mV, aliased @ 2 us sampling)")
    ax3.grid(alpha=0.3); ax3.legend()
    fig3.tight_layout()
    fig3.savefig("sweep_ripple.png", dpi=130)
    print(f"wrote sweep_ripple.png  (mean {mean:.3f} V, sampled pk-pk {pkpk*1e3:.0f} mV)")
