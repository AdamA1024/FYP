#!/usr/bin/env python3
# Runtime-R (load-step) comparison: optimized engine (dab3, background γ-fold)
# vs the ideal golden reference (instantaneous γ).  Shows that moving the γ-update
# off the recurrence loop tracks an abrupt load step to within a few mV / mA.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

d = np.genfromtxt("out_rstep.csv", delimiter=",", names=True)
dt = 50e-9
t = d["idx"] * dt * 1e3          # ms

Co = 470e-6
gamma = d["gamma_q428"] / (1 << 28)
R = dt / (gamma * Co)            # recover load R(t) from the driven γ

# Detect the load-step instants (where γ changes) for vertical markers.
step_idx = np.where(np.diff(d["gamma_q428"]) != 0)[0]
step_t = t[step_idx]

err_v = d["V2_sv"] - d["V2_ref"]
err_i = d["i_sv"] - d["i_ref"]

fig, ax = plt.subplots(2, 2, figsize=(13, 8))


def mark_steps(a):
    for st in step_t:
        a.axvline(st, color="0.6", ls=":", lw=1)


# ── V2 full overlay + load profile ───────────────────────────────────────────
ax[0, 0].plot(t, d["V2_ref"], "k-", lw=1.0, label="golden (instant γ)")
ax[0, 0].plot(t, d["V2_sv"], "r--", lw=0.8, label="dab3 (background γ-fold)")
mark_steps(ax[0, 0])
ax[0, 0].set_title("V2 — full run across load steps")
ax[0, 0].set_xlabel("t [ms]"); ax[0, 0].set_ylabel("V2 [V]"); ax[0, 0].legend(loc="lower right")
axR = ax[0, 0].twinx()
axR.plot(t, R, color="tab:blue", lw=0.8, alpha=0.5)
axR.set_ylabel("R load [Ω]", color="tab:blue"); axR.tick_params(axis="y", labelcolor="tab:blue")

# ── i_L full overlay ─────────────────────────────────────────────────────────
ax[0, 1].plot(t, d["i_ref"], "k-", lw=1.0, label="golden")
ax[0, 1].plot(t, d["i_sv"], "r--", lw=0.8, label="dab3")
mark_steps(ax[0, 1])
ax[0, 1].set_title("i_L — full run")
ax[0, 1].set_xlabel("t [ms]"); ax[0, 1].set_ylabel("i_L [A]"); ax[0, 1].legend(loc="lower right")

# ── Tracking error over the whole run ────────────────────────────────────────
ax[1, 0].plot(t, err_v * 1e3, "tab:purple", lw=0.7, label="V2 error")
ax[1, 0].plot(t, err_i * 1e3, "tab:green", lw=0.7, label="i_L error", alpha=0.7)
mark_steps(ax[1, 0])
ax[1, 0].set_title("engine − golden  (full run)")
ax[1, 0].set_xlabel("t [ms]"); ax[1, 0].set_ylabel("error [mV] / [mA]")
ax[1, 0].legend(loc="upper right")

# ── Zoom of the V2 error around the first (worst) load step ──────────────────
if len(step_t):
    t0 = step_t[0]
    w = (t > t0 - 0.3) & (t < t0 + 0.8)
    ax[1, 1].plot(t[w], err_v[w] * 1e3, "tab:purple", lw=1.0, label="V2 error")
    ax[1, 1].plot(t[w], err_i[w] * 1e3, "tab:green", lw=1.0, label="i_L error", alpha=0.7)
    ax[1, 1].axvline(t0, color="0.6", ls=":", lw=1)
    ax[1, 1].set_title("error zoom @ first load step (%.0f Ω → %.0f Ω)"
                       % (R[step_idx[0]], R[step_idx[0] + 1]))
    ax[1, 1].set_xlabel("t [ms]"); ax[1, 1].set_ylabel("error [mV] / [mA]")
    ax[1, 1].legend(loc="upper right")

fig.suptitle("dab3 background γ-fold vs ideal golden — runtime load (R) steps "
             "[max |V2 err| = %.1f mV, |i_L err| = %.1f mA]"
             % (np.max(np.abs(err_v)) * 1e3, np.max(np.abs(err_i)) * 1e3))
fig.tight_layout()
fig.savefig("dab3_rstep.png", dpi=110)
print("wrote dab3_rstep.png")
print("max |V2 err| = %.3f mV   max |i_L err| = %.3f mA"
      % (np.max(np.abs(err_v)) * 1e3, np.max(np.abs(err_i)) * 1e3))
