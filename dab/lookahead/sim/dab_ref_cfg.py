#!/usr/bin/env python3
# Config-driven DAB golden reference.  Reads the SAME JSON config that
# tools/twin_gen.py used to emit the RTL coefficient package (via
# --emit-ref-config), so the twin and its oracle can never drift: change one
# physical parameter, regenerate both, and they stay consistent by construction.
#
# Physics is the single-step Verlet recurrence (identical to dab_ref.py); only
# the plant parameters come from the config instead of being hard-coded.
#
# Usage: dab_ref_cfg.py <params.json> [--out ref.csv] [--V1 48] [--R 10]
#                                     [--phase-deg 90] [--t-end 30e-3]
import argparse
import json
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("config", help="JSON emitted by twin_gen.py --emit-ref-config")
ap.add_argument("--out", default="ref.csv")
ap.add_argument("--V1", type=float, default=48.0)
ap.add_argument("--R", type=float, default=10.0)
ap.add_argument("--phase-deg", type=float, default=90.0)
ap.add_argument("--t-end", type=float, default=30e-3)
a = ap.parse_args()

cfg = json.load(open(a.config))
assert cfg.get("topology") == "dab", "config is not a DAB twin"
L, Co, k, dt, steps = cfg["L"], cfg["Co"], cfg["k"], cfg["dt"], cfg["steps"]

# Structure param: steps/period sets the switching frequency (not the coeffs).
fsw = 1.0 / (steps * dt)
Tsw = 1.0 / fsw
tshift = (a.phase_deg / 360.0) * Tsw

alpha = dt / (2 * L)
beta = dt / Co
gamma = dt / (a.R * Co)

t = np.arange(0, a.t_end, dt)


def p_square(time, shift=0.0):
    tau = (time - shift) % Tsw
    return 1.0 if tau < (Tsw / 2) else -1.0


i = 0.0
V2 = 0.0
with open(a.out, "w") as f:
    f.write("idx,p1,p2,i_ref,V2_ref\n")
    for idx, tk in enumerate(t):
        p1 = p_square(tk, 0.0)
        p2 = p_square(tk, tshift)
        vL_k = p1 * a.V1 - k * p2 * V2
        i_half = i + alpha * vL_k
        V2_next = V2 + beta * (k * p2 * i_half) - gamma * V2
        vL_k2 = p1 * a.V1 - k * p2 * V2_next
        i_next = i_half + alpha * vL_k2
        f.write("%d,%d,%d,%.10f,%.10f\n" % (idx, int(p1), int(p2), i_next, V2_next))
        i, V2 = i_next, V2_next

print(f"wrote {a.out}: {len(t)} steps  (L={L*1e6:g}u Co={Co*1e6:g}u k={k} dt={dt*1e9:g}n "
      f"steps={steps} → fsw={fsw/1e3:.1f}kHz, γ={gamma:.3e})")
print(f"final i={i:.6f} V2={V2:.6f}")
