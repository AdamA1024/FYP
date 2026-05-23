#!/usr/bin/env python3
# Golden reference for the RUNTIME-R (load-step) test of the optimized engine
# (dab3.sv).  Identical single-step Verlet physics to dab_ref.py, but the load
# resistance R — and therefore γ = Δt/(R·Co) — changes during the run.
#
# This is the "ideal" reference: γ takes effect INSTANTLY at the step.  dab3.sv
# instead folds γ into its coefficients in a background datapath (a few clocks of
# latency), so comparing the two shows exactly the cost of moving the γ-update
# off the recurrence loop.  The schedule below is emitted alongside the trajectory
# so the testbench drives the engine's gamma_in with the matching Q4.28 value.
#
# CSV columns: idx, p1, p2, gamma_q428, i_ref, V2_ref
import numpy as np

V1 = 48.0
k  = 1.75
L  = 20e-6
Co = 470e-6

fsw = 100e3
Tsw = 1.0 / fsw
phi = 90.0 * np.pi / 180.0
tshift = (phi / (2 * np.pi)) * Tsw

dt = 50e-9
t_end = 30e-3
t = np.arange(0, t_end, dt)

alpha = dt / (2 * L)
beta  = dt / Co

Q428 = 1 << 28


def R_schedule(time):
    """Load resistance vs time — two abrupt load steps (worst case for tracking)."""
    if time < 10e-3:
        return 10.0          # nominal load
    elif time < 20e-3:
        return 5.0           # 2x heavier load  (γ doubles)
    else:
        return 20.0          # light load       (γ quarters vs heavy)


def gamma_q428(R):
    g = dt / (R * Co)
    return int(round(g * Q428))


def p_square(time, shift=0.0):
    tau = (time - shift) % Tsw
    return 1.0 if tau < (Tsw / 2) else -1.0


i = 0.0
V2 = 0.0
steps_by_R = {}
with open("ref_rstep.csv", "w") as f:
    f.write("idx,p1,p2,gamma_q428,i_ref,V2_ref\n")
    for idx, tk in enumerate(t):
        R = R_schedule(tk)
        gamma = dt / (R * Co)              # ideal, instantaneous γ
        gq = gamma_q428(R)
        steps_by_R[R] = steps_by_R.get(R, 0) + 1

        p1 = p_square(tk, 0.0)
        p2 = p_square(tk, tshift)

        vL_k = p1 * V1 - k * p2 * V2
        i_half = i + alpha * vL_k
        V2_next = V2 + beta * (k * p2 * i_half) - gamma * V2
        vL_k2 = p1 * V1 - k * p2 * V2_next
        i_next = i_half + alpha * vL_k2

        f.write("%d,%d,%d,%d,%.10f,%.10f\n" % (idx, int(p1), int(p2), gq, i_next, V2_next))
        i, V2 = i_next, V2_next

print("wrote ref_rstep.csv: %d steps" % len(t))
print("R schedule (Ω → γ_Q4.28):")
for R in sorted(steps_by_R):
    print("  R=%5.1f  γ=%.3e  q428=%d (0x%X)  for %d steps"
          % (R, dt / (R * Co), gamma_q428(R), gamma_q428(R), steps_by_R[R]))
print("final i=%.6f V2=%.6f" % (i, V2))
