#!/usr/bin/env python3
# Golden reference for the *pipelined* DAB look-ahead engine.
#
# IMPORTANT: the engine still advances ONE algorithmic dt per clock cycle.
# The 2-step Parhi look-ahead transformation only rewrites the recursion as
#     x[n+1] = M²·x[n-1] + M·b·v[n-1] + b·v[n]
# so the feedback loop tolerates 2 cycles of pipeline registers without
# changing what x[n] means.
#
# The only behavioural difference vs. the old single-step engine is a
# *1-cycle pipeline latency*: state_reg at the end of clock k holds x[k],
# whereas the old engine had x[k+1].  tb_dab.cpp accounts for this by
# comparing engine output at iteration k to ref row (k-1).
#
# So this file is identical to the original single-step reference.
import numpy as np

V1 = 48.0
k = 1.75
L = 20e-6
Co = 470e-6
Rload = 10.0

fsw = 100e3
Tsw = 1.0 / fsw
phi = 90.0 * np.pi / 180.0
tshift = (phi / (2 * np.pi)) * Tsw

dt = 50e-9
t_end = 30e-3
t = np.arange(0, t_end, dt)

alpha = dt / (2 * L)
beta = dt / Co
gamma = dt / (Rload * Co)


def p_square(time, shift=0.0):
    tau = (time - shift) % Tsw
    return 1.0 if tau < (Tsw / 2) else -1.0


i = 0.0
V2 = 0.0
with open("ref.csv", "w") as f:
    f.write("idx,p1,p2,i_ref,V2_ref\n")
    for idx, tk in enumerate(t):
        p1 = p_square(tk, 0.0)
        p2 = p_square(tk, tshift)

        vL_k = p1 * V1 - k * p2 * V2
        i_half = i + alpha * vL_k
        V2_next = V2 + beta * (k * p2 * i_half) - gamma * V2
        vL_k2 = p1 * V1 - k * p2 * V2_next
        i_next = i_half + alpha * vL_k2

        f.write("%d,%d,%d,%.10f,%.10f\n" % (idx, int(p1), int(p2), i_next, V2_next))
        i, V2 = i_next, V2_next

print("wrote ref.csv: %d steps, V1=%.1f" % (len(t), V1))
print("final i=%.6f V2=%.6f" % (i, V2))
