#!/usr/bin/env python3
"""
oversample_vs_pipeline.py — numerical investigation of the supervisor's question:

    "Is oversampling the gate N times within a coarse step H, averaging it, and
     driving the solver with the average, EQUIVALENT to a pipelined solver
     running at the fine step h = H/N — without inserting pipeline registers?"

Test plant: Buck (CCM).  Chosen because the switch gates Vin into the LC filter
purely on the INPUT side — the state matrix is switch-independent — which is the
clean, provable case (matches tools/twin_gen.py buck_model: M is switch-free, the
gate only scales the input vector û).  See the companion write-up for the proof;
this script supplies the evidence.

Method (integrator is EXACT here on purpose):
    For a step of length tau with a CONSTANT gate s, the exact update is
        x+ = Phi(tau) x + Gamma(tau) * s * Vin ,
        Phi   = expm(A*tau),
        Gamma = A^{-1}(Phi - I) b .
    Using the EXACT map removes integrator-order error, so the ONLY thing being
    compared is how each scheme handles the GATE.  Differences seen below are
    therefore purely the oversample-vs-pipeline effect, nothing else.

Four schemes:
    truth     : exact map at a very fine step (gate held at h_truth)   -> reference
    pipeline  : exact map at fine h = H/N, gate sampled every h        -> "5 ns pipeline"
    averaged  : exact map at coarse H, input = mean of the N sub-gates -> supervisor's idea
    naive     : exact map at coarse H, gate sampled once per H         -> baseline

Key identity (proved in the write-up, checked numerically in part 3):
    With the EXACT discretisation, sum_j Phi(h)^{N-1-j} Gamma(h) = Gamma(N h) = Gamma(H).
    Hence  pipeline_input - averaged_input
         = Vin * sum_j Phi(h)^{N-1-j} Gamma(h) (s_j - s_bar)
    whose leading term cancels (sum_j (s_j - s_bar) = 0), leaving an O(H^2),
    edge-localised residual.  => first-order-equivalent; differ only at 2nd order.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from scipy.linalg import expm
except Exception:                                   # tiny fallback if scipy absent
    def expm(M, terms=30):
        S = np.eye(M.shape[0]); T = np.eye(M.shape[0])
        for k in range(1, terms):
            T = T @ M / k; S = S + T
        return S

# ── Buck plant (CCM): x = [vC, iL];  switch s in {0,1} gates Vin on the INPUT ──
L   = 2.25e-6          # H   (matches buck/lookahead twin_gen config)
C   = 1.125e-6         # F
R   = 4.0             # ohm load
Vin = 12.0            # V
A = np.array([[-1.0/(R*C), 1.0/C],
              [-1.0/L,      0.0 ]])
b = np.array([0.0, 1.0/L])         # injected input = s * Vin  (switch-independent A)

# ── PWM gate: 1 MHz, duty d ──────────────────────────────────────────────────
fsw = 1.0e6
Tsw = 1.0/fsw
def gate(t, d):
    return 1.0 if (t % Tsw) < d*Tsw else 0.0

def disc(tau):
    """Exact single-step map for constant gate over length tau."""
    Phi = expm(A*tau)
    Gamma = np.linalg.solve(A, (Phi - np.eye(2)) @ b)
    return Phi, Gamma

# ── Simulators (all return time, vC, iL arrays at their own step grid) ─────────
def run_pipeline(h, d, T, x0=None):
    Phi, Gam = disc(h)
    n = int(round(T/h)); x = np.zeros(2) if x0 is None else x0.copy()
    vc = np.empty(n); il = np.empty(n); ts = np.empty(n)
    for k in range(n):
        t = k*h
        x = Phi @ x + Gam * (gate(t, d) * Vin)
        ts[k], vc[k], il[k] = t+h, x[0], x[1]
    return ts, vc, il, x

def run_averaged(H, N, d, T, x0=None):
    h = H/N
    Phi, Gam = disc(H)                     # ONE coarse exact step ...
    n = int(round(T/H)); x = np.zeros(2) if x0 is None else x0.copy()
    vc = np.empty(n); il = np.empty(n); ts = np.empty(n)
    for k in range(n):
        t = k*H
        sbar = np.mean([gate(t + j*h, d) for j in range(N)])   # ... fed the AVERAGE gate
        x = Phi @ x + Gam * (sbar * Vin)
        ts[k], vc[k], il[k] = t+H, x[0], x[1]
    return ts, vc, il, x

def run_naive(H, d, T, x0=None):
    Phi, Gam = disc(H)
    n = int(round(T/H)); x = np.zeros(2) if x0 is None else x0.copy()
    vc = np.empty(n); il = np.empty(n); ts = np.empty(n)
    for k in range(n):
        t = k*H
        x = Phi @ x + Gam * (gate(t, d) * Vin)   # single sample at window start
        ts[k], vc[k], il[k] = t+H, x[0], x[1]
    return ts, vc, il, x

def steady_mean_vc(run_fn, d, settle=30e-6, meas=5e-6):
    """Mean vC over the last `meas` once settled — independent of ripple shape."""
    ts, vc, il, _ = run_fn(d, settle+meas)
    return vc[ts >= settle].mean()

# ══════════════════════════════════════════════════════════════════════════════
H = 20e-9        # coarse step  (supervisor's example)
N = 4            # oversample factor  -> fine h = 5 ns
h = H/N
h_truth = 1e-9   # reference step (20x finer than H)

print(f"Buck: L={L*1e6:g}uH C={C*1e6:g}uF R={R}ohm Vin={Vin}V  f_LC={1/(2*np.pi*np.sqrt(L*C))/1e3:.1f}kHz")
print(f"H={H*1e9:g}ns  N={N}  h=H/N={h*1e9:g}ns  truth={h_truth*1e9:g}ns  f_sw={fsw/1e3:g}kHz\n")

# ── Part 1: trajectory at one duty ────────────────────────────────────────────
d0 = 0.37
T_traj = 4*Tsw
tt, vt, it, _ = run_pipeline(h_truth, d0, T_traj)
tp, vp, ip, _ = run_pipeline(h,       d0, T_traj)
ta, va, ia, _ = run_averaged(H, N,    d0, T_traj)
tn, vn, in_, _ = run_naive(H,         d0, T_traj)

# ── Part 2: steady-state mean Vout vs commanded duty (the headline) ───────────
duties = np.linspace(0.08, 0.92, 43)
m_truth = np.array([steady_mean_vc(lambda d, T: run_pipeline(h_truth, d, T), d) for d in duties])
m_pipe  = np.array([steady_mean_vc(lambda d, T: run_pipeline(h,       d, T), d) for d in duties])
m_avg   = np.array([steady_mean_vc(lambda d, T: run_averaged(H, N,    d, T), d) for d in duties])
m_naive = np.array([steady_mean_vc(lambda d, T: run_naive(H,          d, T), d) for d in duties])

print("Steady-state mean Vout error vs truth (RMS over duty sweep):")
print(f"  pipeline (h={h*1e9:g}ns): {np.sqrt(np.mean((m_pipe -m_truth)**2))*1e3:7.3f} mV")
print(f"  averaged (H={H*1e9:g}ns): {np.sqrt(np.mean((m_avg  -m_truth)**2))*1e3:7.3f} mV   <-- supervisor")
print(f"  naive    (H={H*1e9:g}ns): {np.sqrt(np.mean((m_naive-m_truth)**2))*1e3:7.3f} mV")
print(f"  |averaged - pipeline| max over sweep: {np.max(np.abs(m_avg-m_pipe))*1e3:7.4f} mV\n")

# ── Part 3: the central identity — pipeline_input - averaged_input is O(H^2) ──
# Direct algebraic check (no simulation noise).  For ONE window of length H with
# sub-gates s = [s_0..s_{N-1}] (s_bar = mean):
#     pipeline injects  Vin * sum_j Phi(h)^{N-1-j} Gamma(h) s_j
#     averaged injects  Vin * Gamma(H) s_bar      ( = Vin*sum_j Phi(h)^{N-1-j}Gamma(h) s_bar,
#                                                    by the exact-map semigroup identity )
# => residual = Vin * || sum_j Phi(h)^{N-1-j} Gamma(h) (s_j - s_bar) ||.
def input_residual(H, s):
    N = len(s); h = H/N
    Phi_h, Gam_h = disc(h)
    sbar = np.mean(s)
    acc = np.zeros(2)
    P = np.eye(2)                         # Phi(h)^0 for the LAST sub-step (j=N-1)
    for j in range(N-1, -1, -1):          # j = N-1 .. 0, accumulating Phi(h)^{N-1-j}
        acc += P @ Gam_h * (s[j] - sbar)
        P = Phi_h @ P
    return Vin*np.linalg.norm(acc), Vin*np.linalg.norm(Gam_h*sbar)  # residual, ref scale

Hs = np.array([5e-9, 10e-9, 20e-9, 40e-9, 80e-9, 160e-9])
s_edge = [1.,1.,0.,0.]                    # one switching edge mid-window (worst case)
s_flat = [1.,1.,1.,1.]                    # no edge -> must be exactly 0
res_edge = np.array([input_residual(Hc, s_edge)[0] for Hc in Hs])
res_flat = np.array([input_residual(Hc, s_flat)[0] for Hc in Hs])

def slope(x, y):
    m = y > 0
    return np.polyfit(np.log(x[m]), np.log(y[m]), 1)[0] if m.sum() > 1 else float('nan')
print("Central identity (pipeline_input - averaged_input):")
print(f"  no edge in window  -> residual = {res_flat.max():.2e}  (exactly 0: schemes IDENTICAL)")
print(f"  one edge in window -> residual log-log slope vs H = {slope(Hs, res_edge):.2f}  (expect 2 => O(H^2))")
print(f"  at H=20ns the edge residual is {input_residual(20e-9, s_edge)[0]*1e3:.3f} mV"
      f"  (vs single-step input scale {input_residual(20e-9,s_edge)[1]*1e3:.0f} mV)\n")

# ── Plots ─────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(1, 2, figsize=(11.5, 4.6))
# zoom on one switching period mid-ramp so the four lines are distinguishable
z0, z1 = 2.0e-6, 3.0e-6
zt = (tt >= z0) & (tt <= z1); zp = (tp >= z0) & (tp <= z1)
za = (ta >= z0) & (ta <= z1); zn = (tn >= z0) & (tn <= z1)
ax[0].plot(tt[zt]*1e6, vt[zt], 'k-',  lw=1.6, label='truth (1 ns)')
ax[0].plot(tp[zp]*1e6, vp[zp], 'C0--',lw=1.2, label=f'pipeline {h*1e9:g} ns')
ax[0].plot(ta[za]*1e6, va[za], 'C2o-',ms=3,   label=f'averaged {H*1e9:g} ns, N={N}')
ax[0].plot(tn[zn]*1e6, vn[zn], 'C3s:',ms=3,   label=f'naive {H*1e9:g} ns')
ax[0].set(xlabel='t [us]', ylabel='vC [V]',
          title=f'Trajectory, one switching period (duty={d0})'); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)

ax[1].axhline(0, color='k', lw=1.0)
ax[1].plot(duties, (m_pipe -m_truth)*1e3, 'C0--',lw=1.4,label=f'pipeline {h*1e9:g} ns')
ax[1].plot(duties, (m_avg  -m_truth)*1e3, 'C2o', ms=4,  label=f'averaged {H*1e9:g} ns, N={N}')
ax[1].plot(duties, (m_naive-m_truth)*1e3, 'C3s', ms=3,  label=f'naive {H*1e9:g} ns')
ax[1].set(xlabel='commanded duty', ylabel='steady-state mean Vout error vs truth [mV]',
          title='Steady-state duty fidelity'); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)

plt.tight_layout()
out = "oversample_vs_pipeline.png"
plt.savefig(out, dpi=130)
print(f"wrote {out}")
