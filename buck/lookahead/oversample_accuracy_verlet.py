#!/usr/bin/env python3
"""
oversample_accuracy_verlet.py — does (20 ns step + 4x gate oversample-average)
reach the accuracy of a TRUE 5 ns solver?

Unlike oversample_vs_pipeline.py (which used the exact matrix-exponential map to
ISOLATE gate handling), this script uses the REAL Verlet engine for the moving
schemes, so the *integration-accuracy* axis is genuinely in play.  That is the
axis the supervisor's averaging trick does NOT help with — so this is the honest
test of "can I keep dt=20ns and still match a 5 ns single-cycle solver?".

Schemes, all scored against a fine exact-map TRUTH:
  truth   : exact e^{A.tau} map @ 0.5 ns, gate @ 0.5 ns          (reference)
  fine5   : Verlet @ 5 ns,  gate @ 5 ns                          ("single-cycle 5 ns solver")
  avg20   : Verlet @ 20 ns, gate = mean of 4 x 5 ns samples      (oversample + average)
  naive20 : Verlet @ 20 ns, gate sampled once per 20 ns          (baseline)

Verdict = RMS error of v_C vs truth at the common 20 ns grid.
  avg20 ~ fine5 << naive20  =>  yes, averaging substitutes for fine-stepping.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
try:
    from scipy.linalg import expm
except Exception:
    def expm(M, terms=40):
        S = np.eye(M.shape[0]); T = np.eye(M.shape[0])
        for k in range(1, terms):
            T = T @ M / k; S = S + T
        return S

# ── Buck plant (same params as oversample_vs_pipeline.py) ─────────────────────
L, C, R, Vin = 2.25e-6, 1.125e-6, 4.0, 12.0
A = np.array([[-1.0/(R*C), 1.0/C],
              [-1.0/L,      0.0 ]])
b = np.array([0.0, 1.0/L])
f_LC = 1.0/(2*np.pi*np.sqrt(L*C))

fsw = 1.0e6; Tsw = 1.0/fsw
def gate(t, d):
    return 1.0 if (t % Tsw) < d*Tsw else 0.0

# ── REAL Verlet single-step (exactly tools/twin_gen.py buck_model, damping folded) ─
def verlet(tau):
    hL = tau/(2*L); kC = tau/C; a = hL*kC; kR = tau/(R*C)
    M = np.array([[1-a-kR,            kC   ],
                  [-hL*(2-a)+kR*hL,   1-a  ]])   # = M0 + kR*M1
    u = np.array([a, hL*(2-a)])
    return M, u

def exact(tau):
    Phi = expm(A*tau)
    Gam = np.linalg.solve(A, (Phi - np.eye(2)) @ b)
    return Phi, Gam

# ── Runners: return v_C sampled at the 20 ns grid (common comparison grid) ─────
H = 20e-9; N = 4; h = H/N

def run_verlet(tau, n_os, d, T):
    """Verlet @ tau; gate = mean of n_os sub-samples (n_os=1 -> naive single sample)."""
    M, u = verlet(tau); sub = tau/n_os
    n = int(round(T/tau)); x = np.zeros(2)
    every = int(round(H/tau))                       # stride to the 20 ns grid
    out = []
    for k in range(n):
        t = k*tau
        s = np.mean([gate(t + j*sub, d) for j in range(n_os)]) if n_os > 1 else gate(t, d)
        x = M @ x + u*(s*Vin)
        if (k+1) % every == 0:
            out.append(x[0])
    return np.array(out)

def run_truth(d, T, tau=0.5e-9):
    Phi, Gam = exact(tau)
    n = int(round(T/tau)); x = np.zeros(2)
    every = int(round(H/tau)); out = []
    for k in range(n):
        t = k*tau
        x = Phi @ x + Gam*(gate(t, d)*Vin)
        if (k+1) % every == 0:
            out.append(x[0])
    return np.array(out)

# ══════════════════════════════════════════════════════════════════════════════
T = 60e-6; d = 0.37                                 # 0.37 -> PWM edges OFF the 20 ns grid
                                                    # (60 us >> 9 us ring time-const, so the
                                                    #  Q~2.8 startup transient is fully settled)
print(f"Buck: f_LC={f_LC/1e3:.1f}kHz  dt*f_LC@20ns={20e-9*f_LC:.2e} (integration over-resolved if <<1)")
print(f"PWM: {fsw/1e3:g}kHz duty={d}; run {T*1e6:g}us; scored at the 20ns grid vs 0.5ns truth\n")

vt    = run_truth(d, T)
vf5   = run_verlet(h,  1, d, T)        # true 5 ns solver
vavg  = run_verlet(H,  N, d, T)        # 20 ns + 4x oversample-average
vnv   = run_verlet(H,  1, d, T)        # 20 ns naive

def rms(a, ref): return np.sqrt(np.mean((a-ref)**2))
tgrid = (np.arange(len(vt))+1)*H
# Q~2.8 ring time-const ~9 us; measure the LAST 4 us (>5 time-consts settled)
mask_ss = tgrid >= 56e-6
print("RMS error of v_C vs truth [mV]:        full run     steady-state tail")
for nm, v in [("fine5  (true 5 ns solver)", vf5),
              ("avg20  (20 ns + 4x avg)  ", vavg),
              ("naive20(20 ns, 1 sample) ", vnv)]:
    print(f"  {nm}:  {rms(v, vt)*1e3:8.3f}      {rms(v[mask_ss], vt[mask_ss])*1e3:8.3f}")
print(f"\n  |avg20 - fine5| RMS = {rms(vavg, vf5)*1e3:.4f} mV   (how far averaging is from the 5 ns solver)")
print(f"  avg20 reaches {rms(vnv,vt)/rms(vavg,vt):.1f}x lower error than naive20")

# ── decompose steady-state error: DC offset vs AC (ripple-shape) error ────────
print("\nSteady-state decomposition (vs truth):   mean[V]   DC-offset[mV]  AC/ripple-RMS[mV]  pkpk[mV]")
def show(nm, v):
    dc = (v[mask_ss].mean() - vt[mask_ss].mean())*1e3
    ac = np.sqrt(np.mean(((v[mask_ss]-v[mask_ss].mean()) - (vt[mask_ss]-vt[mask_ss].mean()))**2))*1e3
    print(f"  {nm}:  {v[mask_ss].mean():7.4f}   {dc:9.3f}    {ac:9.3f}      {(v[mask_ss].max()-v[mask_ss].min())*1e3:7.2f}")
show("truth  ", vt); show("fine5  ", vf5); show("avg20  ", vavg); show("naive20", vnv)

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
z = (tgrid >= 56.0e-6) & (tgrid <= 59.0e-6)         # a few PWM cycles, fully settled
ax[0].plot(tgrid[z]*1e6, vf5[z],  'C0--',lw=1.3, label='fine5: true 5 ns solver')
ax[0].plot(tgrid[z]*1e6, vavg[z], 'C2o', ms=4,   label='avg20: 20 ns + 4x avg')
ax[0].plot(tgrid[z]*1e6, vnv[z],  'C3s', ms=3,   label='naive20: 20 ns')
ax[0].set(xlabel='t [us]', ylabel='v_C [V]', title='Settled ripple (Verlet integration in play)')
ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)

ax[1].plot(tgrid*1e6, (vnv -vt)*1e3, 'C3s-', ms=2, lw=.8, label='naive20 - truth')
ax[1].plot(tgrid*1e6, (vavg-vt)*1e3, 'C2o-', ms=2, lw=.8, label='avg20 - truth')
ax[1].plot(tgrid*1e6, (vf5 -vt)*1e3, 'C0--', lw=1.1,      label='fine5 - truth')
ax[1].set(xlabel='t [us]', ylabel='v_C error vs truth [mV]',
          title='avg20 tracks fine5; both << naive20'); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
plt.tight_layout(); plt.savefig("oversample_accuracy_verlet.png", dpi=130)
print("\nwrote oversample_accuracy_verlet.png")
