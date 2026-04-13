#!/usr/bin/env python3
"""
Buck converter accuracy study: effect of dt and fsw on Verlet integration error.

Circuit:  L=10 µH, C=100 µF, R=5 Ω, Vin=12 V, D=0.5
Integrator: leapfrog (Velocity Verlet) — floating-point mirror of the RTL.

Method
------
1. Compute exact periodic steady-state ICs analytically via matrix exponential
   so both solvers start from identical, settled conditions.
2. Run N_MEASURE switching cycles with:
     (a) Verlet (variable dt)
     (b) Piecewise-exact scipy DOP853 reference (rtol=1e-10, atol=1e-12)
3. Interpolate reference onto Verlet time-grid; compute RMS % error.

Studies
-------
  Study 1 – dt sweep  : dt ∈ [1 ns … 1000 ns]  at fixed fsw = 500 kHz
  Study 2 – fsw sweep : fsw ∈ [25 kHz … 5 MHz] at fixed dt  = 20 ns

Output files
------------
  accuracy_vs_dt.png      – log-log error vs dt, with O(dt²) reference line
  accuracy_vs_fsw.png     – error vs fsw and vs steps-per-cycle
  waveform_comparison.png – overlay of v_out / i_L at four dt values
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import expm

# ── Physical constants ─────────────────────────────────────────────────────
L    = 10e-6    # H
C    = 100e-6   # F
R    = 5.0      # Ω
G    = 1.0 / R  # S  (load conductance)
Vin  = 12.0     # V
DUTY = 0.5

N_MEASURE = 20  # switching cycles used for RMS error


# ══════════════════════════════════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════════════════════════════════

def ss_ic(Tsw, duty=DUTY):
    """
    Exact periodic-steady-state initial conditions [i₀, v₀] at the start
    of the ON-phase, computed via matrix exponential.

    Derivation
    ----------
    During ON  (duration ton):  x' = A·x + B_on,   B_on = [Vin/L, 0]
    During OFF (duration toff): x' = A·x            (B_off = 0)

    Transition over one full cycle:
        x(Tsw) = Φ·x(0) + Γ
    where Φ = Φ_off·Φ_on and Γ = Φ_off·(Φ_on − I)·A⁻¹·B_on.

    Steady-state: x(0) = (I − Φ)⁻¹·Γ
    """
    A    = np.array([[0.0, -1.0 / L],
                     [1.0 / C, -G / C]])
    ton  = duty * Tsw
    toff = (1.0 - duty) * Tsw
    Phi_on  = expm(A * ton)
    Phi_off = expm(A * toff)
    Ainv    = np.linalg.inv(A)
    B_on    = np.array([Vin / L, 0.0])
    Gamma   = Phi_off @ ((Phi_on - np.eye(2)) @ (Ainv @ B_on))
    Phi     = Phi_off @ Phi_on
    return np.linalg.solve(np.eye(2) - Phi, Gamma)


def reference_sim(Tsw, x0, n_cycles=N_MEASURE, duty=DUTY):
    """
    Piecewise-exact ODE integration (DOP853, rtol=1e-10).
    Integrates each on/off half-period separately to avoid discontinuous
    forcing at the switching instant.
    Returns: t [s], i_L [A], v_out [V]
    """
    ton, toff = duty * Tsw, (1.0 - duty) * Tsw
    t_segs, i_segs, v_segs = [], [], []
    y, tc = x0.copy(), 0.0

    for _ in range(n_cycles):
        for vin_eff, dur in [(Vin, ton), (0.0, toff)]:
            def ode(t, y, ve=vin_eff):
                return [(ve - y[1]) / L,
                        (y[0] - G * y[1]) / C]
            sol = solve_ivp(ode, [tc, tc + dur], y,
                            method='DOP853', rtol=1e-10, atol=1e-12,
                            dense_output=True, max_step=dur / 200)
            # Exclude last point — next segment includes it as its first.
            t_segs.append(sol.t[:-1])
            i_segs.append(sol.y[0, :-1])
            v_segs.append(sol.y[1, :-1])
            y   = sol.y[:, -1].copy()
            tc += dur

    # Final sample
    t_segs.append([tc]); i_segs.append([y[0]]); v_segs.append([y[1]])
    return (np.concatenate(t_segs),
            np.concatenate(i_segs),
            np.concatenate(v_segs))


def verlet_sim(dt, Tsw, x0, n_cycles=N_MEASURE, duty=DUTY):
    """
    Leapfrog (Velocity Verlet) integrator — exact floating-point mirror
    of the RTL modules ik_half / vk_new / ik_new.

        i_{k+1/2} = i_k  + (dt/2L) * (s·Vin − v_k)
        v_{k+1}   = v_k  + (dt/C)  * (i_{k+1/2} − G·v_k)
        i_{k+1}   = i_{k+1/2} + (dt/2L) * (s·Vin − v_{k+1})

    Switch state s is sampled at the START of each step (matches RTL).
    Integer modular arithmetic avoids FP accumulation error on the phase.
    """
    kL2 = dt / (2.0 * L)
    kC  = dt / C
    spc = int(round(Tsw / dt))          # steps per cycle
    on_stp = int(round(duty * spc))     # steps with switch ON per cycle

    n_steps = n_cycles * spc
    t  = np.arange(n_steps + 1, dtype=float) * dt
    iv = np.empty(n_steps + 1)
    vv = np.empty(n_steps + 1)

    ik, vk = float(x0[0]), float(x0[1])
    for k in range(n_steps):
        iv[k], vv[k] = ik, vk
        s      = 1 if (k % spc) < on_stp else 0
        i_half = ik + kL2 * (s * Vin - vk)
        v_new  = vk + kC  * (i_half - G * vk)
        i_new  = i_half + kL2 * (s * Vin - v_new)
        ik, vk = i_new, v_new

    iv[-1], vv[-1] = ik, vk
    return t, iv, vv


def rms_pct(t_v, iv_v, vv_v, t_r, iv_r, vv_r):
    """
    RMS percentage error of Verlet vs reference, interpolating the
    high-density reference onto the coarser Verlet time grid.
    """
    vr = np.interp(t_v, t_r, vv_r)
    ir = np.interp(t_v, t_r, iv_r)
    ve = np.sqrt(np.mean((vv_v - vr)**2)) / np.sqrt(np.mean(vr**2)) * 100.0
    ie = np.sqrt(np.mean((iv_v - ir)**2)) / np.sqrt(np.mean(ir**2)) * 100.0
    return ve, ie


# ══════════════════════════════════════════════════════════════════════════
# Study 1 – dt sweep  (fsw = 500 kHz fixed)
# ══════════════════════════════════════════════════════════════════════════
FSW_BASE = 500e3
TSW_BASE = 1.0 / FSW_BASE   # 2 µs

# dt values (ns) that divide Tsw = 2000 ns exactly
DT_NS   = np.array([1, 2, 4, 5, 8, 10, 20, 25, 40, 50, 100, 200, 400, 500, 1000])
DT_VALS = DT_NS * 1e-9

x0_base = ss_ic(TSW_BASE)
print(f"SS IC @ fsw=500 kHz:  i₀ = {x0_base[0]:.5f} A,  v₀ = {x0_base[1]:.5f} V")
print(f"\n{'dt (ns)':>10}  {'spc':>6}  {'v_out err %':>12}  {'i_L err %':>10}")
print("-" * 45)

t_r0, i_r0, v_r0 = reference_sim(TSW_BASE, x0_base)
v_err_dt, i_err_dt = [], []
for dt in DT_VALS:
    spc = int(round(TSW_BASE / dt))
    t_v, iv_v, vv_v = verlet_sim(dt, TSW_BASE, x0_base)
    ve, ie = rms_pct(t_v, iv_v, vv_v, t_r0, i_r0, v_r0)
    v_err_dt.append(ve); i_err_dt.append(ie)
    print(f"{dt*1e9:>10.0f}  {spc:>6}  {ve:>12.5f}  {ie:>10.5f}")

v_err_dt = np.array(v_err_dt)
i_err_dt = np.array(i_err_dt)


# ══════════════════════════════════════════════════════════════════════════
# Study 2 – fsw sweep  (dt = 20 ns fixed)
# ══════════════════════════════════════════════════════════════════════════
DT_FPGA  = 20e-9    # FPGA clock period
FSW_KHZ  = np.array([25, 50, 100, 200, 250, 500, 1000, 2000, 5000])
FSW_VALS = FSW_KHZ * 1e3
TSW_VALS = 1.0 / FSW_VALS
SPC_VALS = np.round(TSW_VALS / DT_FPGA).astype(int)

# Keep only configurations with at least 4 steps/cycle
valid    = SPC_VALS >= 4
FSW_KHZ  = FSW_KHZ[valid]; FSW_VALS = FSW_VALS[valid]
TSW_VALS = TSW_VALS[valid]; SPC_VALS = SPC_VALS[valid]

print(f"\n{'fsw (kHz)':>12}  {'spc':>6}  {'v_out err %':>12}  {'i_L err %':>10}")
print("-" * 45)

v_err_fsw, i_err_fsw = [], []
for fsw, Tsw, spc in zip(FSW_VALS, TSW_VALS, SPC_VALS):
    x0 = ss_ic(Tsw)
    t_r, i_r, v_r   = reference_sim(Tsw, x0)
    t_v, iv_v, vv_v  = verlet_sim(DT_FPGA, Tsw, x0)
    ve, ie = rms_pct(t_v, iv_v, vv_v, t_r, i_r, v_r)
    v_err_fsw.append(ve); i_err_fsw.append(ie)
    print(f"{fsw/1e3:>12.0f}  {spc:>6}  {ve:>12.5f}  {ie:>10.5f}")

v_err_fsw = np.array(v_err_fsw)
i_err_fsw = np.array(i_err_fsw)


# ══════════════════════════════════════════════════════════════════════════
# Study 3 – 2D error surface: v_out RMS % error across (dt, fsw)
#
# For each cell we check two things before running:
#   (a) Integer spc: Tsw must be an integer multiple of dt.
#       Non-integer → impossible to implement; cell left NaN (shown grey).
#   (b) Duty aliasing: with integer spc, round(D·spc) must give the right
#       effective duty.  If |D_eff − D| > 0.5 %, cell is flagged as aliased
#       (shown in orange with hatching) — the error then reflects the DC
#       offset, not integration quality.
# ══════════════════════════════════════════════════════════════════════════
HM_DT_NS   = np.array([1, 2, 5, 10, 20, 50, 100, 200, 500])
HM_FSW_KHZ = np.array([25, 50, 100, 200, 500, 1000, 2000])
HM_DT      = HM_DT_NS  * 1e-9
HM_FSW     = HM_FSW_KHZ * 1e3
HM_TSW     = 1.0 / HM_FSW

nd, nf = len(HM_DT_NS), len(HM_FSW_KHZ)
hm_v_err   = np.full((nf, nd), np.nan)
hm_aliased = np.zeros((nf, nd), dtype=bool)   # duty-cycle aliasing
hm_invalid = np.zeros((nf, nd), dtype=bool)   # non-integer spc

print("\nComputing 2D error surface …")
for j, (fsw, Tsw) in enumerate(zip(HM_FSW, HM_TSW)):
    Tsw_ns = int(round(Tsw * 1e9))
    x0     = ss_ic(Tsw)
    t_r, i_r, v_r = reference_sim(Tsw, x0)
    for i, (dt, dt_ns) in enumerate(zip(HM_DT, HM_DT_NS)):
        if Tsw_ns % dt_ns != 0:
            hm_invalid[j, i] = True
            continue
        spc    = Tsw_ns // dt_ns
        on_stp = int(round(DUTY * spc))
        if abs(on_stp / spc - DUTY) > 0.005:
            hm_aliased[j, i] = True
        t_v, iv_v, vv_v = verlet_sim(dt, Tsw, x0)
        ve, _ = rms_pct(t_v, iv_v, vv_v, t_r, i_r, v_r)
        hm_v_err[j, i] = ve
        tag = ' [alias]' if hm_aliased[j, i] else ''
        print(f"  dt={dt_ns:4d} ns  fsw={fsw/1e3:6.0f} kHz  "
              f"spc={spc:5d}  v_err={ve:.5f}%{tag}")


# ══════════════════════════════════════════════════════════════════════════
# Plotting
# ══════════════════════════════════════════════════════════════════════════
plt.rcParams.update({
    'font.size': 11,
    'axes.grid': True,
    'grid.alpha': 0.35,
    'grid.linestyle': '--',
})

PARAM_STR = (f"L={L*1e6:.0f} µH,  C={C*1e6:.0f} µF,  "
             f"R={R} Ω,  Vin={Vin} V,  D={DUTY}")

# ── Figure 1: accuracy vs dt ───────────────────────────────────────────────
fig1, axs = plt.subplots(1, 2, figsize=(13, 5.5))
fig1.suptitle(
    "Effect of Time-Step  dt  on Verlet Integration Accuracy\n"
    f"fsw = {FSW_BASE/1e3:.0f} kHz  ({PARAM_STR})", fontsize=12)

for ax, errs, sig, col in [
    (axs[0], v_err_dt, r'$v_\mathrm{out}$', 'tab:blue'),
    (axs[1], i_err_dt, r'$i_L$',            'tab:red'),
]:
    ax.loglog(DT_NS, errs, 'o-', color=col, lw=2.2, ms=7, label='Verlet RMS error')

    # O(dt²) reference line anchored at a mid-range point
    anchor = 4
    slope2 = errs[anchor] * (DT_NS / DT_NS[anchor]) ** 2
    ax.loglog(DT_NS, slope2, '--k', lw=1.4, label=r'$O(dt^2)$ slope', zorder=0)

    ax.set_xlabel('dt (ns)')
    ax.set_ylabel('RMS error (%)')
    ax.set_title(f'{sig} RMS error vs dt')
    ax.legend(fontsize=9)

fig1.tight_layout()
fig1.savefig('accuracy_vs_dt.png', dpi=150, bbox_inches='tight')
print("\nSaved: accuracy_vs_dt.png")


# ── Figure 2: accuracy vs fsw ──────────────────────────────────────────────
from matplotlib.colors import LogNorm, BoundaryNorm
from matplotlib.patches import Patch
import matplotlib.ticker as ticker

fig2, axs = plt.subplots(1, 1, figsize=(15, 6))
fig2.suptitle(
    "Verlet Integrator Accuracy — fsw Sweep & Full (dt, fsw) Error Map\n"
    f"({PARAM_STR})", fontsize=12)

# ── Left: 1-D error vs fsw (dt = 20 ns fixed) ─────────────────────────────
ax.semilogy(FSW_KHZ, v_err_fsw, 'o-b', lw=2.2, ms=7, label=r'$v_\mathrm{out}$')
ax.semilogy(FSW_KHZ, i_err_fsw, 's--r', lw=2.2, ms=7, label=r'$i_L$')
ax.axvline(500, color='tab:green', ls=':', lw=2.0, label='Proposed fsw = 500 kHz')
ax.set_xlabel('fsw (kHz)')
ax.set_ylabel('RMS error (%)')
ax.set_title(f'RMS error vs fsw  [dt = {DT_FPGA*1e9:.0f} ns fixed]')
ax.legend(fontsize=9)


# ── Figure 3: waveform comparison at selected dt values ───────────────────
DT_SHOW   = [5e-9, 20e-9, 100e-9, 500e-9]
N_SHOW    = 4
x0_show   = ss_ic(TSW_BASE)
t_rw, i_rw, v_rw = reference_sim(TSW_BASE, x0_show, n_cycles=N_SHOW)

fig3, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
fig3.suptitle(
    f"Waveform Fidelity at Different Time Steps\n"
    f"fsw = {FSW_BASE/1e3:.0f} kHz,  {N_SHOW} steady-state cycles  ({PARAM_STR})",
    fontsize=12)

axs[0].plot(t_rw * 1e6, v_rw, 'k-', lw=2.5, label='Reference (DOP853)', zorder=10)
axs[1].plot(t_rw * 1e6, i_rw, 'k-', lw=2.5, label='Reference (DOP853)', zorder=10)

COLORS = ['tab:blue', 'tab:green', 'tab:orange', 'tab:red']
for dt_s, col in zip(DT_SHOW, COLORS):
    t_vw, iv_vw, vv_vw = verlet_sim(dt_s, TSW_BASE, x0_show, n_cycles=N_SHOW)
    lab = f'Verlet  dt = {dt_s*1e9:.0f} ns'
    axs[0].plot(t_vw * 1e6, vv_vw, '-', color=col, lw=1.4, label=lab, alpha=0.9)
    axs[1].plot(t_vw * 1e6, iv_vw, '-', color=col, lw=1.4, label=lab, alpha=0.9)

axs[0].set_ylabel(r'$v_\mathrm{out}$ (V)')
axs[0].legend(fontsize=9, ncol=3)
axs[1].set_ylabel(r'$i_L$ (A)')
axs[1].set_xlabel('Time (µs)')
axs[1].legend(fontsize=9, ncol=3)

fig3.tight_layout()
fig3.savefig('waveform_comparison.png', dpi=150, bbox_inches='tight')
print("Saved: waveform_comparison.png")

plt.show()
