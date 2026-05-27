"""Diagnose the 5.97 vs 6.00 V steady-state offset.

Runs the 2-step Verlet recurrence the RTL implements, in three configurations:
  (a) EXACT M^2 from single-step M (no linearization, no quantization)
  (b) Linearized M^2 = M2_base + kR*D in float (the package's *formula*)
  (c) Linearized with Q2.16 quantized coefficients (= what the FPGA does)
  (d) (c) with the B1/B2 input split that we just added.
"""
import numpy as np

# Physics (matches verlet_pkg)
L  = 2.25e-6
C  = 1.125e-6
dt = 10e-9
kC = dt / C
h  = dt / (2*L)           # = kL/2  (matches "hL" in the package)
a  = kC * h               # = dt^2/(2 L C)

def single_step_M(kR):
    return np.array([[1 - a - kR,        kC],
                     [-h*(2 - a - kR),   1 - a]])

def u_hat(kR):
    return np.array([a, h*(2 - a - kR)])

def quantize(x, frac_bits=16):
    s = 1 << frac_bits
    return np.round(x * s) / s

# Package coefficients (the same numbers in the .sv)
C_VA  = 65531 / 65536      # +0.999924
C_VB  = 0x048D / 65536
C_VC1 = 4 / 65536          # B1[0]
C_VC2 = 1 / 65536          # B2[0]
C_VC  = 5 / 65536          # old single B[0]
C_VD  = -(((1<<18) - 0x35557) / 65536)  # sign-extend
C_VE  = -(((1<<18) - 0x3FF3E) / 65536)
C_IA  = -(((1<<18) - 0x3FDBA) / 65536)
C_IB  = C_VA
C_IC1 = 0x123 / 65536
C_IC2 = 0x123 / 65536
C_IC  = 0x246 / 65536
C_ID  = 0x0C2 / 65536
C_IE  = 0
S_KR  = 3

FRAC_S = 12   # Q6.12
FRAC_C = 16   # Q2.16
LSB_S  = 1 / (1 << FRAC_S)

USE_ROUNDING = False   # set True to model `+(1<<15)` before the >>>16

def trunc_to_q612(x):
    """Mimics SV `q6_12'(prod >>> 16)`: arithmetic floor by 2^16, then sign-cast.
    For positive x rounds toward zero, for negative x rounds toward -inf (1 LSB bias)."""
    scaled = x * (1 << FRAC_S)
    if USE_ROUNDING:
        scaled = np.floor(scaled + 0.5)   # round-half-up
    else:
        scaled = np.floor(scaled)
    return scaled / (1 << FRAC_S)

def run(mode, kR_phys, Vin=12.0, duty=0.5, period=100, n_periods=2000):
    """mode in {'exact','lin_float','lin_q_oldB','lin_q_newB'}."""
    M = single_step_M(kR_phys)
    M2_exact   = M @ M
    u          = u_hat(kR_phys)
    B_exact_1  = M @ u                    # first  substep input
    B_exact_2  = u                        # second substep input

    # Linearized M^2 = M2_base + kR*D  (drops kR^2)
    M0 = single_step_M(0)
    MkR = np.array([[-1,0],[h,0]])        # dM/dkR
    M2_base = M0 @ M0
    D_lin   = M0 @ MkR + MkR @ M0         # so kR*D_lin is the linear-in-kR part

    if mode == 'exact':
        M2_use = M2_exact
        B1     = B_exact_1
        B2     = B_exact_2
    elif mode == 'lin_float':
        M2_use = M2_base + kR_phys * D_lin
        # Linearized inputs use u_hat at kR=0 (matches package convention)
        u0 = u_hat(0)
        B1 = M0 @ u0
        B2 = u0
    elif mode == 'lin_q_oldB':
        # Quantized coefficients, OLD single-B (= what testbench saw before our fix)
        M2_q = np.array([[C_VA, C_VB],[C_IA, C_IB]])
        D_q  = np.array([[C_VD, C_VE],[C_ID, C_IE]]) * S_KR  # undo the /S
        M2_use = M2_q + kR_phys * D_q
        # OLD B: same coefficient multiplies whichever switch sample the design saw
        B_old = np.array([C_VC, C_IC])
        # In the old design, both "substeps" effectively used the SAME s_k sample.
        # So model it as B1=B_old and B2=0; switch is sampled once per clock.
        # (Equivalently: both substeps use s_k at the time of the multiply.)
        B1 = B_old
        B2 = np.zeros(2)
    elif mode == 'lin_q_newB':
        M2_q = np.array([[C_VA, C_VB],[C_IA, C_IB]])
        D_q  = np.array([[C_VD, C_VE],[C_ID, C_IE]]) * S_KR
        M2_use = M2_q + kR_phys * D_q
        B1 = np.array([C_VC1, C_IC1])
        B2 = np.array([C_VC2, C_IC2])
    else:
        raise ValueError(mode)

    x = np.zeros(2)
    s_prev = 0
    # Build a per-clock switch sequence
    total = n_periods * period
    s_seq = np.array([1 if (k % period) < (period//2) else 0 for k in range(total+2)])
    quantize_intermediates = mode.startswith('lin_q')
    # Pre-split M2 = base + kR*D so we can model the RTL's separate damp path
    if quantize_intermediates:
        M2_base_q = np.array([[C_VA, C_VB],[C_IA, C_IB]])
        D_q       = np.array([[C_VD, C_VE],[C_ID, C_IE]])      # already /S
        kR_in     = kR_phys * S_KR
    vs = []
    for k in range(0, total, 2):
        s1 = s_seq[k]
        s2 = s_seq[k+1]
        if quantize_intermediates:
            # Base path: M2_base @ x + B1*s1 + B2*s2, then a single >>>16 trunc per row
            base = M2_base_q @ x + B1 * s1 * Vin + B2 * s2 * Vin
            base = trunc_to_q612(base)
            # Damp path: (D/S @ x) -> trunc -> * kR_in -> trunc.  Two truncs (matches RTL).
            damp_sum = trunc_to_q612(D_q @ x)
            damp     = trunc_to_q612(damp_sum * kR_in)
            x = base + damp
        else:
            x = M2_use @ x + B1 * s1 * Vin + B2 * s2 * Vin
        vs.append(x[0])
    # Mean over the last 5 periods
    tail = (5 * period) // 2
    return float(np.mean(vs[-tail:])), float(vs[-1])

if __name__ == '__main__':
    print(f"Plant: L={L*1e6} uH, C={C*1e6} uF, dt={dt*1e9} ns, a={a:.4e}")
    print(f"Theoretical DC gain (exact, single-step):  v_ss/Vin = 1 - kR/2\n")
    header = f"{'kR_phys':>10}  {'(a) exact M^2':>14}  {'(b) lin float':>14}  {'(c) lin Q + old B':>18}  {'(d) lin Q + new B':>18}  {'1-kR/2 * D*Vin':>16}"
    print(header)
    print("-"*len(header))
    for kR in [0.0002, 0.0020, 0.0100]:
        a_mean,_ = run('exact',     kR)
        b_mean,_ = run('lin_float', kR)
        c_mean,_ = run('lin_q_oldB',kR)
        d_mean,_ = run('lin_q_newB',kR)
        ideal    = 0.5 * (1 - kR/2) * 12.0
        print(f"{kR:>10.4f}  {a_mean:>14.4f}  {b_mean:>14.4f}  {c_mean:>18.4f}  {d_mean:>18.4f}  {ideal:>16.4f}")
