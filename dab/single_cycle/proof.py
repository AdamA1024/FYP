"""
Formal symbolic verification of the look-ahead transformation.

Proves equivalence between:
  1. RTL Verlet update (from dab_rtl.sv)
  2. Single-step matrix form  x[k+1] = M[k] x[k] + u[k]
  3. 2-step look-ahead form   x[k+2] = M[k+1]M[k] x[k] + M[k+1]u[k] + u[k+1]

If sympy.simplify(diff) == 0 for all comparisons, the algebra is proven
equivalent for ALL values of parameters and switching states.
"""

import sympy as sp

# -----------------------------------------------------------------------------
# Symbolic variables
# -----------------------------------------------------------------------------
alpha, beta, gamma, delta_, n, V1 = sp.symbols('alpha beta gamma delta n V1', real=True)
V2_k, iL_k = sp.symbols('V2_k iL_k', real=True)
V2_k1, iL_k1 = sp.symbols('V2_kp1 iL_kp1', real=True)  # state at k+1
p1k, p2k = sp.symbols('p1_k p2_k')
p1k1, p2k1 = sp.symbols('p1_kp1 p2_kp1')

# -----------------------------------------------------------------------------
# Form 1: RTL Verlet update equations (from dab_rtl.sv, applied at step k)
# -----------------------------------------------------------------------------
def rtl_update(V2, iL, p1, p2):
    """Returns symbolic (V2_next, iL_next) per RTL equations."""
    d = 1 - delta_
    p1_V1    = p1 * V1
    N_p2_V2  = n * p2 * V2
    iL_half  = iL * d + alpha * (p1_V1 - N_p2_V2)
    N_p2_iLh = n * p2 * iL_half
    V2_nd    = V2 + beta * N_p2_iLh
    N_p2_V2nd = n * p2 * V2_nd
    iL_ud    = iL_half + alpha * (p1_V1 - N_p2_V2nd)
    iL_next  = iL_ud * d
    V2_next  = V2_nd - gamma * V2     # NB: uses V2[k], not V2_nd
    return V2_next, iL_next

V2_rtl_k1, iL_rtl_k1 = rtl_update(V2_k, iL_k, p1k, p2k)

# -----------------------------------------------------------------------------
# Form 2: Single-step matrix form (from Python single_step_matrix function)
# -----------------------------------------------------------------------------
def matrix_update(V2, iL, p1, p2):
    d   = 1 - delta_
    np_ = n * p2
    abp = alpha * beta * np_**2

    # V2[k+1] coefficients
    M_VV = 1 - gamma - abp
    M_Vi = beta * np_ * d
    u_V  = alpha * beta * np_ * p1 * V1

    # iL[k+1] coefficients
    M_ii = d**2 * (1 - alpha * beta * np_**2)
    M_iV = -d * alpha * np_ * (2 - alpha * beta * np_**2)
    u_i  = d * alpha * p1 * V1 * (2 - alpha * beta * np_**2)

    V2_next = M_VV * V2 + M_Vi * iL + u_V
    iL_next = M_iV * V2 + M_ii * iL + u_i
    return V2_next, iL_next, sp.Matrix([[M_VV, M_Vi],[M_iV, M_ii]]), sp.Matrix([u_V, u_i])

V2_mat_k1, iL_mat_k1, M_k, u_k = matrix_update(V2_k, iL_k, p1k, p2k)

# -----------------------------------------------------------------------------
# Verification 1: RTL form == single-step matrix form
# -----------------------------------------------------------------------------
print("=" * 70)
print("Verification 1: RTL update  ==  single-step matrix form")
print("=" * 70)

diff_V2 = sp.simplify(sp.expand(V2_rtl_k1 - V2_mat_k1))
diff_iL = sp.simplify(sp.expand(iL_rtl_k1 - iL_mat_k1))
print(f"  V2[k+1] difference (RTL - matrix): {diff_V2}")
print(f"  iL[k+1] difference (RTL - matrix): {diff_iL}")
if diff_V2 == 0 and diff_iL == 0:
    print("  PROVEN: RTL update == single-step matrix form for all parameters.")
else:
    print("  FAIL: forms disagree symbolically.")
print()

# -----------------------------------------------------------------------------
# Verification 2: 2-step look-ahead == 2x RTL applied sequentially
# -----------------------------------------------------------------------------
print("=" * 70)
print("Verification 2: 2-step look-ahead  ==  RTL applied twice")
print("=" * 70)

# Sequential RTL application: x[k] -> x[k+1] -> x[k+2]
V2_seq_k1, iL_seq_k1 = rtl_update(V2_k,    iL_k,    p1k,  p2k)
V2_seq_k2, iL_seq_k2 = rtl_update(V2_seq_k1, iL_seq_k1, p1k1, p2k1)

# Look-ahead form: x[k+2] = M[k+1] M[k] x[k] + M[k+1] u[k] + u[k+1]
_, _, M_k_mat, u_k_vec   = matrix_update(V2_k, iL_k, p1k,  p2k)
_, _, M_k1_mat, u_k1_vec = matrix_update(V2_k, iL_k, p1k1, p2k1)

M2 = M_k1_mat * M_k_mat
u2 = M_k1_mat * u_k_vec + u_k1_vec

x_k = sp.Matrix([V2_k, iL_k])
x_k2_lookahead = M2 * x_k + u2
V2_la_k2 = x_k2_lookahead[0]
iL_la_k2 = x_k2_lookahead[1]

diff_V2_la = sp.simplify(sp.expand(V2_seq_k2 - V2_la_k2))
diff_iL_la = sp.simplify(sp.expand(iL_seq_k2 - iL_la_k2))
print(f"  V2[k+2] difference (sequential - lookahead): {diff_V2_la}")
print(f"  iL[k+2] difference (sequential - lookahead): {diff_iL_la}")
if diff_V2_la == 0 and diff_iL_la == 0:
    print("  PROVEN: 2-step look-ahead == 2x sequential RTL update for all params and switching states.")
else:
    print("  FAIL: forms disagree symbolically.")
print()

# -----------------------------------------------------------------------------
# Verification 3: Check that p1, p2 \in {-1, +1} simplifies p1^2 = p2^2 = 1
# -----------------------------------------------------------------------------
print("=" * 70)
print("Verification 3: substituting p1, p2 in {-1, +1} (the only allowed values)")
print("=" * 70)

# When p1, p2 are +/-1, then p1^2 = p2^2 = 1. SymPy doesn't know this without
# a substitution. Let's check the look-ahead form simplifies cleanly under
# this constraint for all 16 combinations.
all_pass = True
for p1k_val in [-1, +1]:
    for p2k_val in [-1, +1]:
        for p1k1_val in [-1, +1]:
            for p2k1_val in [-1, +1]:
                subs = {p1k: p1k_val, p2k: p2k_val, p1k1: p1k1_val, p2k1: p2k1_val}
                dV = sp.simplify(diff_V2_la.subs(subs))
                dI = sp.simplify(diff_iL_la.subs(subs))
                if dV != 0 or dI != 0:
                    all_pass = False
                    print(f"  FAIL at (p1k,p2k,p1k1,p2k1)=({p1k_val},{p2k_val},{p1k1_val},{p2k1_val}): dV={dV}  dI={dI}")
if all_pass:
    print("  PROVEN: equivalence holds for all 16 combinations of switching states.")
print()

# -----------------------------------------------------------------------------
# Bonus: print the look-ahead M^(2) matrix structure
# -----------------------------------------------------------------------------
print("=" * 70)
print("Look-ahead M^(2) matrix (symbolic, before switching-state substitution)")
print("=" * 70)
print("  Note: only shown for sanity. Real implementation uses 16 precomputed numeric copies.")
print()
M2_simplified = sp.simplify(M2)
print(f"  M^(2)[0,0] = {sp.collect(sp.expand(M2_simplified[0,0]), [alpha, beta, gamma, delta_])}")
print()
print(f"  M^(2)[0,1] = {sp.collect(sp.expand(M2_simplified[0,1]), [alpha, beta, gamma, delta_])}")
print()
print(f"  M^(2)[1,0] = {sp.collect(sp.expand(M2_simplified[1,0]), [alpha, beta, gamma, delta_])}")
print()
print(f"  M^(2)[1,1] = {sp.collect(sp.expand(M2_simplified[1,1]), [alpha, beta, gamma, delta_])}")