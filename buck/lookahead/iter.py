"""
Cycle-by-cycle hardware pipeline trace for the Buck 2-step look-ahead.

This models the ACTUAL FPGA pipeline registers and shows what each register
holds on every clock edge, so you can see how throughput = 1 result/cycle
emerges with a 2-stage pipeline, and why even/odd never collide.

Pipeline structure (2 stages for the M2*x + u2 computation):

   x_in --> [STAGE 1: multiply ]--> reg_mul --> [STAGE 2: add+u2 ]--> x_out
            M2 @ x  (4 mults)                    +u2 (adder tree)

We feed the pipeline alternating even/odd source states each cycle.
The result that emerges is fed BACK as the next source for that same parity,
two cycles later (which is exactly when it arrives).
"""
import numpy as np

# --- Buck params (same as before) ---
L, C, R, Vin, dt, Nper = 10e-6, 100e-6, 5.0, 12.0, 10e-9, 100
kL, kC, kR = dt/L, dt/C, dt/(R*C)
hL = kL/2

M = np.array([[1 - kR - kC*hL, kC],
              [-hL*(2 - kR - kC*hL), 1 - hL*kC]])
def u_vec(s): return np.array([kC*hL*s*Vin, hL*(2 - kC*hL)*s*Vin])
M2 = M @ M
U2 = {(a,b): M @ u_vec(a) + u_vec(b) for a in (0,1) for b in (0,1)}

def buck_step(v, i, s):
    ih = i + hL*(s*Vin - v)
    vn = v + kC*ih - kR*v
    inx = ih + hL*(s*Vin - vn)
    return np.array([vn, inx])

# Switch sequence: all 1s for clarity (duty region)
s_seq = [1]*16

# =============================================================================
# Ground truth (sequential)
# =============================================================================
N = len(s_seq)
truth = [np.array([0.0, 0.0])]
for k in range(N):
    truth.append(buck_step(truth[k][0], truth[k][1], s_seq[k]))

# =============================================================================
# Pipeline model
# =============================================================================
# We track:
#   - The "input register" feeding stage 1 each cycle (holds an x[k])
#   - reg_mul: output of stage-1 multiply (holds M2 @ x for some x), plus the
#     metadata (which index, which u2 key) needed to finish in stage 2
#   - reg_out: the finished x[k+2], written to the state store
#
# State store: holds the most recent known x for each index as it's produced.
# Even parity reads/writes even indices; odd parity reads/writes odd indices.

# Pre-load: x[0] known (initial condition). x[1] from priming (single-step).
x0 = np.array([0.0, 0.0])
x1 = buck_step(x0[0], x0[1], s_seq[0])

# state[k] = computed x[k]
state = {0: x0.copy(), 1: x1.copy()}

print("Initial conditions / priming:")
print(f"  x[0] = (v={x0[0]:.6f}, i={x0[1]:.6f})   [initial condition]")
print(f"  x[1] = (v={x1[0]:.6f}, i={x1[1]:.6f})   [priming single-step]")
print()

# The scheduler: on each cycle we launch ONE new M2-jump into stage 1.
# We alternate parity each cycle: even source, odd source, even, odd, ...
# Even sources in order: x[0], x[2], x[4], ...  -> produce x[2], x[4], x[6]...
# Odd sources in order:  x[1], x[3], x[5], ...  -> produce x[3], x[5], x[7]...

# Source index launched on each cycle (interleave even/odd):
# cycle 0: launch from x[0] (even) -> will produce x[2]
# cycle 1: launch from x[1] (odd)  -> will produce x[3]
# cycle 2: launch from x[2] (even) -> will produce x[4]
# cycle 3: launch from x[3] (odd)  -> will produce x[5]
# ...
def source_index(cycle):
    # even cycles -> even indices, odd cycles -> odd indices
    if cycle % 2 == 0:
        return cycle            # 0,2,4,... mapped from cycles 0,2,4
    else:
        return cycle            # 1,3,5,... mapped from cycles 1,3,5
# Actually simpler: source index == cycle (0,1,2,3,...) gives exactly the
# interleaved even/odd launch we want. Source x[c] produces x[c+2].

# Pipeline registers
reg_stage1 = None   # dict: {'idx':k, 'key':(sk,sk1), 'mul': M2@x[k]} or None
reg_done   = None   # finished result waiting to be committed (for printing)

print("="*92)
print("CYCLE-BY-CYCLE PIPELINE TRACE  (2 stages: MUL then ADD)")
print("="*92)
print(f"{'cyc':>3} | {'STAGE1 in (launch)':>22} | {'STAGE1->reg (mul done)':>26} | {'STAGE2 out (committed)':>26}")
print("-"*92)

n_cycles = N + 2
for cyc in range(n_cycles):
    # ---- STAGE 2 (runs on whatever stage 1 produced last cycle) ----
    committed_str = ""
    if reg_stage1 is not None:
        k        = reg_stage1['idx']
        key      = reg_stage1['key']
        mul      = reg_stage1['mul']
        x_result = mul + U2[key]          # add u2 -> finished x[k+2]
        state[k+2] = x_result
        committed_str = f"x[{k+2}]=(v={x_result[0]:.5f}, i={x_result[1]:.5f})"
        reg_done = (k+2, x_result)

    # ---- STAGE 1 (launch a new jump from x[src]) ----
    launch_str = ""
    mul_str    = ""
    src = cyc  # source index = cycle number
    if src + 2 <= N and src in state:
        x_src = state[src]
        key   = (int(s_seq[src]), int(s_seq[src+1]))
        mul   = M2 @ x_src
        launch_str = f"x[{src}]=(v={x_src[0]:.5f},i={x_src[1]:.5f})"
        mul_str    = f"M2*x[{src}] -> reg"
        reg_stage1_next = {'idx': src, 'key': key, 'mul': mul}
    else:
        reg_stage1_next = None

    print(f"{cyc:>3} | {launch_str:>22} | {mul_str:>26} | {committed_str:>26}")

    # advance pipeline
    reg_stage1 = reg_stage1_next

print()
print("="*92)
print("VERIFY committed pipeline values against sequential ground truth")
print("="*92)
print(f"{'k':>3} | {'truth (v,i)':>28} | {'pipeline (v,i)':>28} | match")
print("-"*92)
for k in sorted(state.keys()):
    t = truth[k]
    p = state[k]
    ok = "OK" if (abs(t[0]-p[0])<1e-9 and abs(t[1]-p[1])<1e-9) else "DIFF"
    print(f"{k:>3} | ({t[0]:>11.6f}, {t[1]:>11.6f}) | ({p[0]:>11.6f}, {p[1]:>11.6f}) | {ok}")

print()
print("="*92)
print("KEY OBSERVATIONS")
print("="*92)
print("""
- Each cycle launches ONE M2-jump (from x[cyc]) and commits ONE result (x[cyc+1]).
  => throughput = 1 finished state per clock cycle (steady state).

- Look at any single cycle: STAGE 1 is multiplying x[cyc] while STAGE 2 is
  adding u2 to the multiply of x[cyc-1]. Those are DIFFERENT indices of
  opposite parity. They never share data within a cycle => no hazard.

- The result x[cyc+1] committed this cycle is NOT needed as a source until
  cycle cyc+1+2-... -> it is needed as source for its own parity exactly 2
  cycles after its own source launched, which is precisely when it's ready.
  The 2-stage latency == the 2-step look-ahead distance. They cancel.

- The clock period only needs to cover ONE stage (one multiply OR one add),
  not the whole 3-equation chain. That's the speedup.
""")