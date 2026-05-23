# Build Plan: Buck Converter 2-Step Look-Ahead Digital Twin (FPGA) — FIXED R

## Goal
Replace the single-cycle 3-equation Verlet update (~40ns crit path, 500kHz @ 100 steps/period)
with a pipelined 2-step look-ahead (~2 sim steps/clock, target 2-5MHz switching).

## Target devices (Xilinx / Vivado)
- **Primary: 7-series (DSP48E1, 25x18 signed multiplier).** Develop and time against this.
- **Also portable to UltraScale+ (DSP48E2, 27x18 signed).** Same RTL: 18-bit coeff on B port,
  25-bit state on A port fits both. 
- Use `(* use_dsp = "yes" *)` and let Vivado infer DSP pipeline registers (auto).

## Architecture (locked)
- **Open-loop plant** simulation (no on-chip controller).
- **N=2 look-ahead**: `x[k+2] = M2 @ x[k] + u2`, even/odd interleaved trajectories through a
  2-stage pipeline. Pipeline latency (2) == look-ahead distance (2): no stalls.
- **FIXED R**: kR = dt/(R*C) is a COMPILE-TIME CONSTANT. Therefore **M2 and u2 are fully
  precomputed constant** matrices/vectors. No runtime kR, no inv_R input, no A0+kR*A1 split.
- **Runtime inputs**: `Vin` and `duty` only. (Optionally fix Vin too if the app allows —
  then u2 is fully constant and Vin disappears from the datapath.)
- **Switch s[k]**: generated internally by a duty-cycle block (high while step-in-period
  < duty*steps_per_period, else low). Provides s[k] AND s[k+1] (needed for u2 ROM index).

## Matrices (precomputed constants, fixed R)
- `M2 = M @ M`  — single constant 2x2 matrix (switch-independent).
- `u2[s_k, s_kp1] = M @ u(s_k) + u(s_kp1)` — 4 constant 2-vectors, indexed by switch pair.
  - u2 scales linearly with Vin: store `u2 = u2_hat * Vin`, with u2_hat the Vin-independent
    coefficients, multiply by runtime Vin. If Vin is also fixed, bake it in (no multiply).
- where single-step: `M = [[1-kR-kC*hL, kC],[-hL*(2-kR-kC*hL), 1-hL*kC]]`,
  `u(s) = [kC*hL*s*Vin, hL*(2-kC*hL)*s*Vin]`, `hL = kL/2`.

## Fixed-point format (locked)
- **State (v, i)**: Q9.16 signed, 25-bit (DSP 25-bit port). 16 frac bits.
- **Coefficients (M2, u2_hat entries)**: Q2.16 signed, 18-bit (DSP 18-bit port). 16 frac bits.
- **Multiply**: 18b coeff x 25b state -> shift right 16, round-half-up (add 1<<15 pre-shift).
- Use **I+D form** for M2 (store D = M2 - I; compute x + D@x + u2). D entries are all small,
  spend all 16 frac bits on real information, avoids near-1.0 cancellation. Strongly recommended.
- All multiplies share 16 frac bits on both operands -> uniform shift, clean DSP inference.

## Files to create (in order)

### 1. `gen_coeffs.py` (host-side coefficient generator)
- Inputs: L, C, R, dt, steps_per_period, Vin_nominal (all compile-time).
- Compute M2 (or D = M2 - I) and u2_hat[s_k,s_kp1] (4 vectors).
- Emit Q2.16 integer codes as `buck_coeffs_pkg.sv` (localparams).
- Print float values + integer codes for manual verification.

### 2. `duty_gen.sv`
- Inputs: clk, rst_n, duty, steps_per_period (param).
- Step counter mod steps_per_period; advance by 2 per valid cycle (2 steps/jump).
- Outputs `s_now` and `s_next` (the switch pair for the current M2 jump).

### 3. `fpmul.sv` (or inline)
- `(* use_dsp = "yes" *)` 18x25 signed multiply, +round const (1<<15), arithmetic >>16.
- Reused everywhere. Verify DSP inference in synth report.

### 4. `buck_lookahead.sv` (core datapath)
- State store: even register (v_e,i_e) and odd register (v_o,i_o).
- **Priming**: on rst deassert, compute x[1] via ONE single-step (small single-step block,
  or pipeline "prime mode" for one cycle).
- **Per cycle** (with I+D form):
  - Dx = D @ x        (4 mults, 2 adds)    // constant coeffs
  - u2 = u2_hat[s_now,s_next] * Vin        // 2 mults (skip if Vin fixed)
  - x_next = x + Dx + u2                   // 2-3 adds
- **2 pipeline stages**: Stage1 = all multiplies -> regs; Stage2 = adder tree + final sum -> reg.
  Register Vin alongside (pipeline-align). Alternate even/odd source each cycle; commit to
  matching parity register.

### 5. `golden_model.py` (reference)
- Bit-accurate fixed-point Python model (reuse buck_precision*.py logic, fixed R).
- Generate expected (v,i) trace for the TB stimulus; dump via $readmemh.

### 6. `tb_buck_lookahead.sv` (testbench)
- Drive Vin, duty. Compare RTL vs golden model cycle-by-cycle.
- Sweep: Vin change, duty change. Assert max abs error bound.

## Verification gates (pass before next stage)
1. gen_coeffs.py integer codes match hand-checked float values.
2. Golden fixed-point model matches float baseline (~1e-3 V at Q*.16).
3. RTL matches golden model BIT-EXACTLY in sim.


## Key correctness invariants (comments + assertions)
- s_now/s_next must correspond to the SAME source index k that x came from.
- Even trajectory only reads/writes even indices; odd only odd. Never cross.
- Priming x[1] uses the single-step update (NOT M2).


## Stretch (after N=2 works end-to-end)
- N=4 look-ahead (4 interleaved trajectories, 4-stage pipeline) for higher fsw, or use M+P DSP registers, but this will require careful assessment of the number of pipeline stages. 