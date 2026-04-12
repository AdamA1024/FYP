// Verilator testbench / digital controller for the buck_verlet digital twin.
//
// Architecture: Soft-start + Voltage Feedforward + PI feedback
// ─────────────────────────────────────────────────────────────
// Plant (buck converter, average model):
//   v_out / d = Vin / (LCs² + (L/R)s + 1)
//   ωn = 1/√(LC) ≈ 31.6 krad/s (~5 kHz),  ζ ≈ 0.032  (underdamped, Q ≈ 16)
//
// The high Q means a plain PI loop needs its crossover well below resonance
// to stay stable; the large startup error (0 → V_REF) also causes severe
// integral windup that drives v_out far past the setpoint.
//
// Three-layer control strategy
// ─────────────────────────────
// 1. Soft-start ramp (first SOFT_PERIODS switching periods):
//      v_ref_active rises linearly 0 → V_REF over ~500 µs.
//      Limits startup inductor current and prevents integral windup.
//
// 2. Voltage feedforward:
//      d_ff = v_ref_active / Vin
//      Tracks Vin changes immediately — the main disturbance rejection.
//      Because D·Vin = (V_REF/Vin)·Vin = V_REF, a simultaneous Vin step
//      with correct feedforward causes zero transient in v_out.
//
// 3. PI feedback (small-signal trim only):
//      e[n]          = v_ref_active[n] - v_out[n]
//      Ki_sum[n+1]   = clamp(Ki_sum[n] + Ki·e[n], I_MIN, I_MAX)
//      u[n]          = Kp·e[n] + Ki_sum[n]
//      duty[n]       = clamp(d_ff[n] + u[n], D_MIN, D_MAX)
//
//  Stability constraint — the plant LC resonance (ωn ≈ 31.6 krad/s, Q ≈ 16)
//  peaks at |G(jωn)| = Q·Vin ≈ 190 V per unit duty.  For stability we need
//  loop gain at ωn below unity:
//      |Kp + Ki/(ωn·Ts)| · 190 < 1   →   Kp < 0.003,  Ki < 0.001
//  Because feedforward handles nearly all the control effort, the slow
//  feedback time constant (~10 ms) is acceptable — residual errors are
//  already < 0.5% from PWM quantisation alone.
//
// Gains: Kp = 0.002,  Ki = 0.0008 / switching-period (Ts = 10 µs)
//
// Scenario (same Vin steps as open-loop test, R = 5 Ω fixed):
//   Phase 0  [0 ms – 2 ms]   Vin = 12.0 V  initial convergence  -> 6.0 V
//   Phase 1  [2 ms – 4 ms]   Vin =  9.0 V  input step down      -> 6.0 V
//   Phase 2  [4 ms – 6 ms]   Vin = 15.0 V  input step up        -> 6.0 V
//
// Output: buck_ctrl.csv  columns: cycle, time_us, vin_V, duty_pct, i_A, v_V

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include "verilated.h"
#include "Vbuck_verlet.h"
#include "../sim_params.h"

// Q8.24 -> double
static inline double fp_to_real(int32_t x) {
    return static_cast<double>(x) / (1 << 24);
}

static void tick(Vbuck_verlet *dut) {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
}

// Return v_in (Q8.24) and Vin (double) for the given cycle.
static void phase_vin(int cycle, int32_t &v_in_fp, double &vin_val) {
    if (cycle < STEP1_CYCLE) {
        v_in_fp = VIN0_FP;  vin_val = 12.0;
    } else if (cycle < STEP2_CYCLE) {
        v_in_fp = VIN1_FP;  vin_val =  9.0;
    } else {
        v_in_fp = VIN2_FP;  vin_val = 15.0;
    }
}

// --- Controller constants ---
static const double V_REF = 6.0;   // target output voltage (V)

// PI gains constrained by LC resonance stability (see header comment):
//   |Kp + Ki/(ωn·Ts)| · Q·Vin < 1  →  Kp < 0.003,  Ki < 0.001
static const double KP    = 0.002;  // PI proportional gain  (duty / V)
static const double KI    = 0.0008; // PI integral gain      (duty / (V · switching-period))
static const double D_MIN = 0.005;
static const double D_MAX = 0.995;

// Tight integral window: feedforward covers most of any Vin step.
// Residual errors are < 0.5% from PWM quantisation, so ±0.02 is ample.
static const double I_MIN = -0.02;
static const double I_MAX =  0.02;

// Soft-start: ramp active reference from 0 → V_REF over this many periods.
// 50 periods × 10 µs/period = 500 µs — safely within phase 0's 2 ms window.
static const int SOFT_PERIODS = 50;

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    Vbuck_verlet *dut = new Vbuck_verlet;

    // --- Apply reset ---
    dut->clk    = 0;
    dut->rst    = 1;
    dut->sk     = 0;
    dut->g_load = GLOAD_FIXED;
    dut->v_in   = VIN0_FP;
    for (int i = 0; i < 8; i++) tick(dut);
    dut->rst = 0;

    FILE *fp = fopen("buck_ctrl.csv", "w");
    if (!fp) { perror("fopen"); return 1; }
    fprintf(fp, "cycle,time_us,vin_V,duty_pct,i_A,v_V\n");

    // --- Controller state ---
    double ki_sum   = 0.0;
    int    period   = 0;          // switching-period counter (for soft-start)
    double duty     = 0.0;        // starts at 0 (v_ref_active = 0 initially)
    int    duty_cyc = 0;

    // --- Per-phase statistics ---
    double v_sum[3] = {}, i_sum[3] = {};
    int    v_cnt[3] = {};
    const int half_phase = STEP1_CYCLE / 2;

    for (int cycle = 0; cycle < SIM_CYCLES; cycle++) {

        // --- Determine this cycle's Vin ---
        int32_t v_in_fp;
        double  vin_val;
        phase_vin(cycle, v_in_fp, vin_val);

        // --- Update controller once per switching period ---
        if (cycle % PWM_PERIOD == 0) {

            // Soft-start: ramp active reference from 0 → V_REF
            double v_ref_active;
            if (period < SOFT_PERIODS) {
                v_ref_active = V_REF * (period + 1.0) / SOFT_PERIODS;
            } else {
                v_ref_active = V_REF;
            }

            // Feedforward: ideal duty for the current Vin and active reference
            double d_ff = v_ref_active / vin_val;
            d_ff = std::max(D_MIN, std::min(D_MAX, d_ff));

            // PI feedback (error against the ramped reference)
            double v_meas = fp_to_real(static_cast<int32_t>(dut->v_out));
            double error  = v_ref_active - v_meas;

            double ki_new = ki_sum + KI * error;
            ki_new = std::max(I_MIN, std::min(I_MAX, ki_new));

            double u = KP * error + ki_new;
            duty     = std::max(D_MIN, std::min(D_MAX, d_ff + u));
            duty_cyc = (int)std::round(duty * PWM_PERIOD);
            duty_cyc = std::max(1, std::min(PWM_PERIOD - 1, duty_cyc));

            ki_sum = ki_new;
            period++;
        }

        // --- Drive DUT ---
        dut->v_in   = v_in_fp;
        dut->g_load = GLOAD_FIXED;
        dut->sk     = ((cycle % PWM_PERIOD) < duty_cyc) ? 1 : 0;

        tick(dut);

        double time_us  = cycle * 0.05;
        double i_A      = fp_to_real(static_cast<int32_t>(dut->i_out));
        double v_V      = fp_to_real(static_cast<int32_t>(dut->v_out));
        double duty_pct = 100.0 * duty_cyc / PWM_PERIOD;

        if (cycle % LOG_EVERY == 0)
            fprintf(fp, "%d,%.3f,%.1f,%.3f,%.6f,%.6f\n",
                    cycle, time_us, vin_val, duty_pct, i_A, v_V);

        // Accumulate steady-state stats over second half of each phase
        int phase     = (cycle < STEP1_CYCLE) ? 0
                      : (cycle < STEP2_CYCLE) ? 1 : 2;
        int phase_cyc = cycle - phase * STEP1_CYCLE;
        if (phase_cyc >= half_phase) {
            v_sum[phase] += v_V;
            i_sum[phase] += i_A;
            v_cnt[phase]++;
        }
    }
    fclose(fp);

    // --- Summary ---
    const double vin_vals[3] = {12.0,  9.0, 15.0};
    const double t_starts[3] = { 0.0,  2.0,  4.0};

    printf("=== Buck Converter – Feedforward + PI Controller Simulation ===\n");
    printf("  Soft-start: %d periods (%.0f µs)\n", SOFT_PERIODS,
           SOFT_PERIODS * (double)PWM_PERIOD * 0.05);
    printf("  Kp=%.4f  Ki=%.5f/period  V_ref=%.1f V  R=5.0 Ω\n\n",
           KP, KI, V_REF);
    printf("  %-8s  %-10s  %-14s  %-14s  %-8s\n",
           "Phase", "Vin (V)", "Avg v_out (V)", "Avg i_L (A)", "Error");
    printf("  %s\n", std::string(60, '-').c_str());
    for (int p = 0; p < 3; p++) {
        double v_avg   = (v_cnt[p] > 0) ? v_sum[p] / v_cnt[p] : 0.0;
        double i_avg   = (v_cnt[p] > 0) ? i_sum[p] / v_cnt[p] : 0.0;
        double err_pct = 100.0 * (v_avg - V_REF) / V_REF;
        printf("  [%.0f–%.0f ms]   %-10.1f  %-14.5f  %-14.5f  %+.3f%%\n",
               t_starts[p], t_starts[p] + 2.0, vin_vals[p],
               v_avg, i_avg, err_pct);
    }
    printf("\n  Results: buck_ctrl.csv\n");

    delete dut;
    return 0;
}
