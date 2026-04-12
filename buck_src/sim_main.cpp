// Verilator testbench: input-voltage-step test for buck_verlet digital twin.
//
// Tests that v_out converges to D*Vin after sudden changes in input voltage.
// The ideal buck gives V_out = D * Vin = 0.5 * Vin.
//
// Scenario (each phase is 2 ms = 40_000 cycles):
//   Phase 0  [0 ms – 2 ms]   Vin = 12.0 V  initial convergence to 6.0 V
//   Phase 1  [2 ms – 4 ms]   Vin =  9.0 V  input step down  -> re-converges to 4.5 V
//   Phase 2  [4 ms – 6 ms]   Vin = 15.0 V  input step up    -> re-converges to 7.5 V
//
// Output: buck_sim.csv  columns: cycle, time_us, vin_V, i_A, v_V

#include <cstdio>
#include <cstdint>
#include <cmath>
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

// Return v_in (Q8.24) and Vin value for the given cycle.
static void phase_params(int cycle, int32_t &v_in, double &vin_val) {
    if (cycle < STEP1_CYCLE) {
        v_in = VIN0_FP;  vin_val = 12.0;
    } else if (cycle < STEP2_CYCLE) {
        v_in = VIN1_FP;  vin_val =  9.0;
    } else {
        v_in = VIN2_FP;  vin_val = 15.0;
    }
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    Vbuck_verlet *dut = new Vbuck_verlet;

    // Apply reset (hold for 8 cycles, sk=0, g_load=fixed, v_in=nominal)
    dut->clk    = 0;
    dut->rst    = 1;
    dut->sk     = 0;
    dut->g_load = GLOAD_FIXED;
    dut->v_in   = VIN0_FP;
    for (int i = 0; i < 8; i++) tick(dut);
    dut->rst = 0;

    FILE *fp = fopen("buck_sim.csv", "w");
    if (!fp) { perror("fopen"); return 1; }
    fprintf(fp, "cycle,time_us,vin_V,i_A,v_V\n");

    // Per-phase statistics (3 phases)
    double v_sum[3] = {}, i_sum[3] = {};
    int    v_cnt[3] = {};
    // Average over the second half of each phase to skip transient
    const int half_phase = (STEP1_CYCLE) / 2;   // 20_000 cycles = 1 ms

    for (int cycle = 0; cycle < SIM_CYCLES; cycle++) {
        int32_t v_in_fp;
        double  vin_val;
        phase_params(cycle, v_in_fp, vin_val);

        dut->v_in   = v_in_fp;
        dut->g_load = GLOAD_FIXED;
        // 50% duty-cycle PWM
        dut->sk = ((cycle % PWM_PERIOD) < DUTY_CYCLES) ? 1 : 0;

        tick(dut);

        double time_us = cycle * 0.05;   // dt = 50 ns
        double i_A = fp_to_real(static_cast<int32_t>(dut->i_out));
        double v_V = fp_to_real(static_cast<int32_t>(dut->v_out));

        if (cycle % LOG_EVERY == 0)
            fprintf(fp, "%d,%.3f,%.1f,%.6f,%.6f\n",
                    cycle, time_us, vin_val, i_A, v_V);

        // Accumulate steady-state average over second half of each phase
        int phase      = (cycle < STEP1_CYCLE) ? 0
                       : (cycle < STEP2_CYCLE) ? 1 : 2;
        int phase_cyc  = cycle - phase * STEP1_CYCLE;
        if (phase_cyc >= half_phase) {
            v_sum[phase] += v_V;
            i_sum[phase] += i_A;
            v_cnt[phase]++;
        }
    }
    fclose(fp);

    // --- Print per-phase summary ---
    const double vin_vals[3]   = {12.0, 9.0, 15.0};
    const double t_starts[3]   = {0.0, 2.0, 4.0};
    const double duty          = (double)DUTY_CYCLES / PWM_PERIOD;

    printf("=== Buck Converter Input-Voltage-Step Simulation ===\n");
    printf("  PWM: period=%d cycles, duty=%d/%d (%.0f%%)  R=5.0 Ω\n",
           PWM_PERIOD, DUTY_CYCLES, PWM_PERIOD,
           100.0 * duty);
    printf("  %-8s  %-10s  %-12s  %-12s  %-12s  %-8s\n",
           "Phase", "Vin (V)", "Exp v_out (V)", "Avg v_out (V)", "Avg i_L (A)", "Error");
    printf("  %s\n", std::string(70, '-').c_str());
    for (int p = 0; p < 3; p++) {
        double v_avg    = (v_cnt[p] > 0) ? v_sum[p] / v_cnt[p] : 0.0;
        double i_avg    = (v_cnt[p] > 0) ? i_sum[p] / v_cnt[p] : 0.0;
        double expected = vin_vals[p] * duty;
        double err_pct  = 100.0 * (v_avg - expected) / expected;
        printf("  [%.0f–%.0f ms]   %-10.1f  %-12.3f  %-12.5f  %-12.5f  %+.3f%%\n",
               t_starts[p], t_starts[p] + 2.0, vin_vals[p],
               expected, v_avg, i_avg, err_pct);
    }
    printf("\n  Results: buck_sim.csv\n");

    delete dut;
    return 0;
}
