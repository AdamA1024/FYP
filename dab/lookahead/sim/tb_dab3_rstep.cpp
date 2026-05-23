// Verilator testbench: runtime-R (load-step) test for the optimized DAB engine
// (dab3.sv).  Drives the exact (p1, p2, γ) schedule from ref_rstep.csv and logs
// both the engine trajectory and the golden trajectory so plot_rstep.py can show
// how the background γ-fold tracks an instantaneous load step.
#include "Vdab_look_ahead_solver.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstdint>

static const double Q24 = 16777216.0;       // 2^24, state scale
static const int32_t V1_Q824 = 0x30000000;  // 48.0 in Q8.24

// encode polarity {+1,0,-1} into 2-bit two's complement
static uint8_t pol(int p) { return p > 0 ? 0x1 : (p < 0 ? 0x3 : 0x0); }

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vdab_look_ahead_solver* dut = new Vdab_look_ahead_solver;

    FILE* f = fopen("ref_rstep.csv", "r");
    if (!f) { fprintf(stderr, "ref_rstep.csv missing - run dab_ref_rstep.py first\n"); return 1; }
    char line[256];
    if (!fgets(line, sizeof(line), f)) return 1;  // header

    // Read the very first data row to know the initial γ to apply during reset.
    long  first_pos = ftell(f);
    int   idx0, p1_0, p2_0, gq0; double ir0, vr0;
    if (!fgets(line, sizeof(line), f) ||
        sscanf(line, "%d,%d,%d,%d,%lf,%lf", &idx0, &p1_0, &p2_0, &gq0, &ir0, &vr0) != 6) {
        fprintf(stderr, "ref_rstep.csv has no data rows\n"); return 1;
    }
    fseek(f, first_pos, SEEK_SET);  // rewind to first data row

    // reset (end with clk low so the main loop owns each rising edge)
    dut->rst_n = 0; dut->V1 = V1_Q824; dut->p1 = 0; dut->p2 = 0;
    dut->gamma_in = gq0;
    for (int c = 0; c < 4; c++) { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); }
    dut->rst_n = 1; dut->clk = 0; dut->eval();

    FILE* o = fopen("out_rstep.csv", "w");
    fprintf(o, "idx,i_sv,V2_sv,i_ref,V2_ref,gamma_q428\n");

    double max_e_i = 0, max_e_v = 0, sse_i = 0, sse_v = 0;
    long n = 0;

    // Same 1-cycle look-ahead output latency as tb_dab.cpp: engine output after
    // applying row k is compared against golden row (k-1).  We hold one row back.
    int    prev_idx = -1, prev_gq = gq0;
    double prev_i_ref = 0.0, prev_v2_ref = 0.0;
    bool   have_prev = false;

    int idx, p1, p2, gq; double i_ref, v2_ref;
    while (fgets(line, sizeof(line), f)) {
        if (sscanf(line, "%d,%d,%d,%d,%lf,%lf", &idx, &p1, &p2, &gq, &i_ref, &v2_ref) != 6)
            continue;
        dut->p1 = pol(p1); dut->p2 = pol(p2); dut->V1 = V1_Q824;
        dut->gamma_in = gq;                              // runtime load update

        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();

        double i_sv = (int32_t)dut->i_L_out / Q24;
        double v_sv = (int32_t)dut->V2_out / Q24;

        if (have_prev) {  // engine state now aligns with golden row (idx-1)
            double ei = fabs(i_sv - prev_i_ref), ev = fabs(v_sv - prev_v2_ref);
            if (ei > max_e_i) max_e_i = ei;
            if (ev > max_e_v) max_e_v = ev;
            sse_i += ei * ei; sse_v += ev * ev; n++;
            // log every 5th step (sub-period resolution is unnecessary for ms transients)
            if (prev_idx % 5 == 0)
                fprintf(o, "%d,%.6f,%.6f,%.6f,%.6f,%d\n",
                        prev_idx, i_sv, v_sv, prev_i_ref, prev_v2_ref, prev_gq);
        }
        prev_idx = idx; prev_i_ref = i_ref; prev_v2_ref = v2_ref; prev_gq = gq;
        have_prev = true;
    }
    fclose(f); fclose(o);
    dut->final(); delete dut;

    printf("steps compared : %ld\n", n);
    printf("i_L : max|err|=%.6f A   rms=%.6f A\n", max_e_i, sqrt(sse_i / n));
    printf("V2  : max|err|=%.6f V   rms=%.6f V\n", max_e_v, sqrt(sse_v / n));
    // Across abrupt load steps a few-LSB transient is expected and acceptable.
    bool ok = (max_e_i < 0.10) && (max_e_v < 0.10);
    printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
