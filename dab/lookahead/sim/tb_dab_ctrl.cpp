// Reproduce the Vitis controller (vitis/dab_ctrl/src/main.c) operating point on
// the Verilator twin: V1=100 V, R=10 Ω, phase=45° → exact register values the
// firmware writes (V1=0x64000000, gamma=0x476, phase=13). Runs to steady state
// (>5τ, τ=R·Co=4.7 ms) and also samples at 5 ms (the firmware's usleep(5000)).
#include "Vdab_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cmath>

static const double Q24 = 16777216.0;

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vdab_top* dut = new Vdab_top;

    // Literal register values from main.c (Q8.24 V1, Q4.28 gamma, clocks phase).
    dut->V1          = 0x64000000;  // volts_to_q24(100.0)
    dut->gamma_in    = 0x476;       // gamma_from_R(10.0), dt=20ns -> 4.2553e-6
    dut->phase_shift = 13;          // phase_deg_to_clocks(45.0)

    dut->rst_n = 0;
    for (int c = 0; c < 4; c++) { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); }
    dut->rst_n = 1;

    const long N = 1500000;              // 30 ms at dt = 20 ns
    const long S5MS = 250000;            // 5 ms (usleep(5000) in firmware)
    double v = 0, i = 0, v5 = 0, i5 = 0;
    for (long s = 0; s < N; s++) {
        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();
        v = (int32_t)dut->V2_out / Q24;
        i = (int32_t)dut->i_L_out / Q24;
        if (s == S5MS) { v5 = v; i5 = i; }
    }
    dut->final(); delete dut;

    printf("controller op-pt (V1=100, R=10, phase=45/13clk):\n");
    printf("  @5ms   V2 = %.3f V   i_L = %.3f A\n", v5, i5);
    printf("  @30ms  V2 = %.3f V   i_L = %.3f A  (steady)\n", v, i);
    return 0;
}
