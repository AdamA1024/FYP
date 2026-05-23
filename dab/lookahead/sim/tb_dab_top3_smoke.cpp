// End-to-end smoke test for dab_top3 (switch-gen → registered p1/p2 → dab3).
// Validates optimization #5: with PWM_PERIOD=200 and phase_shift=50 (90°), the
// SPS switch generator reproduces dab_ref.py's p1/p2, so the closed plant must
// converge to the same R=10 Ω steady state (V2 ≈ 52 V, i_L bounded).  The extra
// 1-cycle p1/p2 pipeline only shifts the trajectory in time, not its shape.
#include "Vdab_top3.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cmath>

static const double  Q24      = 16777216.0;
static const int32_t V1_Q824  = 0x30000000;  // 48.0
static const int32_t GAMMA    = 0x00000B28;  // R=10 Ω → γ≈1.064e-5

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vdab_top3* dut = new Vdab_top3;

    dut->rst_n = 0; dut->V1 = V1_Q824; dut->gamma_in = GAMMA; dut->phase_shift = 50;
    for (int c = 0; c < 4; c++) { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); }
    dut->rst_n = 1;

    const long N = 600000;                 // 30 ms at dt = 50 ns
    double v = 0, i = 0, vmax = -1e9, imax = 0;
    for (long s = 0; s < N; s++) {
        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();
        v = (int32_t)dut->V2_out / Q24;
        i = (int32_t)dut->i_L_out / Q24;
        if (v > vmax) vmax = v;
        if (fabs(i) > imax) imax = fabs(i);
    }
    dut->final(); delete dut;

    printf("dab_top3 smoke: final V2=%.4f V  i_L=%.4f A   (peak V2=%.4f, peak|i_L|=%.4f)\n",
           v, i, vmax, imax);
    // R=10 steady state from dab_ref.py is V2≈52.4 V; allow a wide sanity band.
    bool ok = (v > 48.0 && v < 56.0) && (imax < 60.0);
    printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
