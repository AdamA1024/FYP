// =============================================================================
// tb_dab_top.cpp  -  Verilator testbench for dab_top
//
// Drives dab_top with a fixed PHASE value through the AXI-style `phase`
// input and writes a CSV trace of (cycle, p1, p2, i_L, V2) to stdout.
// Steady-state min/max are summarised at the end on stderr and checked
// against the expected open-loop operating point.
//
// Expected steady state (PERIOD=100, PHASE=13, defaults from dab_rtl.sv):
//   V2 mean   ~40.4 V    (open-loop, phi = 13/50 * pi ~= 46.8 deg, fsw=1 MHz)
//                        38.5 V from ideal physics + ~5% lift from GAMMA
//                        rounding (R_eff ≈ 10.5 Ω, see dab_rtl.sv).
//   V2 ripple ~ small (Tsw halved vs. previous 500 kHz design)
//   i_L       symmetric about 0, peak ~ a few A
// =============================================================================

#include "Vdab_top.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

// ----- Simulation knobs ------------------------------------------------------
static constexpr int     PERIOD      = 100;      // cycles per Tsw (matches RTL)
static constexpr int     PHASE       = 13;       // fixed phase shift in cycles
static constexpr int     SIM_CYCLES  = 500'000;  // 5 ms at dt = 10 ns (5*tau)
static constexpr int     PRINT_EVERY = 5;        // 20 samples / switching period
static constexpr int     FRAC        = 20;
static constexpr int32_t V1_FP       = 400 * (1 << FRAC);  // 400 V in Q11.20

// Last 10 % of the run is treated as steady state for the pass/fail check.
static constexpr int     SS_START    = (SIM_CYCLES * 9) / 10;
static constexpr double  V2_EXPECTED = 40.4;
static constexpr double  V2_TOL      = 1.5;       // V

static inline double fp_to_double(int32_t v) {
    return static_cast<double>(v) / static_cast<double>(1 << FRAC);
}

// Pulse a clock edge: drive falling -> rising and evaluate the DUT.
static inline void tick(Vdab_top *dut) {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    Vdab_top *dut = new Vdab_top;

    // ----- Inputs held stable across reset -----
    dut->V1    = V1_FP;
    dut->phase = PHASE;

    // ----- Reset: hold rst_n low for a few cycles -----
    dut->rst_n = 0;
    dut->clk   = 0; dut->eval();
    for (int i = 0; i < 4; ++i) tick(dut);
    dut->rst_n = 1;

    std::printf("cycle,i_L_A,V2_V\n");

    double v2_min = 0.0, v2_max = 0.0, v2_sum = 0.0;
    double il_min = 0.0, il_max = 0.0;
    int    ss_n   = 0;

    for (int cycle = 0; cycle < SIM_CYCLES; ++cycle) {
        // Inputs are already stable; just clock the DUT.
        tick(dut);

        const double iL = fp_to_double(static_cast<int32_t>(dut->i_L_out));
        const double V2 = fp_to_double(static_cast<int32_t>(dut->V2_out));

        if (cycle % PRINT_EVERY == 0) {
            std::printf("%d,%.7f,%.7f\n", cycle, iL, V2);
        }

        if (cycle >= SS_START) {
            if (ss_n == 0) {
                v2_min = v2_max = V2;
                il_min = il_max = iL;
            } else {
                if (V2 < v2_min) v2_min = V2;
                if (V2 > v2_max) v2_max = V2;
                if (iL < il_min) il_min = iL;
                if (iL > il_max) il_max = iL;
            }
            v2_sum += V2;
            ++ss_n;
        }
    }

    dut->final();
    delete dut;

    const double v2_mean = (ss_n > 0) ? (v2_sum / ss_n) : 0.0;

    std::fprintf(stderr,
        "\n--- steady-state summary (last %d cycles) ---\n"
        "  V2  mean = %7.3f V   min = %7.3f V   max = %7.3f V   pp = %5.3f V\n"
        "  i_L                  min = %7.3f A   max = %7.3f A   pp = %5.3f A\n"
        "  phase = %d cycles    expected V2 ~ %.2f V (+-%.1f V)\n",
        SIM_CYCLES - SS_START,
        v2_mean, v2_min, v2_max, v2_max - v2_min,
        il_min, il_max, il_max - il_min,
        PHASE, V2_EXPECTED, V2_TOL);

    const bool pass =
        (std::abs(v2_mean - V2_EXPECTED) <= V2_TOL) &&
        (v2_max - v2_min) < 20.0 &&        // sanity: not blowing up
        (il_max > 0.0) && (il_min < 0.0);  // sanity: i_L is bipolar

    std::fprintf(stderr, "  result : %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
