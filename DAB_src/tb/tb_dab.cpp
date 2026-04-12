// =============================================================================
// tb_dab.cpp  –  Verilator testbench for dab_rtl
//
// Generates two phase-shifted square waves (p1, p2) that drive a DAB
// converter model, then prints cycle-by-cycle state as CSV to stdout.
//
// Phase convention
//   p1  : +1 for first half-period, −1 for second half-period.
//   p2  : same waveform but lagging p1 by PHASE_SHIFT clock cycles.
//
// Build & run:  make run   (see Makefile in project root)
// =============================================================================

#include "Vdab_rtl.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ── Simulation knobs ─────────────────────────────────────────────────────────

// Clock cycles per half-period of the switching waveform.
// With Δt = 100 ns and fsw = 10 kHz  →  Tsw = 100 µs  →  half = 500 cycles.
static constexpr int HALF_PERIOD  = 500;

// Phase lag of p2 behind p1, in clock cycles.
// π/4 (45°) = 25% of half-period  →  125 cycles.
static constexpr int PHASE_SHIFT  = 125;

// Total clock cycles to simulate.
static constexpr int SIM_CYCLES   = 400'000;   // 40 ms at Δt = 100 ns

// Print every N cycles to keep the CSV manageable.
static constexpr int PRINT_EVERY  = 10;

// Fixed-point fractional bits — must match RTL FRAC parameter.
static constexpr int FRAC         = 20;

// Primary bus voltage V1 in fixed-point (real_value × 2^FRAC).
// 400 V × 2^20 = 419 430 400
static constexpr int32_t V1_FP    = 400 * (1 << FRAC);

// ── Fixed-point helpers ───────────────────────────────────────────────────────

static inline double fp_to_double(int32_t v)
{
    return static_cast<double>(v) / static_cast<double>(1 << FRAC);
}

// ── Phase-wave generator ─────────────────────────────────────────────────────
// Returns 1 (logic high = +1 sense) or 0 (logic low = −1 sense).

static inline uint8_t square_wave(int cycle, int half_period, int lag_cycles)
{
    int full_period = 2 * half_period;
    // Positive modulo even when (cycle - lag_cycles) is negative
    int phase = ((cycle - lag_cycles) % full_period + full_period) % full_period;
    return (phase < half_period) ? 1u : 0u;
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);

    Vdab_rtl *dut = new Vdab_rtl;

    // ── Reset sequence (two half-cycles) ─────────────────────────────────────
    dut->clk   = 0;
    dut->rst_n = 0;
    dut->V1    = V1_FP;
    dut->p1    = 1;
    dut->p2    = 1;
    dut->eval();

    dut->clk = 1; dut->eval();
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->clk = 0; dut->eval();

    dut->rst_n = 1;

    // ── CSV header ────────────────────────────────────────────────────────────
    std::printf("cycle,p1,p2,i_L_A,V2_V\n");

    // ── Simulation loop ───────────────────────────────────────────────────────
    for (int cycle = 0; cycle < SIM_CYCLES; ++cycle) {

        // Drive inputs before clock edge
        dut->V1 = V1_FP;
        dut->p1 = square_wave(cycle, HALF_PERIOD, 0);
        dut->p2 = square_wave(cycle, HALF_PERIOD, PHASE_SHIFT);

        // Rising edge — state registers update here
        dut->clk = 1;
        dut->eval();

        // Sample outputs (registered, so stable now)
        if (cycle % PRINT_EVERY == 0) {
            std::printf("%d,%u,%u,%.7f,%.7f\n",
                        cycle,
                        static_cast<unsigned>(dut->p1),
                        static_cast<unsigned>(dut->p2),
                        fp_to_double(static_cast<int32_t>(dut->i_L_out)),
                        fp_to_double(static_cast<int32_t>(dut->V2_out)));
        }

        // Falling edge
        dut->clk = 0;
        dut->eval();
    }

    dut->final();
    delete dut;
    return 0;
}
