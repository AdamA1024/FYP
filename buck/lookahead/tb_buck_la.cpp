// tb_buck_lookahead.cpp - Verilator C++ harness (run locally; no network here)
//
// Build (locally):
//   verilator --cc --exe --build -j 0 \
//       buck_coeffs_pkg.sv fpmul.sv duty_gen.sv buck_lookahead.sv \
//       tb_buck_lookahead.cpp --top-module buck_lookahead
//   ./obj_dir/Vbuck_lookahead
//
// It primes x1,x2 via the test ports, runs the pipeline, and prints committed
// (v,i). Compare against rtl_cycle_model.py's look-ahead golden (bit-exact) or
// golden_model output.
//
// Priming values: take x1,x2 from the Python golden (printed by
// rtl_cycle_model.py) and paste their integer codes below.

#include "Vbuck_lookahead.h"
#include "verilated.h"
#include <cstdio>

static Vbuck_lookahead* dut;
static vluint64_t main_time = 0;

void tick() {
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    main_time++;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vbuck_lookahead;

    // ---- reset ----
    dut->rst_n = 0;
    dut->Vin = 12 << 16;        // Q*.16, Vin=12V  (adjust FRAC_S if changed)
    dut->duty_steps = 49;       // ~50% of STEPS=99
    dut->v_init = 0;
    dut->i_init = 0;
    dut->prime_we = 0;
    dut->prime_v1 = 0; dut->prime_i1 = 0;
    dut->prime_v2 = 0; dut->prime_i2 = 0;
    tick(); tick();
    dut->rst_n = 1;
    tick();

    // ---- prime x1, x2 (PASTE codes from rtl_cycle_model.py golden) ----
    // Example placeholders; replace with real golden codes:
    dut->prime_we = 1;
    dut->prime_v1 = 0;     dut->prime_i1 = 3144;   // x[1]
    dut->prime_v2 = 0;     dut->prime_i2 = 3930;   // x[2]
    tick();
    dut->prime_we = 0;

    // ---- run and print commits ----
    printf("cyc, v_out, i_out\n");
    for (int c = 0; c < 60; c++) {
        tick();
        printf("%llu, %d, %d\n",
               (unsigned long long)main_time,
               (int)dut->v_out, (int)dut->i_out);
    }

    delete dut;
    return 0;
}