// Verilator testbench for dab_switch_gen.sv  —  SPS bridge polarity generator.
//
// Sweeps a set of phase_shift values through the *actual* RTL and logs the
// free-running period counter together with the two bridge polarities p1/p2.
// plot_switch_gen.py turns the CSV into the phase-shift waveform figure.
//
// PWM_PERIOD defaults to 100, so phase_shift is in clock units where
// 100 clocks = one switching period (50 clocks = 180°).
#include "Vdab_switch_gen.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>

// Must match the -GPWM_PERIOD passed to Verilator.  200 clocks/period at the
// 20 MHz (dt=50 ns) solver clock = 100 kHz f_sw, matching the generated
// dab_la_pkg coefficient set (steps/period = 200).
static const int PWM_PERIOD = 200;

// b_pol is `logic signed [1:0]`: +1 = 2'sb01 (raw 1), -1 = 2'sb11 (raw 3).
// Verilator hands us the raw 2-bit pattern; sign-extend it to a real int.
static int pol(uint8_t raw) { return (raw & 0x2) ? (int)raw - 4 : (int)raw; }

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vdab_switch_gen* dut = new Vdab_switch_gen;

    // Phase-shift values to sweep (clocks).  0/50/100/150 -> 0/90/180/270 deg.
    const int phases[] = {0, 50, 100, 150};
    const int NPH       = sizeof(phases) / sizeof(phases[0]);
    const int NPER      = 2;                 // periods to log per phase value
    const int N         = NPER * PWM_PERIOD; // clocks logged per phase value

    FILE* o = fopen("out_switch.csv", "w");
    fprintf(o, "phase,deg,t,p1,p2\n");

    for (int k = 0; k < NPH; k++) {
        int ph     = phases[k];
        double deg = 360.0 * (double)ph / PWM_PERIOD;

        // Apply the phase, then reset so phase_latched is seeded with it.
        dut->phase_shift = ph;
        dut->rst_n = 0;
        for (int c = 0; c < 2; c++) { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); }
        dut->rst_n = 1;

        // Sample one (p1, p2) per clock.  Read while clk is low so the
        // combinational outputs reflect the current registered cnt, then pulse
        // the clock to advance cnt for the next sample.  cnt == t % PWM_PERIOD.
        for (int t = 0; t < N; t++) {
            dut->clk = 0; dut->eval();
            fprintf(o, "%d,%.1f,%d,%d,%d\n",
                    ph, deg, t, pol(dut->p1), pol(dut->p2));
            dut->clk = 1; dut->eval();
        }
    }

    fclose(o);
    dut->final(); delete dut;
    printf("wrote out_switch.csv  (%d phase values x %d clocks)\n", NPH, NPER * PWM_PERIOD);
    return 0;
}
