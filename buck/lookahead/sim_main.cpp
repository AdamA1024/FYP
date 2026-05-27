// sim_main.cpp - Verilator harness for buck_verlet_mixed_precision.sv
//
// Drives the 2-step look-ahead Verlet buck digital twin with a 50% duty
// PWM source (Vin = 12 V) and sweeps a few values of the dynamic damping
// parameter kR (Q2.16) to check whether v_out converges toward ~6 V.
//
// State format: Q6.12 (18-bit signed) -> LSB = 1/4096 V or A
// Coeff format: Q2.16 (18-bit signed) -> LSB = 1/65536

#include "Vbuck_verlet_mixed_precision.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>

static constexpr int Q6_12_FRAC  = 12;
static constexpr int Q2_16_FRAC  = 16;
static constexpr int Q6_12_SCALE = 1 << Q6_12_FRAC;   // 4096
static constexpr int Q2_16_SCALE = 1 << Q2_16_FRAC;   // 65536

// Sign-extend an 18-bit two's-complement value carried in a 32-bit int.
static inline int32_t sext18(uint32_t v) {
    v &= 0x3FFFFu;
    if (v & 0x20000u) v |= 0xFFFC0000u;
    return static_cast<int32_t>(v);
}

static inline double q6_12_to_f(uint32_t code) {
    return static_cast<double>(sext18(code)) / Q6_12_SCALE;
}

static int q6_12_code(double x) {
    long c = std::lround(x * Q6_12_SCALE);
    if (c >  131071) c =  131071;
    if (c < -131072) c = -131072;
    return static_cast<int>(c & 0x3FFFFu);
}

static int q2_16_code(double x) {
    long c = std::lround(x * Q2_16_SCALE);
    if (c >  131071) c =  131071;
    if (c < -131072) c = -131072;
    return static_cast<int>(c & 0x3FFFFu);
}

struct RunResult {
    double v_final;
    double i_final;
    double v_mean_last;   // mean over the last full PWM period
    double v_min_last;
    double v_max_last;
};

// D-row scale baked into the verlet_pkg coefficients: the package shrinks
// the kR-dependent damping coefficients by S=16, so the user must drive
// kR_in = S * (dt/(R*C)).
static constexpr double KR_SCALE = 3.0;

static RunResult run_sim(Vbuck_verlet_mixed_precision* dut,
                        VerilatedVcdC* trace,
                        double vin_volts,
                        double kR_float,       // physical kR = dt/(R*C)
                        int    pwm_period,     // total clocks per PWM period
                        int    pwm_on_clocks,  // on-time clocks per period
                        int    total_clocks,
                        bool   verbose,
                        FILE*  csv,
                        const char* tag) {
    // ---- reset ----
    // Hot-bootstrap inputs were removed from the module port list to keep
    // the Vivado I/O pin budget under 125; the module now zero-seeds its
    // state registers internally on rst_n=0.
    dut->rst_n     = 0;
    dut->kR        = q2_16_code(KR_SCALE * kR_float);
    dut->Vin       = q6_12_code(vin_volts);
    dut->s_k       = 0;

    auto tick = [&](vluint64_t& t) {
        dut->clk = 0; dut->eval(); if (trace) trace->dump(t); t++;
        dut->clk = 1; dut->eval(); if (trace) trace->dump(t); t++;
    };

    static vluint64_t sim_time = 0;
    // hold reset two clocks
    tick(sim_time); tick(sim_time);
    dut->rst_n = 1;

    double v_sum_last = 0.0;
    double v_min_last = +1e30;
    double v_max_last = -1e30;
    int    last_period_samples = 0;
    const int last_period_start = total_clocks - pwm_period;

    for (int c = 0; c < total_clocks; ++c) {
        int phase = c % pwm_period;
        bool s_on = (phase < pwm_on_clocks);
        dut->s_k = s_on ? 1 : 0;

        tick(sim_time);

        double v = q6_12_to_f(dut->v_out);
        double i = q6_12_to_f(dut->i_out);

        if (csv) {
            fprintf(csv, "%s,%d,%d,%.6f,%.6f\n", tag, c, s_on ? 1 : 0, v, i);
        }
        if (verbose && (c % (total_clocks/40 == 0 ? 1 : total_clocks/40) == 0)) {
            printf("  c=%6d  s=%d  v=%+8.4f  i=%+8.4f\n", c, s_on?1:0, v, i);
        }
        if (c >= last_period_start) {
            v_sum_last += v;
            if (v < v_min_last) v_min_last = v;
            if (v > v_max_last) v_max_last = v;
            last_period_samples++;
        }
    }

    RunResult r;
    r.v_final = q6_12_to_f(dut->v_out);
    r.i_final = q6_12_to_f(dut->i_out);
    r.v_mean_last = (last_period_samples > 0)
                    ? v_sum_last / last_period_samples
                    : r.v_final;
    r.v_min_last = v_min_last;
    r.v_max_last = v_max_last;
    return r;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    auto* dut = new Vbuck_verlet_mixed_precision;

    // Optional: dump VCD only for the "main" run
    VerilatedVcdC* trace = new VerilatedVcdC;
    dut->trace(trace, 99);
    trace->open("simulation_trace.vcd");

    const double Vin = 12.0;
    const int PWM_PERIOD    = 100;        // clocks per PWM period
    const int PWM_ON        = 50;         // 50% duty
    const int TOTAL_CLOCKS  = 20000;      // ~200 PWM periods

    printf("=== buck_verlet_mixed_precision convergence test ===\n");
    printf("Vin=%.1f V, PWM period=%d clk, duty=%d/%d, total clocks=%d\n",
           Vin, PWM_PERIOD, PWM_ON, PWM_PERIOD, TOTAL_CLOCKS);
    printf("State format Q6.12 -> range [-32, +32) V/A, LSB = %.6f\n",
           1.0/Q6_12_SCALE);
    printf("kR fmt Q2.16 -> range [-2, +2), LSB = %.3e\n\n",
           1.0/Q2_16_SCALE);

    // Sweep kR (the dynamic damping parameter dt/(R*C))
    // and a couple of PWM periods.
    // New plant has |eig(M^2)| ~ 1 - 2*kR, so time constants are ~1/(2*kR)
    // cycles.  At kR_phys=0.002 -> tau ~ 250 cycles = 2.5 PWM periods.
    // At kR_phys=0.0002 -> tau ~ 2500 cycles -> need ~10x more total clocks.
    struct Cfg { double kR; int period; int on; int total; const char* tag; };
    std::vector<Cfg> sweeps = {
        // 50% PWM @ 100 clk (canonical: 1 MHz switching at 100 MHz fpga clk)
        {0.0000, 100,  50, 100000, "pwm50 kR=0.0000"},
        {0.0002, 100,  50, 100000, "pwm50 kR=0.0002"},
        {0.0020, 100,  50, 100000, "pwm50 kR=0.0020"},
        {0.0100, 100,  50, 100000, "pwm50 kR=0.0100"},
        // Constant input (DC gain check)
        {0.0000,   1,   1,  20000, "DC v_in_s=12 kR=0.0000"},
        {0.0020,   1,   1,  50000, "DC v_in_s=12 kR=0.0020"},
        {0.0100,   1,   1,  20000, "DC v_in_s=12 kR=0.0100"},
    };

    FILE* csv = fopen("trace.csv", "w");
    fprintf(csv, "tag,cycle,s,v_out,i_out\n");

    // Print waypoints for the canonical 50% / kR=0.02 case so we can see
    // the convergence trajectory.
    {
        printf("\n--- Trajectory dump: 50%% PWM @ 100 clk, kR=0.02 ---\n");
        printf("  %-8s %-10s %-10s\n", "cycle", "v_out [V]", "i_out [A]");
        // re-run, tapping waypoints
        dut->rst_n = 0;
        dut->kR = q2_16_code(KR_SCALE * 0.002);
        dut->Vin = q6_12_code(12.0);
        dut->s_k = 0;
        static vluint64_t t = 100000000;  // distinct VCD region
        auto tk = [&](){ dut->clk=0; dut->eval(); dut->clk=1; dut->eval(); t+=2; };
        tk(); tk();
        dut->rst_n = 1;
        const std::vector<int> waypoints = {0,100,300,1000,3000,10000,30000,60000,99999};
        std::vector<double> vs(100000), is_(100000);
        for (int c = 0; c < 100000; ++c) {
            dut->s_k = ((c % 100) < 50) ? 1 : 0;
            tk();
            vs[c] = q6_12_to_f(dut->v_out);
            is_[c] = q6_12_to_f(dut->i_out);
        }
        for (int w : waypoints) {
            printf("  %-8d %+10.4f %+10.4f\n", w, vs[w], is_[w]);
        }
    }

    printf("%-32s %-10s %-8s %-8s %-12s %-12s %-12s %-12s\n",
           "tag", "kR", "P", "ton", "v_final", "i_final", "v_mean", "v_ripple_pp");

    for (auto& cfg : sweeps) {
        RunResult r = run_sim(dut, trace, Vin, cfg.kR,
                              cfg.period, cfg.on, cfg.total,
                              false, csv, cfg.tag);
        double ripple_pp = r.v_max_last - r.v_min_last;
        printf("%-32s %-10.4f %-8d %-8d %+12.4f %+12.4f %+12.4f %12.4f\n",
               cfg.tag, cfg.kR, cfg.period, cfg.on,
               r.v_final, r.i_final, r.v_mean_last, ripple_pp);
    }

    fclose(csv);
    trace->close();
    delete trace;
    delete dut;
    return 0;
}
