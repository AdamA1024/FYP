// sim_step_main.cpp - Vin step-response test for buck_verlet_mixed_precision
//
// Procedure for each step case:
//   1. Hold reset, then drive Vin = Vin_1 with fixed PWM duty for PRE_STEP
//      clocks so the plant settles to v_ss_1 = duty * Vin_1.
//   2. At cycle PRE_STEP, step Vin -> Vin_2 (duty unchanged).
//   3. Run POST_STEP more clocks and measure how long it takes the per-PWM
//      period mean of v_out to re-enter (and stay inside) a tolerance band
//      around v_ss_2 = duty * Vin_2.
//
// Settling time is reported in PWM periods and in clocks, using the
// standard "last time the response leaves the band" definition computed
// in reverse.

#include "Vbuck_verlet_mixed_precision.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

static constexpr int Q6_12_FRAC  = 12;
static constexpr int Q2_16_FRAC  = 16;
static constexpr int Q6_12_SCALE = 1 << Q6_12_FRAC;
static constexpr int Q2_16_SCALE = 1 << Q2_16_FRAC;

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

// Matches the KR_SCALE baked into verlet_pkg (see sim_main.cpp).
static constexpr double KR_SCALE = 3.0;

struct StepCfg {
    const char* tag;
    double vin1;
    double vin2;
    double kR_phys;       // physical dt/(R*C); module sees KR_SCALE*kR_phys
    int    pwm_period;
    int    pwm_on;
    int    pre_step;      // clocks held at Vin_1
    int    post_step;     // clocks after the step
    double band_frac;     // settling band, fraction of v_ss_2 (e.g. 0.02 = 2%)
};

struct StepResult {
    double v_ss1, v_ss2;        // measured steady-state means
    double v_min_post, v_max_post;
    int    settle_clocks;       // clocks after step to last band exit
    int    settle_periods;      // == settle_clocks / pwm_period (rounded up)
    bool   settled;             // false if response never enters the band
};

static StepResult run_step(Vbuck_verlet_mixed_precision* dut,
                           VerilatedVcdC* trace,
                           vluint64_t& sim_time,
                           const StepCfg& cfg,
                           FILE* csv) {
    // Reset + initial Vin
    dut->rst_n = 0;
    dut->kR    = q2_16_code(KR_SCALE * cfg.kR_phys);
    dut->Vin   = q6_12_code(cfg.vin1);
    dut->s_k   = 0;

    auto tick = [&]() {
        dut->clk = 0; dut->eval(); if (trace) trace->dump(sim_time); sim_time++;
        dut->clk = 1; dut->eval(); if (trace) trace->dump(sim_time); sim_time++;
    };
    tick(); tick();
    dut->rst_n = 1;

    const int total = cfg.pre_step + cfg.post_step;
    std::vector<double> v(total), i(total);

    for (int c = 0; c < total; ++c) {
        // Step Vin at c == pre_step (duty unchanged)
        if (c == cfg.pre_step) dut->Vin = q6_12_code(cfg.vin2);

        int phase = c % cfg.pwm_period;
        dut->s_k = (phase < cfg.pwm_on) ? 1 : 0;
        tick();
        v[c] = q6_12_to_f(dut->v_out);
        i[c] = q6_12_to_f(dut->i_out);

        if (csv) {
            fprintf(csv, "%s,%d,%d,%.6f,%.6f\n",
                    cfg.tag, c, dut->s_k, v[c], i[c]);
        }
    }

    // Per-PWM-period mean of v_out across the whole run
    const int P = cfg.pwm_period;
    const int n_periods = total / P;
    std::vector<double> vmean(n_periods);
    for (int p = 0; p < n_periods; ++p) {
        double s = 0.0;
        for (int k = 0; k < P; ++k) s += v[p*P + k];
        vmean[p] = s / P;
    }

    const int step_period = cfg.pre_step / P;
    // Steady-state references: mean of the last few periods before/after.
    auto mean_range = [&](int a, int b) {
        double s = 0; for (int p = a; p < b; ++p) s += vmean[p];
        return s / (b - a);
    };
    const int tail = std::min(20, n_periods - step_period - 1);
    double v_ss1 = mean_range(std::max(0, step_period - tail), step_period);
    double v_ss2 = mean_range(n_periods - tail, n_periods);

    const double band = cfg.band_frac * std::abs(v_ss2);

    // Settling time: walk back from the end and find the last period whose
    // mean is outside the band around v_ss2. Settle = next period boundary.
    int last_out = -1;
    for (int p = n_periods - 1; p >= step_period; --p) {
        if (std::abs(vmean[p] - v_ss2) > band) { last_out = p; break; }
    }

    StepResult r;
    r.v_ss1 = v_ss1;
    r.v_ss2 = v_ss2;
    r.v_min_post = *std::min_element(v.begin() + cfg.pre_step, v.end());
    r.v_max_post = *std::max_element(v.begin() + cfg.pre_step, v.end());
    if (last_out < 0) {
        // Never left the band after the step.
        r.settled = true;
        r.settle_periods = 0;
        r.settle_clocks  = 0;
    } else if (last_out >= n_periods - 1) {
        // Still outside the band at the end of the run.
        r.settled = false;
        r.settle_periods = n_periods - step_period;
        r.settle_clocks  = r.settle_periods * P;
    } else {
        r.settled = true;
        r.settle_periods = (last_out + 1) - step_period;
        r.settle_clocks  = r.settle_periods * P;
    }
    return r;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    auto* dut = new Vbuck_verlet_mixed_precision;
    auto* trace = new VerilatedVcdC;
    dut->trace(trace, 99);
    trace->open("step_trace.vcd");
    vluint64_t sim_time = 0;

    FILE* csv = fopen("step_trace.csv", "w");
    fprintf(csv, "tag,cycle,s,v_out,i_out\n");

    // PRE/POST sized so the slowest case (kR=0.0002, tau ~ 2500 clk = 25
    // periods) gets ~20 tau on each side.
    std::vector<StepCfg> cases = {
        // tag                       Vin1  Vin2  kR     P    on  pre    post   band
        {"step12to18 kR=0.002",      12.0, 18.0, 0.002, 100, 50, 60000, 60000, 0.02},
        {"step12to8  kR=0.002",      12.0,  8.0, 0.002, 100, 50, 60000, 60000, 0.02},
        {"step12to24 kR=0.002",      12.0, 24.0, 0.002, 100, 50, 60000, 60000, 0.02},
        {"step12to18 kR=0.010",      12.0, 18.0, 0.010, 100, 50, 20000, 20000, 0.02},
        {"step12to18 kR=0.0002",     12.0, 18.0, 0.0002,100, 50,200000,200000, 0.02},
    };

    printf("=== buck Vin-step response test ===\n");
    printf("Settling band = 2%% of v_ss2 (per-PWM-period mean).  "
           "dt = 10 ns, P=100 clk -> f_sw = 1 MHz.\n\n");
    printf("%-26s %-7s %-7s %-7s %-9s %-9s %-9s %-9s %-7s\n",
           "tag", "Vin1", "Vin2", "kR_phys",
           "v_ss1[V]", "v_ss2[V]", "ts[clk]", "ts[us]", "settled");

    for (auto& c : cases) {
        StepResult r = run_step(dut, trace, sim_time, c, csv);
        double ts_us = r.settle_clocks * 10e-3; // 10 ns/clk -> us
        printf("%-26s %-7.2f %-7.2f %-7.4f %+9.4f %+9.4f %-9d %-9.2f %-7s\n",
               c.tag, c.vin1, c.vin2, c.kR_phys,
               r.v_ss1, r.v_ss2, r.settle_clocks, ts_us,
               r.settled ? "yes" : "NO");
    }

    fclose(csv);
    trace->close();
    delete trace;
    delete dut;
    return 0;
}
