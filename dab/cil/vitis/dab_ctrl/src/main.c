// DAB digital-twin closed-loop control / characterisation.
//
// Runs a PI regulator (run_control) that trims the bridge PHASE SHIFT every tick
// to hold V2 at a reference, exercised against V1 steps (line disturbance) and
// Vref steps (reference tracking). Each tick is logged to a DDR buffer; afterwards
// the buffer is streamed out as CSV for the host to plot. UART is touched only
// AFTER capture, so printf never perturbs the sampled timing.
//
// Plant: L=20uH, Co=470uF, dt=20ns, 100 steps/period -> f_sw = 500 kHz at a
// 50 MHz core clock. Control knob is the phase shift (clocks): power transfer is
// monotonic in phase over 0..PWM_PERIOD/4 (0..90 deg), so the duty is clamped to
// that region. Gains tuned & verified in closed-loop Verilator sim.
//
// Target: standalone (bare-metal).

#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"
#include <stdint.h>

// ── Base address ─────────────────────────────────────────────────────────────
#ifdef XPAR_DAB_SLAVE_LITE_VFINAL_0_S00_AXI_BASEADDR
  #define DAB_BASE  XPAR_DAB_SLAVE_LITE_VFINAL_0_S00_AXI_BASEADDR
#else
  #define DAB_BASE  0x43C00000U
#endif

// ── Register map ──────────────────────────────────────────────────────────────
//   reg0 W V1 (Q8.24)  reg1 W gamma (Q4.28)  reg2 W phase (clocks)
//   reg3 R V2 (Q8.24)  reg4 R i_L (Q8.24)
#define R_V1       0x00
#define R_GAMMA    0x04
#define R_PHASE    0x08
#define R_V2_OUT   0x0C
#define R_IL_OUT   0x10

// ── Plant constants (must match the generated coefficient package) ───────────
#define DT_S       20e-9
#define CO_F       470e-6
#define PWM_PERIOD 100

// ── Fixed-point helpers ──────────────────────────────────────────────────────
static inline int32_t volts_to_q24(double v) { return (int32_t)(v * (double)(1 << 24)); }   // Q8.24
static inline double  q24_to_volts(int32_t q) { return (double)q / (double)(1 << 24); }
static inline int32_t gamma_q28(double g)     { return (int32_t)(g * (double)(1 << 28)); }   // Q4.28
static inline int32_t gamma_from_R(double R)  { return gamma_q28(DT_S / (R * CO_F)); }

// ── AXI access wrappers ──────────────────────────────────────────────────────
#define BARRIER()  __asm__ volatile ("dsb sy" ::: "memory")
static inline void     wr(uint32_t off, uint32_t v) { Xil_Out32(DAB_BASE + off, v); }
static inline uint32_t rd(uint32_t off)             { return Xil_In32(DAB_BASE + off); }

static inline void  set_v1(double v)       { wr(R_V1, (uint32_t)volts_to_q24(v)); }
static inline void  set_phase(uint32_t p)  { wr(R_PHASE, p); }
static inline double get_v2(void)          { return q24_to_volts((int32_t)rd(R_V2_OUT)); }

// Mean V2 over n reads spaced ~1 us apart, to smooth the (aliased) ~500 kHz ripple
// before feeding it to the controller.
static double get_v2_avg(uint32_t n) {
    double acc = 0.0;
    for (uint32_t i = 0; i < n; i++) { acc += get_v2(); usleep(1); }
    return acc / (double)n;
}

// ── Closed-loop PI phase regulator ───────────────────────────────────────────
//     e     = Vref - V2                         (volts)
//     u     = Kp*e + Ki*integ                   (phase clocks)
//     phase = clamp(round(u), 0, PHASE_MAX)
// Plant DC gain dV2/dphase ~ 0.4 V/clock near the operating point; integral
// action removes steady-state error; anti-windup freezes the integrator on
// saturation. PHASE_MAX = PWM_PERIOD/4 keeps the loop in the monotonic
// (increasing power) region of the SPS transfer.
#define N_TICKS    800
#define TICK_US    200
#define CTRL_R_OHM 10.0
#define KP         2.5      // phase-clocks per volt of error
#define KI         0.25     // phase-clocks per (volt*tick) of accumulated error
#define PHASE_MAX  (PWM_PERIOD / 4)   // = 25 clocks (90 deg)
#define VAVG_N     8        // V2 reads averaged per tick (ripple rejection)

// V1 schedule: line disturbances during the regulation phase, then fixed.
static double v1_schedule(uint32_t t) {
    if (t < 200) return 100.0;
    if (t < 400) return 120.0;   // step up   (line disturbance)
    if (t < 600) return  80.0;   // step down
    return 100.0;                // fixed for the reference-tracking phase
}
// Reference schedule: flat while V1 is disturbed, then stepped to show tracking.
static double vref_schedule(uint32_t t) {
    if (t < 600) return 12.0;
    if (t < 700) return 18.0;    // reference step up
    return 8.0;                  // reference step down
}

// ── Capture buffer (DDR) ─────────────────────────────────────────────────────
// 4 words/tick: { v1_q24, vref_q24, v2_raw, phase }. CPU-written & read -> cache
// coherent. N_TICKS*4 words; default 3200 words (12.5 KiB).
#define RES_BASE_ADDR  0x10000000U
static volatile int32_t * const RES = (volatile int32_t *)RES_BASE_ADDR;

static void run_control(void) {
    wr(R_GAMMA, (uint32_t)gamma_from_R(CTRL_R_OHM));   // fixed load
    set_phase(0);                                      // start from no transfer
    BARRIER();

    double integ = 0.0;
    uint32_t idx = 0;

    for (uint32_t t = 0; t < N_TICKS; t++) {
        double v1   = v1_schedule(t);
        double vref = vref_schedule(t);
        set_v1(v1);

        double v2 = get_v2_avg(VAVG_N);
        double e  = vref - v2;

        double u  = KP * e + KI * integ;                // PI control law
        double uc = u;
        if (uc < 0.0)                  uc = 0.0;
        if (uc > (double)PHASE_MAX)    uc = (double)PHASE_MAX;
        if (u == uc) integ += e;                        // anti-windup: hold on saturation

        uint32_t phase = (uint32_t)(uc + 0.5);
        set_phase(phase);

        RES[idx++] = volts_to_q24(v1);
        RES[idx++] = volts_to_q24(vref);
        RES[idx++] = (int32_t)rd(R_V2_OUT);
        RES[idx++] = (int32_t)phase;

        usleep(TICK_US);
    }
    set_phase(0);   // leave at no transfer
}

// ── UART dump ────────────────────────────────────────────────────────────────
static void print_q24(int32_t q) {
    int mv = (int)(((int64_t)q * 1000 + (q >= 0 ? (1 << 23) : -(1 << 23))) >> 24);
    if (mv < 0) { xil_printf("-"); mv = -mv; }
    int frac = mv % 1000;
    xil_printf("%d.", mv / 1000);
    if (frac < 100) xil_printf("0");
    if (frac < 10)  xil_printf("0");
    xil_printf("%d", frac);
}

static void dump_control(void) {
    xil_printf("\r\n# DAB_CTRL_BEGIN\r\n");
    xil_printf("# pi: N=%d, tick=%d us, R=%d ohm, Kp_milli=%d, Ki_milli=%d, phase_max=%d\r\n",
               N_TICKS, TICK_US, (int)(CTRL_R_OHM + 0.5),
               (int)(KP * 1000 + 0.5), (int)(KI * 1000 + 0.5), PHASE_MAX);
    xil_printf("tick,t_us,v1,vref,v2,phase\r\n");
    usleep(100000);   // let the host terminal settle before the bulk burst

    uint32_t idx = 0;
    for (uint32_t t = 0; t < N_TICKS; t++) {
        int32_t v1    = RES[idx++];
        int32_t vref  = RES[idx++];
        int32_t v2    = RES[idx++];
        int32_t phase = RES[idx++];
        xil_printf("%u,%u,", t, t * TICK_US);
        print_q24(v1);   xil_printf(",");
        print_q24(vref); xil_printf(",");
        print_q24(v2);   xil_printf(",");
        xil_printf("%d\r\n", phase);
        usleep(200);    // pace the stream so the host UART can't overrun
    }
    xil_printf("# DAB_CTRL_END\r\n");
}

int main(void) {
    xil_printf("DAB PI phase regulator: running %d ticks...\r\n", N_TICKS);
    run_control();
    dump_control();
    xil_printf("control done.\r\n");
    while (1) { ; }
}
