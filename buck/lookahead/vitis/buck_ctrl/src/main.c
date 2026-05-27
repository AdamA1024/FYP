// Buck digital-twin closed-loop control / characterisation for buck_slave_lite_vFinal.
//
// Drives the buck look-ahead mixed-precision Verlet twin over AXI4-Lite. main() runs a
// closed-loop PI regulator (run_control) that trims PWM duty every tick to hold
// Vout at a reference, exercised against Vin steps (line disturbance) and Vref
// steps (reference tracking). Each tick is logged to a DDR buffer; afterwards
// the buffer is streamed out as CSV for the host to plot.
//
// Also included (not run by default) is capture_grid/dump_csv: a fixed-duty 2D
// Vin x R sweep that captures the open-loop Vout/Iout transient at each point.
// In both cases UART is touched only AFTER capture, so printf never perturbs the
// sampled timing.
//
// Plant: L=2.25uH, C=1.125uF, dt=10ns, 100 steps/period -> f_sw = 1 MHz at a
// 100 MHz core clock. Must match the generated verlet_pkg coefficient package.
//
// NOTE on "reset": this slave has NO software reset bit. To re-arm between
// operating points we set duty=0 (s_k stays low, so Vin is gated out) and wait —
// the LC then decays to zero through the kR damping term (RC ~= 11 us), a clean
// soft reset.
//
// Target: standalone (bare-metal), Zynq-7000/A9 (PYNQ) or ZynqMP/A53 (AXU5).

#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"
#include <stdint.h>

// ── Base address ─────────────────────────────────────────────────────────────
#ifdef XPAR_BUCK_SLAVE_LITE_VFINAL_0_S00_AXI_BASEADDR
  #define BUCK_BASE  XPAR_BUCK_SLAVE_LITE_VFINAL_0_S00_AXI_BASEADDR
#else
  #define BUCK_BASE  0x43C00000U
#endif

// ── Register map (ADDR_LSB = 2, so offset = reg_index * 4) ────────────────────
//   reg0  W  Vin    Q6.12  input voltage         (low 18 bits)
//   reg1  W  kR     Q2.16  KR_SCALE*dt/(R*C)     (low 18 bits)
//   reg2  W  duty   int    PWM duty count, 0..PWM_PERIOD(=100)
//   reg3  R  Vout   Q6.12  output voltage        (18-bit sign-extended)
//   reg4  R  Iout   Q6.12  inductor current      (18-bit sign-extended)
#define R_VIN      0x00
#define R_KR       0x04
#define R_DUTY     0x08
#define R_VOUT     0x0C
#define R_IOUT     0x10

// ── Plant constants (must match the generated coefficient package) ───────────
#define DT_S        10e-9
#define C_F         1.125e-6
#define KR_SCALE    3        // S factor baked into the package (twin_gen --kr-scale)
#define PWM_PERIOD  100

// ── Fixed-point helpers ──────────────────────────────────────────────────────
// Q6.12: 6 signed integer bits, 12 fractional. Range approx +-32.
static inline int32_t volts_to_q12(double v)  { return (int32_t)(v * (double)(1 << 12)); }

// Q2.16: 2 signed integer bits, 16 fractional. kR is small, well inside.
static inline int32_t kR_q16(double k)        { return (int32_t)(k * (double)(1 << 16)); }

// Load resistance -> kR = KR_SCALE*dt/(R*C), in Q2.16.
static inline int32_t kR_from_R(double R_ohm) {
    return kR_q16((double)KR_SCALE * DT_S / (R_ohm * C_F));
}

// Duty percent -> count, clamped to [0, PWM_PERIOD].
static inline uint32_t duty_pct_to_count(double pct) {
    int d = (int)(pct / 100.0 * PWM_PERIOD + 0.5);
    if (d < 0)          d = 0;
    if (d > PWM_PERIOD) d = PWM_PERIOD;
    return (uint32_t)d;
}

// ── AXI access wrappers ──────────────────────────────────────────────────────
#define BARRIER()  __asm__ volatile ("dsb sy" ::: "memory")

static inline void     wr(uint32_t off, uint32_t val) { Xil_Out32(BUCK_BASE + off, val); }
static inline uint32_t rd(uint32_t off)               { return Xil_In32(BUCK_BASE + off); }

static inline void  set_vin(double v)    { wr(R_VIN, (uint32_t)(volts_to_q12(v) & 0x3FFFF)); }
static inline void  set_kR(double R)     { wr(R_KR,  (uint32_t)(kR_from_R(R)   & 0x3FFFF)); }
static inline void  set_duty(uint32_t d) { wr(R_DUTY, d); }

// Q6.12 is 18-bit sign-extended in reg3; rd() gives the raw 32-bit word.
static inline double q12_to_volts(int32_t q) { return (double)q / (double)(1 << 12); }
static inline double get_vout(void)          { return q12_to_volts((int32_t)rd(R_VOUT)); }

// Mean Vout over n reads spaced ~1 us apart, to average out the ~1 MHz
// switching ripple before feeding it to the controller (a single read is
// dominated by ripple; n>=8 over >=8 us gives a clean DC estimate).
static double get_vout_avg(uint32_t n) {
    double acc = 0.0;
    for (uint32_t i = 0; i < n; i++) { acc += get_vout(); usleep(1); }
    return acc / (double)n;
}

// Park the plant at rest: gate the input off and let the LC decay through kR.
// ~1 ms is ~90 RC time constants — a clean soft reset.
static void park_and_reset(void) {
    set_duty(0);
    BARRIER();
    usleep(1000);
}

// ── Sweep configuration ──────────────────────────────────────────────────────
// The grid: every Vin x every load R. Edit these arrays to retarget the sweep.
static const double VINS[] = { 6.0, 9.0, 12.0, 15.0, 18.0 };
static const double RS[]   = { 5.0, 10.0, 15.0, 20.0 };
#define N_VIN   (sizeof(VINS)/sizeof(VINS[0]))
#define N_R     (sizeof(RS)/sizeof(RS[0]))
#define N_PT    (N_VIN * N_R)

#define N_SAMP    256       // samples of the transient per operating point
#define SAMP_US   2         // sampling period -> window = N_SAMP*SAMP_US us
#define DUTY_PCT  50.0      // fixed PWM duty for the whole sweep

// ── Capture buffer (DDR) ─────────────────────────────────────────────────────
// Flat int32 stream, two words per sample (raw Q6.12 vout, iout). We write it
// with the CPU and read it back with the CPU, so it stays cache-coherent — no
// flush needed. Sized for the full grid.
#define RES_BASE_ADDR  0x10000000U
static volatile int32_t * const RES = (volatile int32_t *)RES_BASE_ADDR;
// N_PT * N_SAMP * 2 words must fit; for the default grid that is 10240 words.

// Capture every operating point into RES. No UART here — pure timing-critical
// sampling. Returns nothing; layout is implicit (point-major, then sample).
static void capture_grid(void) {
    uint32_t duty = duty_pct_to_count(DUTY_PCT);
    uint32_t idx  = 0;

    for (uint32_t iv = 0; iv < N_VIN; iv++) {
        for (uint32_t ir = 0; ir < N_R; ir++) {
            park_and_reset();
            set_kR(RS[ir]);
            set_vin(VINS[iv]);
            set_duty(duty);              // converter starts pumping
            BARRIER();

            for (uint32_t k = 0; k < N_SAMP; k++) {
                RES[idx++] = (int32_t)rd(R_VOUT);
                RES[idx++] = (int32_t)rd(R_IOUT);
                usleep(SAMP_US);
            }
        }
    }
    set_duty(0);   // leave the plant parked
}

// ── UART dump ────────────────────────────────────────────────────────────────
// xil_printf has no %f, so print a Q6.12 raw value as a signed decimal with
// three fractional digits (millis), without relying on width specifiers.
static void print_q12(int32_t q) {
    int mv = (q * 1000 + (q >= 0 ? 2048 : -2048)) / 4096;   // rounded millis
    if (mv < 0) { xil_printf("-"); mv = -mv; }
    int frac = mv % 1000;
    xil_printf("%d.", mv / 1000);
    if (frac < 100) xil_printf("0");
    if (frac < 10)  xil_printf("0");
    xil_printf("%d", frac);
}

// Stream the captured grid as CSV. One row per sample; the host can pivot on
// (vin,R) to get a curve per operating point.
static void dump_csv(void) {
    xil_printf("\r\n# BUCK_SWEEP_BEGIN\r\n");
    xil_printf("# grid: %d Vin x %d R = %d points, %d samples @ %d us, duty %d%%\r\n",
               (int)N_VIN, (int)N_R, (int)N_PT, N_SAMP, SAMP_US, (int)(DUTY_PCT + 0.5));
    xil_printf("point,vin,R,sample,t_us,vout,iout\r\n");
    usleep(100000);   // let the host terminal settle before the bulk burst

    uint32_t idx = 0, pt = 0;
    for (uint32_t iv = 0; iv < N_VIN; iv++) {
        for (uint32_t ir = 0; ir < N_R; ir++) {
            for (uint32_t k = 0; k < N_SAMP; k++) {
                int32_t vout = RES[idx++];
                int32_t iout = RES[idx++];
                // point,vin,R,sample,t_us,
                xil_printf("%u,", pt);
                print_q12(volts_to_q12(VINS[iv])); xil_printf(",");
                print_q12(volts_to_q12(RS[ir]));   xil_printf(",");
                xil_printf("%u,%u,", k, k * SAMP_US);
                // vout,iout
                print_q12(vout); xil_printf(",");
                print_q12(iout); xil_printf("\r\n");
                // Pace the stream so the host UART/terminal can't overrun: at
                // 115200 baud a ~30-char row is ~2.6 ms of wire time, but the
                // host RX buffer can still drop on a sustained burst. A short
                // gap per row gives it margin (cost: ~N_PT*N_SAMP*delay total).
                usleep(200);
            }
            pt++;
        }
    }
    xil_printf("# BUCK_SWEEP_END\r\n");
}

// ── Closed-loop PI voltage regulator ─────────────────────────────────────────
// Instead of a fixed duty, trim PWM duty every tick to hold Vout at Vref:
//
//     e   = Vref - Vout                       (volts)
//     u   = Kp*e + Ki*integ                   (duty counts)
//     duty= clamp(round(u), 0, PWM_PERIOD)
//
// The plant DC gain is dVout/dduty ~= Vin/PWM_PERIOD, so the gains are in
// duty-counts per volt. Integral action removes steady-state error; anti-windup
// freezes the integrator whenever the duty command saturates. The loop is
// exercised two ways so the capture shows it actually regulating:
//   - LINE rejection : Vref held while Vin is stepped underneath it.
//   - REFERENCE track: Vin held while Vref is stepped.
// Log layout in RES: 4 words/tick { vin_q12, vref_q12, vout_raw, duty }.
#define N_TICKS    400
#define TICK_US    100
#define CTRL_R_OHM 10.0
#define KP         4.0      // duty-counts per volt of error
#define KI         1.5      // duty-counts per (volt*tick) of accumulated error
                            // (the integrator supplies the operating-point duty,
                            //  e.g. ~50 counts for 6 V at 12 V in, so it must
                            //  wind up in ~tens of ticks, not hundreds)
#define VAVG_N     8        // Vout reads averaged per tick (ripple rejection)

// Vin schedule: line disturbances during the regulation phase, then fixed.
static double vin_schedule(uint32_t t) {
    if (t < 100) return 12.0;
    if (t < 200) return 16.0;   // step up  (line disturbance)
    if (t < 300) return  8.0;   // step down
    return 12.0;                 // fixed for the reference-tracking phase
}
// Reference schedule: flat while Vin is disturbed, then stepped to show tracking.
static double vref_schedule(uint32_t t) {
    if (t < 300) return 6.0;
    if (t < 350) return 9.0;    // reference step up
    return 4.0;                 // reference step down
}

static void run_control(void) {
    park_and_reset();
    set_kR(CTRL_R_OHM);

    double integ = 0.0;
    uint32_t idx = 0;

    for (uint32_t t = 0; t < N_TICKS; t++) {
        double vin  = vin_schedule(t);
        double vref = vref_schedule(t);
        set_vin(vin);

        double vout = get_vout_avg(VAVG_N);
        double e    = vref - vout;

        double u = KP * e + KI * integ;                 // PI control law
        double uc = u;
        if (uc < 0.0)                 uc = 0.0;
        if (uc > (double)PWM_PERIOD)  uc = (double)PWM_PERIOD;
        if (u == uc) integ += e;                        // anti-windup: hold on saturation

        uint32_t duty = (uint32_t)(uc + 0.5);
        set_duty(duty);

        RES[idx++] = volts_to_q12(vin);
        RES[idx++] = volts_to_q12(vref);
        RES[idx++] = (int32_t)rd(R_VOUT);
        RES[idx++] = (int32_t)duty;

        usleep(TICK_US);
    }
    set_duty(0);
}

static void dump_control(void) {
    xil_printf("\r\n# BUCK_CTRL_BEGIN\r\n");
    xil_printf("# pi: N=%d, tick=%d us, R=%d ohm, Kp_milli=%d, Ki_milli=%d\r\n",
               N_TICKS, TICK_US, (int)(CTRL_R_OHM + 0.5),
               (int)(KP * 1000 + 0.5), (int)(KI * 1000 + 0.5));
    xil_printf("tick,t_us,vin,vref,vout,duty\r\n");
    usleep(100000);   // let the host terminal settle before the bulk burst

    uint32_t idx = 0;
    for (uint32_t t = 0; t < N_TICKS; t++) {
        int32_t vin  = RES[idx++];
        int32_t vref = RES[idx++];
        int32_t vout = RES[idx++];
        int32_t duty = RES[idx++];
        xil_printf("%u,%u,", t, t * TICK_US);
        print_q12(vin);  xil_printf(",");
        print_q12(vref); xil_printf(",");
        print_q12(vout); xil_printf(",");
        xil_printf("%d\r\n", duty);
        usleep(200);    // pace the stream so the host UART can't overrun
    }
    xil_printf("# BUCK_CTRL_END\r\n");
}

int main(void) {
    park_and_reset();

    xil_printf("buck PI voltage regulator: running %d ticks...\r\n", N_TICKS);
    run_control();
    dump_control();

    // Fixed-duty Vin x R sweep is still available; not run in this build.
    (void)capture_grid; (void)dump_csv;

    xil_printf("control done.\r\n");
    while (1) { ; }
}
