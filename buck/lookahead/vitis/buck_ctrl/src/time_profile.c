// Buck digital-twin bring-up / profiling / control for buck_slave_lite_vFinal.
//
// Drives the buck look-ahead mixed-precision Verlet twin over AXI4-Lite and runs three
// back-to-back experiments, logging all results to a fixed DDR buffer (no
// printf for bulk data — the host pulls the buffer back with XSDB `mrd`):
//
//   1. AXI timing profile  — latency of a read, a (posted) write, and a
//      write+read round-trip, averaged over many iterations.
//   2. Setpoint sweep      — for a list of Vin values: park the plant at rest,
//      apply the new Vin + duty, and capture the Vout/Iout transient.
//   3. Closed-loop control — a PI regulator adjusts PWM duty to hold Vout at a
//      reference while Vin is stepped underneath it (line disturbance).
//
// Plant: L=2.25uH, C=1.125uF, dt=10ns, 100 steps/period -> f_sw = 1 MHz at a
// 100 MHz core clock. Must match the generated verlet_pkg coefficient package.
//
// NOTE on "reset": this slave has NO software reset bit. The twin clears only on
// the AXI bus reset (S_AXI_ARESETN) at boot. To re-arm between setpoints we set
// duty=0 (s_k stays low, so Vin is gated out) and wait — the LC then decays to
// zero through the kR damping term (RC ~= 11 us), which is a clean soft reset.
//
// Target: standalone (bare-metal). Builds on both Zynq-7000/Cortex-A9 (PYNQ)
// and ZynqMP/Cortex-A53 (AXU5) — the tick source is selected per-arch below.
// Build in Vitis against the platform exported from the Vivado project
// containing buck_slave_lite_vFinal.

#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "sleep.h"
#include <stdint.h>

// ── Portable high-resolution tick source ─────────────────────────────────────
// The ZynqMP/A53 BSP ships xtime_l.h, but the Zynq-7000/A9 BSP does not — it
// exposes the Cortex-A9 PMU cycle counter instead (xpm_counter.h). We use the
// PMU cycle counter on AArch32 (CP15, ~1.5 ns/tick at 650 MHz) and fall back to
// xtime_l.h on AArch64, behind one abstraction: tick_init / tick_now / TICK_HZ.
#if defined(__aarch64__)
  #include "xtime_l.h"
  typedef XTime tick_t;
  static inline void  tick_init(void) { }
  static inline tick_t tick_now(void) { XTime t; XTime_GetTime(&t); return t; }
  #define TICK_HZ  ((double)COUNTS_PER_SECOND)
#else
  // Cortex-A9 (ARMv7) — CP15 performance-monitor cycle counter (CCNT, 32-bit).
  typedef uint32_t tick_t;
  static inline void tick_init(void) {
      // PMCR: E (enable, bit0) | C (reset cycle counter, bit2); D=0 -> count every cycle.
      __asm__ volatile ("mcr p15, 0, %0, c9, c12, 0" :: "r"(0x5u));
      // CNTENS: enable the cycle counter (bit31).
      __asm__ volatile ("mcr p15, 0, %0, c9, c12, 1" :: "r"(0x80000000u));
  }
  static inline tick_t tick_now(void) {
      uint32_t v; __asm__ volatile ("mrc p15, 0, %0, c9, c13, 0" : "=r"(v)); return v;
  }
  #if defined(XPAR_CPU_CORE_CLOCK_FREQ_HZ)
    #define TICK_HZ  ((double)XPAR_CPU_CORE_CLOCK_FREQ_HZ)
  #elif defined(XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ)
    #define TICK_HZ  ((double)XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ)
  #else
    #define TICK_HZ  650000000.0   // Zynq-7000 A9 default; edit to your PS clock
  #endif
#endif

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
static inline double  q12_to_volts(int32_t q) { return (double)q / (double)(1 << 12); }

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
static inline double get_vout(void)      { return q12_to_volts((int32_t)rd(R_VOUT)); }
static inline double get_iout(void)      { return q12_to_volts((int32_t)rd(R_IOUT)); }

// Park the plant at rest: gate the input off and let the LC decay through kR.
// ~1 ms is ~90 RC time constants — a clean soft reset.
static void park_and_reset(void) {
    set_duty(0);
    BARRIER();
    usleep(1000);
}

// ── DDR results buffer ───────────────────────────────────────────────────────
// All experiments write here as a flat int32 stream; the host reads it back with
// XSDB `mrd` (see dump_results.tcl) and parse_results.py decodes the layout.
//
//   word 0x0000  HDR_MAGIC          0xBC0FFEE2 (set last; 0 while filling)
//   word 0x0001  schema version     (=2)
//   word 0x0002  TICK_HZ            low 32  (cycle/timestamp counter frequency)
//   word 0x0003  TICK_HZ            high 32
//   word 0x0004  DONE_MAGIC slot    (set last; 0 until every section is valid)
//
//   word 0x0040  PROFILE section    (see run_profile)
//   word 0x0400  SWEEP   section    (see run_sweep)
//   word 0x4000  CONTROL section    (see run_control)
#define RES_BASE_ADDR   0x10000000U
#define RES_DUMP_WORDS  0x5400U          // 0x15000 bytes — covers all sections
                                         // (control ends at word 0x4000+8+N_TICKS*4)

static volatile int32_t * const RES = (volatile int32_t *)RES_BASE_ADDR;

#define W_HDR_MAGIC   0x0000
#define W_VERSION     0x0001
#define W_CPS_LO      0x0002
#define W_CPS_HI      0x0003
#define W_DONE        0x0004

#define HDR_MAGIC     0xBC0FFEE2
#define DONE_MAGIC    0xD09ED09E
#define SCHEMA_VER    2

#define W_PROF        0x0040
#define W_SWEEP       0x0400
#define W_CTRL        0x4000

#define PROF_MAGIC    0x50524F46   // "PROF"
#define SWEEP_MAGIC   0x53574550   // "SWEP"
#define CTRL_MAGIC    0x4354524C   // "CTRL"

static inline void put(uint32_t word, int32_t val) { RES[word] = val; }

// ── 1. AXI timing profile ────────────────────────────────────────────────────
// Latency is below the 10 ns timestamp tick, so we time N transactions and
// divide. Three figures, reported in picoseconds:
//   - read  : Xil_In32 stalls until RDATA returns -> true read latency.
//   - write : Xil_Out32 is a posted store; the CPU may not wait for BVALID, so
//             this is the CPU-visible cost, not the on-bus completion time.
//   - rtrip : write immediately followed by a read of the same slave — the read
//             forces the prior write to drain, i.e. a completed write+read pair.
// The A9 PMU cycle counter resolves ~1.5 ns, so we time ONE transaction at a
// time (not a batch average) and report the MIN over many reps — the minimum is
// the cleanest estimate of intrinsic latency, least perturbed by interrupts or
// cache refills. We subtract the back-to-back tick_now() instrumentation cost
// measured on the same core, and print everything to UART.
#define REPS 4000

static uint32_t ins(uint32_t ticks) {            // ticks -> ns (rounded), for xil_printf
    return (uint32_t)((double)ticks * 1e9 / TICK_HZ + 0.5);
}
static uint32_t sub(uint32_t v, uint32_t o) { return v > o ? v - o : 0; }

// Min tick cost of the timing harness alone (two reads, nothing between them).
static uint32_t probe_overhead(void) {
    uint32_t mn = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < REPS; i++) {
        uint32_t a = tick_now();
        uint32_t b = tick_now();
        uint32_t d = (uint32_t)(b - a);
        if (d < mn) mn = d;
    }
    return mn;
}

// Min ticks for a single read of `off` (instrumentation overhead removed).
static uint32_t time_read(uint32_t off, uint32_t ovh) {
    volatile uint32_t s = 0; uint32_t mn = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < REPS; i++) {
        uint32_t a = tick_now();
        s ^= rd(off);
        uint32_t d = (uint32_t)(tick_now() - a);
        if (d < mn) mn = d;
    }
    (void)s; return sub(mn, ovh);
}

// Min ticks for a single write to `off`. `drain` adds a dsb so the write fully
// completes to the slave (vs. the bare posted-store cost the CPU sees).
static uint32_t time_write(uint32_t off, uint32_t val, int drain, uint32_t ovh) {
    uint32_t mn = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < REPS; i++) {
        uint32_t a = tick_now();
        wr(off, val);
        if (drain) BARRIER();
        uint32_t d = (uint32_t)(tick_now() - a);
        if (d < mn) mn = d;
    }
    return sub(mn, ovh);
}

static void run_profile(void) {
    uint32_t ovh = probe_overhead();
    uint32_t vin = (uint32_t)(volts_to_q12(12.0) & 0x3FFFF);
    uint32_t kr  = (uint32_t)(kR_from_R(10.0)    & 0x3FFFF);

    xil_printf("\r\n== AXI4-Lite latency (min of %d reps; harness overhead %u ns) ==\r\n",
               REPS, ins(ovh));

    // --- single reads ---
    xil_printf("  READ  Vout (reg3): %u ns\r\n", ins(time_read(R_VOUT, ovh)));
    xil_printf("  READ  Iout (reg4): %u ns\r\n", ins(time_read(R_IOUT, ovh)));
    xil_printf("  READ  Vin  (reg0): %u ns\r\n", ins(time_read(R_VIN,  ovh)));

    // --- single writes (posted = CPU-visible; drained = completed to slave) ---
    xil_printf("  WRITE Vin  (reg0): %u ns posted / %u ns drained\r\n",
               ins(time_write(R_VIN, vin, 0, ovh)), ins(time_write(R_VIN, vin, 1, ovh)));
    xil_printf("  WRITE kR   (reg1): %u ns drained\r\n", ins(time_write(R_KR,   kr, 1, ovh)));
    xil_printf("  WRITE duty (reg2): %u ns drained\r\n", ins(time_write(R_DUTY, 50, 1, ovh)));

    // --- write+read round-trip (write reg0, then read reg3 back) ---
    uint32_t mn = 0xFFFFFFFFu; volatile uint32_t s = 0;
    for (uint32_t i = 0; i < REPS; i++) {
        uint32_t a = tick_now();
        wr(R_VIN, vin); s ^= rd(R_VOUT);
        uint32_t d = (uint32_t)(tick_now() - a);
        if (d < mn) mn = d;
    }
    (void)s;
    xil_printf("  RTRIP write+read : %u ns\r\n", ins(sub(mn, ovh)));

    set_duty(0);   // leave the plant parked
}

// ── 2. Setpoint sweep ────────────────────────────────────────────────────────
// For each Vin: reset, apply Vin+duty, then sample the Vout/Iout transient.
// Layout at W_SWEEP:
//   +0 SWEEP_MAGIC   +1 n_set   +2 n_samp   +3 samp_us   +4 duty_count
//   +5 kR_raw        +6 load_ohm_x1000
//   +8 data: per setpoint { vin_q12, then n_samp * (vout_raw, iout_raw) }
static const double SWEEP_VINS[] = { 4.0, 6.0, 8.0, 10.0, 12.0 };
#define N_SET    (sizeof(SWEEP_VINS)/sizeof(SWEEP_VINS[0]))
#define N_SAMP   512
#define SAMP_US  2
#define SWEEP_DUTY_PCT 50.0
#define SWEEP_LOAD_OHM 10.0

static void run_sweep(void) {
    uint32_t duty = duty_pct_to_count(SWEEP_DUTY_PCT);
    uint32_t base = W_SWEEP;

    put(base + 0, SWEEP_MAGIC);
    put(base + 1, (int32_t)N_SET);
    put(base + 2, N_SAMP);
    put(base + 3, SAMP_US);
    put(base + 4, (int32_t)duty);
    put(base + 5, kR_from_R(SWEEP_LOAD_OHM));
    put(base + 6, (int32_t)(SWEEP_LOAD_OHM * 1000.0));

    uint32_t w = base + 8;
    set_kR(SWEEP_LOAD_OHM);

    for (uint32_t s = 0; s < N_SET; s++) {
        park_and_reset();
        set_vin(SWEEP_VINS[s]);
        put(w++, volts_to_q12(SWEEP_VINS[s]));

        set_duty(duty);           // converter starts pumping
        BARRIER();
        for (uint32_t k = 0; k < N_SAMP; k++) {
            put(w++, (int32_t)rd(R_VOUT));
            put(w++, (int32_t)rd(R_IOUT));
            usleep(SAMP_US);
        }
    }
}

// ── 3. Closed-loop PI controller ─────────────────────────────────────────────
// Hold Vout at VREF by trimming PWM duty; step Vin underneath to disturb it.
// The plant is quasi-static at the control rate (LC settles in ~tens of us <<
// tick), so a simple PI on duty is well behaved. Anti-windup: freeze the
// integrator when the duty command saturates.
// Layout at W_CTRL:
//   +0 CTRL_MAGIC  +1 n_ticks  +2 tick_us  +3 vref_q12  +4 fields_per_tick(=4)
//   +8 data: per tick { vin_q12, vref_q12, vout_raw, duty_count }
#define N_TICKS  1200
#define TICK_US  50
#define VREF_V   6.0
#define CTRL_LOAD_OHM 10.0
#define KP       4.0       // duty-counts per volt of error
#define KI       0.08      // duty-counts per (volt*tick) of accumulated error

// Vin schedule: piecewise-constant to create line disturbances the loop rejects.
static double vin_schedule(uint32_t tick) {
    if (tick < 300)  return 12.0;
    if (tick < 600)  return 8.0;
    if (tick < 900)  return 16.0;
    return 10.0;
}

static void run_control(void) {
    uint32_t base   = W_CTRL;
    int32_t  vref_q = volts_to_q12(VREF_V);

    put(base + 0, CTRL_MAGIC);
    put(base + 1, N_TICKS);
    put(base + 2, TICK_US);
    put(base + 3, vref_q);
    put(base + 4, 4);

    park_and_reset();
    set_kR(CTRL_LOAD_OHM);
    set_vin(vin_schedule(0));

    double integ = 0.0;
    uint32_t w = base + 8;

    for (uint32_t t = 0; t < N_TICKS; t++) {
        double vin = vin_schedule(t);
        set_vin(vin);

        double vout = get_vout();
        double e    = VREF_V - vout;

        double u = KP * e + KI * integ;                 // PI control law
        double u_clamped = u;
        if (u_clamped < 0.0)                u_clamped = 0.0;
        if (u_clamped > (double)PWM_PERIOD) u_clamped = (double)PWM_PERIOD;
        if (u == u_clamped) integ += e;                 // anti-windup

        uint32_t duty = (uint32_t)(u_clamped + 0.5);
        set_duty(duty);

        put(w++, volts_to_q12(vin));
        put(w++, vref_q);
        put(w++, (int32_t)rd(R_VOUT));
        put(w++, (int32_t)duty);

        usleep(TICK_US);
    }
}

int main(void) {
    tick_init();
    park_and_reset();

    run_profile();

    // Defined for later experiments (DDR-logged); not run in this profiling-only
    // build. The (void) refs keep them from tripping -Wunused-function.
    (void)run_sweep; (void)run_control;

    xil_printf("profile done.\r\n");
    while (1) { ; }
}
