// dab_rtl.sv  -  Dual Active Bridge (DAB) DC-DC converter
//                Fixed-point Velocity-Verlet RTL
//
// Velocity-Verlet integration (one timestep dt per clock cycle):
//
//   alpha  = dt/(2L),      beta = dt/Co,   gamma = dt/(RCo),   delta = R_L*dt/(2L)
//
//   i_L[k+1/2]  = i_L[k]*(1-delta)  + alpha*(p1*V1 - N*p2*V2[k])    <- R_L half-damp
//   V2_nd     = V2[k]          + beta*N*p2*i_L[k+1/2]             (no output damping)
//   i_L_ud    = i_L[k+1/2]       + alpha*(p1*V1 - N*p2*V2_nd)     (uses undamped V2)
//   i_L[k+1]  = i_L_ud*(1-delta)                                 <- R_L half-damp
//   V2[k+1]   = V2_nd          - gamma*V2[k]                     (output dissipation)
//
// Two-fix design:
//  1. R_L damping (delta): models inductor winding resistance.  Without it, i_L
//     accumulates a slowly-growing DC offset (no transformer DC-block in this
//     model), which makes the V2 peak-to-peak ripple grow over time while the
//     mean stays correct.  delta applied symmetrically in both half-steps keeps the
//     integrator second-order accurate.
//  2. V2_nd in step 3: separates the output R*Co dissipation from the symplectic
//     step, preventing gamma*V2[k] from adding a spurious force contribution to i_L.
//
// All arithmetic is fixed-point Q(WIDTH-FRAC-1).(FRAC) signed.
// All constants are pre-scaled by 2^FRAC.
//
// Latency:  one clock cycle (no pipelining).
// The combinational datapath computes all three half-steps in parallel
// where data-dependencies allow:
//   - p1*V1 and N*V2[k] are computed simultaneously (both read-only inputs).
//   - fpmul(GAMMA, V2_reg) overlaps with step-1 multiplier.
//   - Steps 2 and 3 are serialised by their data dependency on i_L_half /
//     V2_new respectively - this is fundamental to the algorithm.

module dab_rtl #(
    parameter int WIDTH   = 32,           // fixed-point word width (bits)
    parameter int FRAC    = 20,           // fractional bits
    //
    // Default operating point
    //   dt =  10 ns,  fsw = 1 MHz   (HALF_PERIOD = 50 timesteps, PERIOD = 100)
    //   V1 = 400 V,   N = 2,  V2_nom = 200 V
    //   L  =  20 uH  (transformer leakage inductance)
    //   Co = 100 uF,  R = 10 ohm  (output filter / load)
    //
    // All constants are stored as round(real_value x 2^FRAC).
    //
    // alpha = dt/(2L) = 10e-9/(2x20e-6)  = 2.500e-4  ->  round(2.5e-4 x 2^20) = 262
    parameter logic signed [WIDTH-1:0] ALPHA    = 32'sd262,

    // beta = dt/Co: 10e-9/100e-6 = 1e-4 -> round(1e-4 x 2^20) = 105 (105/2^20 = 1.001e-4).
    parameter logic signed [WIDTH-1:0] BETA     = 32'sd105,

    // gamma = dt/(R*Co)= 10e-9/(10x100e-6) = 1.000e-5  ->  round(1e-5 x 2^20) = 10.
    // With dt=10ns, gamma is only ~10 LSBs and rounds with ~4.6% error.
    // GAMMA/BETA = 10/105 = 0.0952 (vs ideal 0.1) -> R_effective ~ 10.5 ohm.
    // V2 steady state is shifted up by sqrt(1.05) ~ 2.5%.  Bump FRAC to 21
    // if exact R is needed.
    parameter logic signed [WIDTH-1:0] GAMMA    = 32'sd10,

    // delta = R_L*dt/(2L)  (inductor series resistance, applied as half-step each side)
    //   R_L ~ 0.1 ohm  ->  delta = 0.1x10e-9/(2x20e-6) = 2.500e-5  ->  round(x2^20) = 26
    //   This damps the i_L DC component (tau ~ L/R_L = 200 us) and prevents the
    //   growing-ripple instability caused by the lack of transformer DC blocking.
    parameter logic signed [WIDTH-1:0] DELTA_L  = 32'sd26,

    // N = 2  ->  2 x 2^FRAC = 2 097 152
    parameter logic signed [WIDTH-1:0] N_RATIO  = 32'sd2097152,

    // Reset / initial conditions
    //   Cold-start initial conditions: both set to 0.
    parameter logic signed [WIDTH-1:0] V2_INIT  = 32'sd0,
    parameter logic signed [WIDTH-1:0] IL_INIT  = 32'sd0
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // Primary bus voltage (fixed-point, same Q format as outputs)
    input  logic signed [WIDTH-1:0] V1,

    // Bridge drive polarity:  1 -> phase = +1,  0 -> phase = -1
    input  logic                    p1,
    input  logic                    p2,

    output logic signed [WIDTH-1:0] i_L_out,   // inductor current  [A * 2^FRAC]
    output logic signed [WIDTH-1:0] V2_out     // output voltage    [V * 2^FRAC]
);

    // State registers
    logic signed [WIDTH-1:0] i_L_reg, V2_reg;

    // Fixed-point multiply:  fpmul(a, b) = round_nearest(a * b * 2^-FRAC)
    //
    // Written in the DSP48E1 idiom:  P = A*B + C  followed by a bit-slice on P.
    // Vivado recognises this exact pattern and packs the multiply + rounding-add
    // into a single DSP slice, with the bit-slice as free output truncation.
    // (The previous form - `(prod + round) >>> FRAC` then `WIDTH'(...)` - was
    // mathematically identical but Vivado refused to infer a DSP from it.)
    //
    // Round-half-up (add 1/2 LSB before truncation) instead of plain truncation,
    // which always rounds toward zero: for always-positive quantities like V2,
    // fpmul(GAMMA, V2) would be consistently ~1/2 LSB too small every cycle,
    // leaving a tiny net positive power imbalance that manifests as a slow
    // upward drift in V2 steady state.  Rounding removes that systematic bias.
    function automatic logic signed [WIDTH-1:0] fpmul (
        input logic signed [WIDTH-1:0] a,
        input logic signed [WIDTH-1:0] b
    );
        /* verilator lint_off WIDTHEXPAND */
        logic signed [2*WIDTH-1:0] prod;
        /* verilator lint_on  WIDTHEXPAND */
        // DSP48 multiply-add: A*B + C, with C = 1/2 LSB for round-half-up.
        prod = a * b + (2*WIDTH)'(1 << (FRAC-1));
        // Bit-slice [WIDTH+FRAC-1 -: WIDTH] = Q(WIDTH-FRAC-1).FRAC result.
        return prod[WIDTH+FRAC-1 -: WIDTH];
    endfunction

    // Combinational datapath  - all three Verlet sub-steps in one cycle

    // Intermediate wires
    logic signed [WIDTH-1:0] p1_V1;         // p1 * V1
    logic signed [WIDTH-1:0] n_v2k;         // N * V2[k]          (unsigned sense)
    logic signed [WIDTH-1:0] N_p2_V2k;      // N * p2 * V2[k]

    logic signed [WIDTH-1:0] i_L_half;      // i_L[k+1/2]

    logic signed [WIDTH-1:0] n_ilh;         // N * i_L[k+1/2]       (unsigned sense)
    logic signed [WIDTH-1:0] N_p2_iLhalf;   // N * p2 * i_L[k+1/2]
    logic signed [WIDTH-1:0] gamma_V2k;     // gamma * V2[k]
    logic signed [WIDTH-1:0] V2_nd;         // V2_undamped = V2[k] + beta*N*p2*i_L[k+1/2]
    logic signed [WIDTH-1:0] V2_new;        // V2[k+1]  (after dissipation)

    logic signed [WIDTH-1:0] n_v2nd;        // N * V2_nd          (unsigned sense)
    logic signed [WIDTH-1:0] N_p2_V2nd;     // N * p2 * V2_nd
    logic signed [WIDTH-1:0] i_L_ud;        // i_L[k+1] before R_L half-damping
    logic signed [WIDTH-1:0] i_L_new;       // i_L[k+1]

    always_comb begin

        // p1*V1 and N*V2[k] in parallel (both read from registers only)
        p1_V1    = p1 ? V1 : -V1;
        n_v2k    = fpmul(N_RATIO, V2_reg);
        N_p2_V2k = p2 ? n_v2k : -n_v2k;

        // gamma*V2[k] can also start here (needed in step 2, independent of step 1)
        gamma_V2k = fpmul(GAMMA, V2_reg);

        // Step 1: i_L[k+1/2] (with R_L first half-damp)
        i_L_half = i_L_reg - fpmul(DELTA_L, i_L_reg)
                            + fpmul(ALPHA,   p1_V1 - N_p2_V2k);

        // Step 2a: V2_undamped (conservative only, needs i_L_half)
        n_ilh       = fpmul(N_RATIO, i_L_half);
        N_p2_iLhalf = p2 ? n_ilh : -n_ilh;
        V2_nd       = V2_reg + fpmul(BETA, N_p2_iLhalf);

        // Step 3: i_L[k+1] (uses V2_undamped, then R_L second half-damp)
        n_v2nd     = fpmul(N_RATIO, V2_nd);
        N_p2_V2nd  = p2 ? n_v2nd : -n_v2nd;
        i_L_ud     = i_L_half + fpmul(ALPHA,   p1_V1 - N_p2_V2nd);
        i_L_new    = i_L_ud   - fpmul(DELTA_L, i_L_ud);

        // Step 2b: apply output dissipation to get V2[k+1]
        V2_new      = V2_nd - gamma_V2k;
    end

    // State registers  - capture new state on every rising clock edge
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_L_reg <= IL_INIT;
            V2_reg  <= V2_INIT;
        end else begin
            i_L_reg <= i_L_new;
            V2_reg  <= V2_new;
        end
    end

    // Outputs (registered, so stable between clock edges)
    assign i_L_out = i_L_reg;
    assign V2_out  = V2_reg;

endmodule
