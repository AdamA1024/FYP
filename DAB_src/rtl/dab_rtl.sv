// =============================================================================
// dab_rtl.sv  –  Dual Active Bridge (DAB) DC-DC converter
//                Fixed-point Velocity-Verlet RTL
//
// Velocity-Verlet integration (one timestep Δt per clock cycle):
//
//   α = Δt/(2L),  β = Δt/Co,  γ = Δt/(RCo)
//
//   i_L[k+½] = i_L[k]   + α·(p1·V1  − N·p2·V2[k])
//   V2[k+1]  = V2[k]    + β·N·p2·i_L[k+½] − γ·V2[k]
//   i_L[k+1] = i_L[k+½] + α·(p1·V1  − N·p2·V2[k+1])
//
// All arithmetic is fixed-point Q(WIDTH-FRAC-1).(FRAC) signed.
// All constants are pre-scaled by 2^FRAC.
//
// Latency:  one clock cycle (no pipelining).
// The combinational datapath computes all three half-steps in parallel
// where data-dependencies allow:
//   • p1·V1 and N·V2[k] are computed simultaneously (both read-only inputs).
//   • fpmul(GAMMA, V2_reg) overlaps with step-1 multiplier.
//   • Steps 2 and 3 are serialised by their data dependency on i_L_half /
//     V2_new respectively — this is fundamental to the algorithm.
// =============================================================================

module dab_rtl #(
    parameter int WIDTH   = 32,           // fixed-point word width (bits)
    parameter int FRAC    = 20,           // fractional bits
    //
    // Default operating point
    //   Δt = 100 ns,  fsw = 10 kHz  (HALF_PERIOD = 500 timesteps)
    //   V1 = 400 V,   N = 2,  V2_nom = 200 V
    //   L  =   1 mH  (transformer leakage inductance)
    //   Co = 100 µF,  R = 10 Ω  (output filter / load)
    //
    // All constants are stored as round(real_value × 2^FRAC).
    //
    // α = Δt/(2L) = 100e-9/(2×1e-3) = 5.000e-5  →  round(5e-5 × 2^20) = 52
    parameter logic signed [WIDTH-1:0] ALPHA    = 32'sd52,

    // β = Δt/Co   = 100e-9/100e-6   = 1.000e-3   →  round(1e-3 × 2^20) = 1049
    parameter logic signed [WIDTH-1:0] BETA     = 32'sd1049,

    // γ = Δt/(RCo)= 100e-9/(10×100e-6)= 1.000e-4  →  round(1e-4 × 2^20) = 105
    parameter logic signed [WIDTH-1:0] GAMMA    = 32'sd105,

    // N = 2  →  2 × 2^FRAC = 2 097 152
    parameter logic signed [WIDTH-1:0] N_RATIO  = 32'sd2097152,

    // Reset / initial conditions
    //   V2_INIT: pre-charge output cap (0 = cold start, 209715200 = 200 V nominal)
    parameter logic signed [WIDTH-1:0] V2_INIT  = 32'sd0,
    parameter logic signed [WIDTH-1:0] IL_INIT  = 32'sd0
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // Primary bus voltage (fixed-point, same Q format as outputs)
    input  logic signed [WIDTH-1:0] V1,

    // Bridge drive polarity:  1 → phase = +1,  0 → phase = −1
    input  logic                    p1,
    input  logic                    p2,

    output logic signed [WIDTH-1:0] i_L_out,   // inductor current  [A · 2^FRAC]
    output logic signed [WIDTH-1:0] V2_out     // output voltage    [V · 2^FRAC]
);

    // -------------------------------------------------------------------------
    // State registers
    // -------------------------------------------------------------------------
    logic signed [WIDTH-1:0] i_L_reg, V2_reg;

    // -------------------------------------------------------------------------
    // Fixed-point multiply:  fpmul(a, b) = round_toward_zero(a · b · 2^−FRAC)
    //
    // Intermediate product is 2·WIDTH bits (signed); arithmetic right-shift
    // by FRAC recovers the Q(WIDTH-FRAC-1).(FRAC) result.
    // -------------------------------------------------------------------------
    function automatic logic signed [WIDTH-1:0] fpmul (
        input logic signed [WIDTH-1:0] a,
        input logic signed [WIDTH-1:0] b
    );
        /* verilator lint_off WIDTHEXPAND */
        logic signed [2*WIDTH-1:0] prod;
        /* verilator lint_on  WIDTHEXPAND */
        prod = a * b;
        // Arithmetic right-shift by FRAC; assignment truncates to WIDTH bits.
        return WIDTH'(prod >>> FRAC);
    endfunction

    // -------------------------------------------------------------------------
    // Combinational datapath  — all three Verlet sub-steps in one cycle
    // -------------------------------------------------------------------------

    // ── Intermediate wires ────────────────────────────────────────────────────
    logic signed [WIDTH-1:0] p1_V1;         // p1 · V1
    logic signed [WIDTH-1:0] n_v2k;         // N · V2[k]          (unsigned sense)
    logic signed [WIDTH-1:0] N_p2_V2k;      // N · p2 · V2[k]

    logic signed [WIDTH-1:0] i_L_half;      // i_L[k+½]

    logic signed [WIDTH-1:0] n_ilh;         // N · i_L[k+½]       (unsigned sense)
    logic signed [WIDTH-1:0] N_p2_iLhalf;   // N · p2 · i_L[k+½]
    logic signed [WIDTH-1:0] gamma_V2k;     // γ · V2[k]
    logic signed [WIDTH-1:0] V2_new;        // V2[k+1]

    logic signed [WIDTH-1:0] n_v2n;         // N · V2[k+1]        (unsigned sense)
    logic signed [WIDTH-1:0] N_p2_V2new;    // N · p2 · V2[k+1]
    logic signed [WIDTH-1:0] i_L_new;       // i_L[k+1]

    always_comb begin

        // ── PARALLEL: p1·V1  and  N·V2[k]  (both read from registers only) ──
        p1_V1    = p1 ? V1 : -V1;
        n_v2k    = fpmul(N_RATIO, V2_reg);
        N_p2_V2k = p2 ? n_v2k : -n_v2k;

        // γ·V2[k] can also start here (needed in step 2, independent of step 1)
        gamma_V2k = fpmul(GAMMA, V2_reg);

        // ── STEP 1: i_L[k+½] ─────────────────────────────────────────────────
        i_L_half = i_L_reg + fpmul(ALPHA, p1_V1 - N_p2_V2k);

        // ── STEP 2: V2[k+1]  (needs i_L_half from step 1) ───────────────────
        n_ilh       = fpmul(N_RATIO, i_L_half);
        N_p2_iLhalf = p2 ? n_ilh : -n_ilh;
        V2_new      = V2_reg + fpmul(BETA, N_p2_iLhalf) - gamma_V2k;

        // ── STEP 3: i_L[k+1]  (needs V2_new from step 2) ─────────────────────
        n_v2n      = fpmul(N_RATIO, V2_new);
        N_p2_V2new = p2 ? n_v2n : -n_v2n;
        i_L_new    = i_L_half + fpmul(ALPHA, p1_V1 - N_p2_V2new);
    end

    // -------------------------------------------------------------------------
    // State registers  — capture new state on every rising clock edge
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_L_reg <= IL_INIT;
            V2_reg  <= V2_INIT;
        end else begin
            i_L_reg <= i_L_new;
            V2_reg  <= V2_new;
        end
    end

    // -------------------------------------------------------------------------
    // Outputs (registered, so stable between clock edges)
    // -------------------------------------------------------------------------
    assign i_L_out = i_L_reg;
    assign V2_out  = V2_reg;

endmodule
