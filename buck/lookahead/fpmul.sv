// fpmul.sv  -  Fixed-point multiply for the Buck look-ahead datapath.
//
//   result = round( a * b / 2^SHIFT )
//
//   a : coefficient, Q*.FRAC_C  (signed, up to ~26 bits -> may use 2 DSPs)
//   b : state,       Q*.FRAC_S  (signed)
//   product frac bits = FRAC_C + FRAC_S; we shift right by SHIFT to land the
//   result back in the state format Q*.FRAC_S, so SHIFT = FRAC_C.
//
// Pipelined: 2 register stages so a wide (cascaded-DSP) multiply still meets
// timing. Latency = 2 cycles. Round-half-up (add 1<<(SHIFT-1) before shift).
//
// (* use_dsp = "yes" *) asks Vivado to map the multiply (and the rounding add)
// into DSP slices; the wide coefficient cascades across two DSP48s via the
// dedicated PCIN/PCOUT path.
module fpmul #(
    parameter int A_W   = 32,   // coefficient width
    parameter int B_W   = 32,   // state width
    parameter int OUT_W = 32,   // output (state) width
    parameter int SHIFT = 24    // = FRAC_C; right-shift to return to state frac
)(
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic signed [A_W-1:0]      a,      // coefficient
    input  logic signed [B_W-1:0]      b,      // state
    output logic signed [OUT_W-1:0]    p       // rounded product, 2-cycle latency
);
    localparam int PROD_W = A_W + B_W;

    // ---- Stage 1: the multiply (mapped to DSP, cascaded if wide) ----
    (* use_dsp = "yes" *)
    logic signed [PROD_W-1:0] prod_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) prod_q <= '0;
        else        prod_q <= a * b;
    end

    // ---- Stage 2: round-half-up + arithmetic shift, then truncate ----
    logic signed [PROD_W-1:0] rounded;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) p <= '0;
        else begin
            rounded = prod_q + (PROD_W'(1) <<< (SHIFT-1));
            p <= OUT_W'(rounded >>> SHIFT);
        end
    end
endmodule