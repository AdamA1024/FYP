// dab_top3.sv  -  Top level for the timing-optimized DAB engine (dab3.sv).
//
// Same wiring as dab_top.sv (switch generator -> look-ahead solver) but applies
// optimization #5: the bridge polarities p1/p2 are REGISTERED at the switch-gen
// output before entering the solver.
//
// Why: dab_switch_gen produces p1/p2 combinationally from the free-running
// counter (`cnt - phase_latched`, then a magnitude compare).  That subtract +
// compare otherwise sits in series with the solver's Stage-1 coefficient mux and
// multiplier, lengthening the critical path.  A single pipeline register pulls
// that logic off the front of the solver for one cycle of latency - free for a
// streaming plant model.  Both p1 and p2 are delayed by the SAME one cycle, so
// their relative phase shift (the actual DAB control variable) is unchanged.
//
// Compile order: dab3.sv (defines dab_la_pkg + dab_look_ahead_solver),
//                dab_switch_gen.sv, dab_top3.sv.
// NOTE: compile dab3.sv INSTEAD of dab2.sv - both define package dab_la_pkg.

import dab_la_pkg::*;

module dab_top3 #(
    parameter int PWM_PERIOD = 100,
    parameter int PHASE_W    = 32
) (
    input  logic               clk,
    input  logic               rst_n,

    // Plant + modulation inputs
    input  q8_24               V1,           // primary DC link voltage (Q8.24)
    input  q4_28               gamma_in,     // gamma = dt/(R*Co) (Q4.28), runtime
    input  logic [PHASE_W-1:0] phase_shift,  // p2 lag in clocks (0..PWM_PERIOD)

    // State outputs (Q8.24)
    output q8_24               i_L_out,
    output q8_24               V2_out
);
    // Combinational bridge polarities straight from the SPS generator.
    b_pol p1_comb, p2_comb;

    dab_switch_gen #(
        .PWM_PERIOD (PWM_PERIOD),
        .PHASE_W    (PHASE_W)
    ) switch_gen_inst (
        .clk         (clk),
        .rst_n       (rst_n),
        .phase_shift (phase_shift),
        .p1          (p1_comb),
        .p2          (p2_comb)
    );

    //  Opt #5: pipeline p1/p2 once before the solver 
    // Takes the switch-gen subtract/compare off the solver's Stage-1 path.
    //
    // Opt #6 (timing): p2_q (with p2_d1 inside the solver) drives the solver's
    // full coefficient mux - cidx -> 10:1 mux of a 192-bit cset feeding 8 DSPs -
    // so a single FF fans out to ~160 loads, and the routed report showed that net
    // at ~3.2 ns (fo=162): the post-fold critical path.  max_fanout forces the
    // synthesiser to REPLICATE these registers so each copy drives a small, local
    // cluster of mux loads.  Identical values in every copy -> no functional or
    // latency change; purely a placement/routing win.
    (* max_fanout = 24 *) b_pol p1_q, p2_q;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            p1_q <= '0;
            p2_q <= '0;
        end else begin
            p1_q <= p1_comb;
            p2_q <= p2_comb;
        end
    end

    dab_look_ahead_solver dab1 (
        .clk      (clk),
        .rst_n    (rst_n),
        .V1       (V1),
        .p1       (p1_q),
        .p2       (p2_q),
        .gamma_in (gamma_in),
        .i_L_out  (i_L_out),
        .V2_out   (V2_out)
    );

endmodule
