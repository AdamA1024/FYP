// dab_top.sv  -  Top level for the DAB twin (timing-optimized engine).
//
// switch generator -> (registered p1/p2) -> look-ahead solver.  The bridge
// polarities p1/p2 are REGISTERED at the switch-gen output before entering the
// solver (opt #5): it takes dab_switch_gen's subtract+compare off the solver's
// Stage-1 critical path for one cycle of latency.  Both p1 and p2 are delayed by
// the SAME cycle, so their relative phase shift (the DAB control variable) is
// unchanged.
//
// Compile order:  dab_la_pkg.sv  ->  dab4_core.sv  ->  dab_switch_gen.sv  ->  dab_top.sv
// (Module is named `dab_top` so the AXI slave instantiates it unchanged.)

import dab_la_pkg::*;

module dab_top #(
    parameter int PWM_PERIOD = 100,
    parameter int PHASE_W    = 32
) (
    input  logic               clk,
    input  logic               rst_n,        // active-low; resets twin STATE only

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

    // Opt #5: pipeline p1/p2 once before the solver
    // Takes the switch-gen subtract/compare off the solver's Stage-1 path.
    // max_fanout forces the synthesiser to REPLICATE these registers so each copy
    // drives a small, local cluster of coefficient-mux loads (placement win;
    // identical values in every copy -> no functional or latency change).
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
