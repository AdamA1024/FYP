// dab_top.sv  -  Top level wiring the SPS switch generator to the DAB solver.
//
// The controller supplies the high-level plant inputs (V1, gamma) and the
// modulation command (phase_shift).  dab_switch_gen turns the phase shift into
// the two bridge polarities p1/p2, which drive the look-ahead solver (dab1).
//
// Compile order: dab2.sv (defines dab_la_pkg + dab_look_ahead_solver),
//                dab_switch_gen.sv, dab_top.sv.

import dab_la_pkg::*;

module dab_top #(
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
    // Bridge polarities from the SPS generator.
    b_pol p1, p2;

    dab_switch_gen #(
        .PWM_PERIOD (PWM_PERIOD),
        .PHASE_W    (PHASE_W)
    ) switch_gen_inst (
        .clk         (clk),
        .rst_n       (rst_n),
        .phase_shift (phase_shift),
        .p1          (p1),
        .p2          (p2)
    );

    dab_look_ahead_solver dab1 (
        .clk      (clk),
        .rst_n    (rst_n),
        .V1       (V1),
        .p1       (p1),
        .p2       (p2),
        .gamma_in (gamma_in),
        .i_L_out  (i_L_out),
        .V2_out   (V2_out)
    );

endmodule
