// dab_switch_gen.sv  -  Single-phase-shift (SPS) bridge polarity generator.
//
// Drives the two bridge polarities p1 (primary) and p2 (secondary) that feed
// the DAB look-ahead solver.  Both bridges run a fixed 50% square wave
// (+/-1, never Z); the controller sets the *phase shift* between them, which is
// what actually controls power transfer in a DAB.
//
//   p1 = +1 for the first half period, -1 for the second   (reference bridge)
//   p2 = the same square wave, delayed by `phase_shift` clocks (lags p1)
//
// Matches DAB.py:  p1 = p_square(t, 0),  p2 = p_square(t, tshift).
//
// Low latency: p1/p2 are produced *combinationally* from the free-running
// period counter - there are no pipeline registers between the phase input
// and the polarity outputs, so the solver sees a fresh polarity every clock.
// `phase_shift` is sampled once per period (at the p1 boundary) so a mid-period
// controller update can't produce a runt half-cycle.
//
// Period convention (matches the solver's clock cadence): PWM_PERIOD counts
// one per clock.  phase_shift is in the same clock units:
//   phase_shift = 0              -> p2 in phase with p1   (0deg,  no transfer)
//   phase_shift = PWM_PERIOD/2   -> p2 antiphase          (180deg, max transfer)
// Default PWM_PERIOD=100 -> 100 clocks/period.

import dab_la_pkg::*;

module dab_switch_gen #(
    parameter int PWM_PERIOD = 100,
    parameter int PHASE_W    = 32
) (
    input  logic              clk,
    input  logic              rst_n,
    input  logic [PHASE_W-1:0] phase_shift,  // p2 lag in clocks (0..PWM_PERIOD)
    output b_pol              p1,            // primary bridge polarity   (+1/-1)
    output b_pol              p2             // secondary bridge polarity (+1/-1)
);
    localparam b_pol POS = 2'sb01;   //  +1
    localparam b_pol NEG = 2'sb11;   //  -1
    localparam logic [PHASE_W-1:0] CNT_MAX     = PHASE_W'(PWM_PERIOD - 1);
    localparam logic [PHASE_W-1:0] HALF_PERIOD = PHASE_W'(PWM_PERIOD / 2);

    // Free-running period counter + per-period latched phase shift.
    logic [PHASE_W-1:0] cnt;
    logic [PHASE_W-1:0] phase_latched;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cnt           <= '0;
            phase_latched <= phase_shift;       // seed first period
        end else if (cnt == CNT_MAX) begin
            cnt           <= '0;
            phase_latched <= phase_shift;       // start of next period: sample controller
        end else begin
            cnt <= cnt + 1'b1;
        end
    end

    // p2 position = (cnt - phase) mod PWM_PERIOD, computed without a divide.
    logic [PHASE_W-1:0] p2_pos;
    always_comb begin
        if (cnt >= phase_latched) p2_pos = cnt - phase_latched;
        else                      p2_pos = cnt + PWM_PERIOD - phase_latched;
    end

    // Combinational 50%-duty square waves -> low-latency polarity outputs.
    assign p1 = (cnt    < HALF_PERIOD) ? POS : NEG;
    assign p2 = (p2_pos < HALF_PERIOD) ? POS : NEG;
endmodule
