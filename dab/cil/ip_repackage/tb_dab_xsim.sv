// tb_dab_xsim.sv  -  Vivado xsim testbench for the DAB twin (dab_top).
//
// Purpose: settle the twin at the exact firmware operating point
//   V1 = 0x64000000 (100 V, Q8.24),  gamma = 0x476 (R=10, Q4.28),  phase = 13
// and print V2 over time.  Verilator settles this at 16.8 V; the board reads
// ~105 V (a constant 6.25x gain error).  This isolates WHERE the 6.25x enters:
//
//   1) BEHAVIORAL xsim (this tb, plain RTL): if V2 -> ~105 V, then Vivado's RTL
//      interpretation differs from Verilator -> a SystemVerilog elaboration bug
//      in dab4_core.sv (width/precision of the fold or the Q8.24xQ4.28 products).
//   2) If behavioral xsim -> ~16.8 V, rerun as POST-SYNTHESIS FUNCTIONAL sim
//      (xsim on the synthesized netlist).  If THAT -> ~105 V, synthesis (DSP48
//      mapping of c1*gamma>>>28) is the culprit.
//
// Compile order: dab_la_pkg.sv, dab4_core.sv, dab_switch_gen.sv, dab_top.sv, this.
// CLI:  xvlog -sv dab_la_pkg.sv dab4_core.sv dab_switch_gen.sv dab_top.sv tb_dab_xsim.sv
//       xelab -debug typical tb_dab_xsim -s sim ; xsim sim -runall

`timescale 1ns/1ps
module tb_dab_xsim;
    logic clk = 0, rst_n = 0;
    logic signed [31:0] V1, gamma_in;
    logic        [31:0] phase_shift;
    logic signed [31:0] i_L_out, V2_out;

    // 50 MHz (20 ns) - matches dt; frequency does not affect the settled value.
    always #10 clk = ~clk;

    dab_top #(.PWM_PERIOD(100), .PHASE_W(32)) dut (
        .clk(clk), .rst_n(rst_n),
        .V1(V1), .gamma_in(gamma_in), .phase_shift(phase_shift),
        .i_L_out(i_L_out), .V2_out(V2_out)
    );

    // Q8.24 raw -> volts*1000 (integer), for display without reals.
    function automatic int q24_mv(input logic signed [31:0] q);
        q24_mv = int'((longint'(q) * 1000) >>> 24);
    endfunction

    integer ms;
    initial begin
        V1          = 32'h64000000;  // 100.0 V  (Q8.24)
        gamma_in    = 32'h00000476;  // R=10     (Q4.28)
        phase_shift = 32'd13;        // 45 deg
        rst_n = 0;
        repeat (8) @(posedge clk);
        rst_n = 1;

        $display("t_ms,V2_mV,iL_mV   (expect V2 -> ~16837; board reads ~105000)");
        // 15 ms total; print every 1 ms.  15 ms = 750k clocks.
        for (ms = 1; ms <= 15; ms = ms + 1) begin
            repeat (50000) @(posedge clk);   // 50k clk * 20 ns = 1 ms
            $display("%0d,%0d,%0d", ms, q24_mv(V2_out), q24_mv(i_L_out));
        end
        $display("DONE: V2 = %0d mV (16837=correct, ~105000=the bug)", q24_mv(V2_out));
        $finish;
    end
endmodule
