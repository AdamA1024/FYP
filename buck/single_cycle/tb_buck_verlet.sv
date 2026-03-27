// Vivado/xsim integration testbench for buck_verlet.
//
// Tests (all self-checking, $error on failure):
//   T0 - Reset:            i_out = 0, v_out = 0 immediately after reset.
//   T1 - Cycle-1 golden:   First clock edge after rst=0, sk=1, Vin=12V, R=5ohm.
//   T2 - Cycle-2 golden:   Second clock edge (verified by integer arithmetic).
//   T3 - Steady-state:     100 000 cycles, D=0.5, Vin=12V -> avg v_out ~ 6 V +/- 2 %.
//   T4 - Load step:        R 5ohm -> 10ohm mid-run, new ss still ~ 6 V +/- 3 %.
//   T5 - Vin step:         Vin 12->9 V mid-run, new ss ~ 4.5 V +/- 2 %.
//
// Golden values (dt = 20 ns, KL2_SCALED = 4294967, KC_SCALED = 858993):
//   After cycle 1: i_out = 402652,  v_out = 40
//   After cycle 2: i_out = 805304,  v_out = 160
//
// Physical parameters:
//   L = 10 uH,  C = 100 uF,  R = 5 ohm,  Vin = 12 V
//   fsw = 500 kHz -> Tsw = 2000 ns = 100 clock cycles at 20 ns/cycle
//   LC resonance omegan ~ 31.6 krad/s -> Tn ~ 200 us -> 10 000 cycles to settle

`timescale 1ns/1ps

module tb_buck_verlet;

    // DUT signals
    logic        clk, rst, sk;
    logic signed [31:0] g_load, v_in;
    logic signed [31:0] i_out, v_out;

    buck_verlet dut (.*);

    // Clock: 50 MHz -> 20 ns period
    localparam real CLK_PERIOD = 20.0;
    initial clk = 0;
    always #(CLK_PERIOD/2.0) clk = ~clk;

    // Parameters (Q8.24 format: N = round(value * 2^24))
    localparam int VIN_12V  = 32'h0C000000;  // 12.0 V
    localparam int VIN_9V   = 32'h09000000;  //  9.0 V
    localparam int G_5OHM   = 3_355_443;     // 1/5 ohm  = 0.2
    localparam int G_10OHM  = 1_677_722;     // 1/10 ohm = 0.1

    localparam int PWM_PERIOD  = 100;        // 2000 ns / 20 ns
    localparam int DUTY_CYCLES =  50;        // 50 % duty

    // Expected steady-state 6 V in Q8.24
    localparam int SS_6V        = 100_663_296; // 6 * 2^24
    localparam int SS_4V5       =  75_497_472; // 4.5 * 2^24
    localparam int TOL_2PCT     =   2_013_266; // 2 % of 6 V
    localparam int TOL_3PCT     =   3_019_899; // 3 % of 6 V

    // Exact golden values for cycles 1 and 2 (computed from integer arithmetic)
    localparam int GOLDEN_I1 = 402_652;
    localparam int GOLDEN_V1 =      40;
    localparam int GOLDEN_I2 = 805_304;
    localparam int GOLDEN_V2 =     160;

    // Helper task: apply reset for N cycles
    task apply_reset(input int n);
        rst    = 1; sk = 0;
        g_load = G_5OHM;
        v_in   = VIN_12V;
        repeat(n) @(posedge clk);
        rst = 0;
    endtask

    // Stat accumulator for steady-state check
    longint v_accum;
    int     v_count;
    longint v_avg;
    longint ss_lo, ss_hi;

    localparam int TOL_4V5 = 32'd1_509_950; // 2 % of 4.5 V

    // Test flow
    int cycle;
    int errors;

    initial begin
        errors = 0;
        rst = 1; sk = 0; g_load = G_5OHM; v_in = VIN_12V;
        @(posedge clk); #1;

        // T0 - Reset check
        $display("[T0] Reset check ...");
        if (i_out !== 0 || v_out !== 0) begin
            $error("[T0] FAIL: i_out=%0d v_out=%0d (expected 0,0)", i_out, v_out);
            errors++;
        end else
            $display("[T0] PASS");

        // T1 - Cycle-1 golden value check (sk=1, Vin=12 V, R=5 ohm)
        $display("[T1] Cycle-1 golden values ...");
        rst = 0; sk = 1; v_in = VIN_12V; g_load = G_5OHM;
        @(posedge clk); #1;
        if (i_out !== GOLDEN_I1 || v_out !== GOLDEN_V1) begin
            $error("[T1] FAIL: i_out=%0d (exp %0d)  v_out=%0d (exp %0d)",
                   i_out, GOLDEN_I1, v_out, GOLDEN_V1);
            errors++;
        end else
            $display("[T1] PASS  i_out=%0d  v_out=%0d", i_out, v_out);

        // T2 - Cycle-2 golden value check
        $display("[T2] Cycle-2 golden values ...");
        @(posedge clk); #1;
        if (i_out !== GOLDEN_I2 || v_out !== GOLDEN_V2) begin
            $error("[T2] FAIL: i_out=%0d (exp %0d)  v_out=%0d (exp %0d)",
                   i_out, GOLDEN_I2, v_out, GOLDEN_V2);
            errors++;
        end else
            $display("[T2] PASS  i_out=%0d  v_out=%0d", i_out, v_out);

        // T3 - Steady-state: 100 000 cycles, D=50%, Vin=12V, R=5ohm
        //       Average v_out over the last 20 000 cycles must be within 2% of 6V.
        //       The LC resonance settles after ~10 000 cycles (Tn ~ 200 us).
        $display("[T3] Steady-state run (100 000 cycles) ...");
        apply_reset(4);
        v_accum = 0; v_count = 0;
        for (cycle = 0; cycle < 100_000; cycle++) begin
            sk     = ((cycle % PWM_PERIOD) < DUTY_CYCLES) ? 1 : 0;
            g_load = G_5OHM;
            v_in   = VIN_12V;
            @(posedge clk); #1;
            if (cycle >= 80_000) begin
                v_accum += v_out;
                v_count++;
            end
        end
        v_avg = v_accum / v_count;
        ss_lo = SS_6V - TOL_2PCT;
        ss_hi = SS_6V + TOL_2PCT;
        $display("[T3]   avg v_out = %0d Q8.24 = %.4f V  (exp %.4f V +/- 2%%)",
                 v_avg, real'(v_avg) / 16777216.0, 6.0);
        if (v_avg < ss_lo || v_avg > ss_hi) begin
            $error("[T3] FAIL: avg v_out=%0d outside [%0d,%0d]", v_avg, ss_lo, ss_hi);
            errors++;
        end else
            $display("[T3] PASS");

        // T4 - Load step: R 5ohm -> 10ohm after 60 000 cycles.
        //       Steady-state v_out should remain ~ 6 V (+/- 3%, wider tol for transient).
        $display("[T4] Load step R=5ohm -> 10ohm ...");
        apply_reset(4);
        v_accum = 0; v_count = 0;
        for (cycle = 0; cycle < 120_000; cycle++) begin
            sk     = ((cycle % PWM_PERIOD) < DUTY_CYCLES) ? 1 : 0;
            g_load = (cycle < 60_000) ? G_5OHM : G_10OHM;
            v_in   = VIN_12V;
            @(posedge clk); #1;
            if (cycle >= 100_000) begin  // 20 000 cycles after step to settle
                v_accum += v_out;
                v_count++;
            end
        end
        v_avg = v_accum / v_count;
        ss_lo = SS_6V - TOL_3PCT;
        ss_hi = SS_6V + TOL_3PCT;
        $display("[T4]   avg v_out (post-step) = %0d Q8.24 = %.4f V  (exp ~6 V)",
                 v_avg, real'(v_avg) / 16777216.0);
        if (v_avg < ss_lo || v_avg > ss_hi) begin
            $error("[T4] FAIL: avg v_out=%0d outside [%0d,%0d]", v_avg, ss_lo, ss_hi);
            errors++;
        end else
            $display("[T4] PASS");

        // T5 - Vin step: 12V -> 9V after 60 000 cycles.
        //       Expected new steady state: 0.5 x 9 = 4.5 V (+/- 2%).
        $display("[T5] Vin step 12V -> 9V ...");
        apply_reset(4);
        v_accum = 0; v_count = 0;
        for (cycle = 0; cycle < 120_000; cycle++) begin
            sk     = ((cycle % PWM_PERIOD) < DUTY_CYCLES) ? 1 : 0;
            g_load = G_5OHM;
            v_in   = (cycle < 60_000) ? VIN_12V : VIN_9V;
            @(posedge clk); #1;
            if (cycle >= 100_000) begin
                v_accum += v_out;
                v_count++;
            end
        end
        v_avg = v_accum / v_count;
        ss_lo = SS_4V5 - TOL_4V5;
        ss_hi = SS_4V5 + TOL_4V5;
        $display("[T5]   avg v_out (post-step) = %0d Q8.24 = %.4f V  (exp ~4.5 V)",
                 v_avg, real'(v_avg) / 16777216.0);
        if (v_avg < ss_lo || v_avg > ss_hi) begin
            $error("[T5] FAIL: avg v_out=%0d outside [%0d,%0d]", v_avg, ss_lo, ss_hi);
            errors++;
        end else
            $display("[T5] PASS");

        // Summary
        $display("=================================================");
        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("%0d TEST(S) FAILED", errors);
        $display("=================================================");
        $finish;
    end

    // Safety watchdog: kill sim if it runs longer than expected
    initial begin
        #(20.0 * 500_000_000);  // 500 M cycles
        $error("WATCHDOG: simulation timeout");
        $finish;
    end

endmodule
