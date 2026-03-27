// Vivado/xsim unit testbench for ik_half.
//
// Checks the combinational equation:
//   i_half = i_k + (KL2_SCALED * (sk ? v_in : 0) - v_k) >> 32
//
// All expected outputs are computed from integer arithmetic using
// KL2_SCALED = 4294967  (= round(1e-3 * 2^32), dt=20ns, L=10uH).
//
// Test vectors:
//   TV0: sk=0,  all inputs zero      -> i_half = 0
//   TV1: sk=0,  i_k=Q8.24(1A)       -> i_half = i_k  (sk=0 kills drive term)
//   TV2: sk=1,  i_k=0, v_k=0, 12V   -> i_half = 201326  (~ 0.012 A)
//   TV3: sk=1,  v_k=v_in (12V)      -> i_half = i_k    (diff=0)
//   TV4: sk=1,  v_k=6V, v_in=12V    -> i_half = 100663  (~ 0.006 A step)
//   TV5: sk=1,  negative diff        -> i_half = -100664  (di/dt < 0, floor of neg)
//   TV6: sk=1,  large i_k with step  -> accumulation

`timescale 1ns/1ps

module tb_ik_half;

    // DUT signals
    logic signed [31:0] i_k, v_k, v_in;
    logic               sk;
    logic signed [31:0] i_half;

    ik_half dut (.*);

    // Constants (Q8.24: N = round(value * 2^24))
    localparam int Q_12V = 32'h0C000000;  // 12.0 V = 201326592
    localparam int Q_6V  = 32'h06000000;  //  6.0 V = 100663296
    localparam int Q_1A  = 32'h01000000;  //  1.0 A =  16777216

    // KL2_SCALED = 4294967
    // TV2: diff = Q_12V - 0 = 201326592
    //   prod = 4294967 * 201326592 = 864691068862064
    //   prod[63:32] = 864691068862064 >> 32 = 201326
    //   i_half = 0 + 201326
    localparam int EXP_TV2 = 201326;

    // TV4: diff = Q_12V - Q_6V = 100663296
    //   prod = 4294967 * 100663296 = 432345534431232
    //   432345534431232 >> 32 = 100663
    //   i_half = 0 + 100663
    localparam int EXP_TV4 = 100663;

    // TV5: sk=1, i_k=0, v_k=Q_12V, v_in=Q_6V -> diff = Q_6V - Q_12V = -100663296
    //   prod = 4294967 * (-100663296) = -432345534431232  (signed)
    //   prod[63:32] = -100664  (floor of negative quotient, i_half decreases)
    localparam signed [31:0] EXP_TV5 = -32'sd100664;

    // TV6: i_k=Q_1A, sk=1, v_k=0, v_in=Q_12V
    //   i_half = 16777216 + 201326 = 16978542
    localparam int EXP_TV6 = 16978542;

    int errors = 0;

    task check(
        input string   name,
        input int      got,
        input int      exp
    );
        if (got !== exp) begin
            $error("[%s] FAIL: got=%0d  expected=%0d", name, got, exp);
            errors++;
        end else
            $display("[%s] PASS  i_half=%0d", name, got);
    endtask

    initial begin
        // TV0: sk=0, everything zero -> no drive, i_half = 0
        sk=0; i_k=0; v_k=0; v_in=0; #1;
        check("TV0 sk=0 all-zero", int'(i_half), 0);

        // TV1: sk=0, i_k=1A -> drive term suppressed, i_half = i_k
        sk=0; i_k=Q_1A; v_k=Q_6V; v_in=Q_12V; #1;
        check("TV1 sk=0 passthru", int'(i_half), Q_1A);

        // TV2: sk=1, i_k=0, v_k=0, v_in=12V -> small positive current step
        sk=1; i_k=0; v_k=0; v_in=Q_12V; #1;
        check("TV2 sk=1 zero-start", int'(i_half), EXP_TV2);

        // TV3: sk=1, v_k=v_in -> diff=0, i_half=i_k (steady state at this voltage)
        sk=1; i_k=Q_1A; v_k=Q_12V; v_in=Q_12V; #1;
        check("TV3 sk=1 diff-zero", int'(i_half), Q_1A);

        // TV4: sk=1, v_k half of v_in -> half-step
        sk=1; i_k=0; v_k=Q_6V; v_in=Q_12V; #1;
        check("TV4 sk=1 half-diff", int'(i_half), EXP_TV4);

        // TV5: sk=1, v_k > v_in -> negative diff, inductor de-energises
        sk=1; i_k=0; v_k=Q_12V; v_in=Q_6V; #1;
        check("TV5 sk=1 neg-diff", int'(i_half), int'(EXP_TV5));

        // TV6: sk=1, accumulation with existing current
        sk=1; i_k=Q_1A; v_k=0; v_in=Q_12V; #1;
        check("TV6 sk=1 accum", int'(i_half), EXP_TV6);

        // Summary
        $display("==============================");
        if (errors == 0)
            $display("ik_half: ALL TESTS PASSED");
        else
            $display("ik_half: %0d TEST(S) FAILED", errors);
        $display("==============================");
        $finish;
    end

endmodule
