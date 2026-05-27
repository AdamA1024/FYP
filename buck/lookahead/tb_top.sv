// tb_top.sv - Vivado xsim testbench for the buck digital twin (top.sv)
//
// Applies a constant 50% PWM duty to a fixed Vin = 12 V and lets the plant
// run to steady state. With the DC gain of 1.0 by construction, v_out should
// settle near duty * Vin = 0.50 * 12 = 6 V.
//
// Run in xsim:
//   xvlog -sv buck_verlet_mixed_precision.sv pwm_duty_gen.sv top.sv tb_top.sv
//   xelab -debug typical tb_top -s tb_top_sim
//   xsim tb_top_sim -runall
// (or just `make xsim` if you add the target to the Makefile)

`timescale 1ns / 1ps

module tb_top;

    // ---- Fixed-point helpers (match buck_verlet_mixed_precision Q-formats) ----
    localparam int Q6_12_FRAC = 12;   // state format: Q6.12
    localparam int Q2_16_FRAC = 16;   // coeff format: Q2.16
    localparam int DUTY_W     = 32;
    localparam int STATE      = 18;

    // Encode a real voltage/current into Q6.12 (18-bit signed).
    function automatic logic signed [STATE-1:0] to_q6_12(input real x);
        to_q6_12 = $rtoi(x * (1 << Q6_12_FRAC));
    endfunction

    // Encode a real coefficient into Q2.16 (18-bit signed).
    function automatic logic signed [STATE-1:0] to_q2_16(input real x);
        to_q2_16 = $rtoi(x * (1 << Q2_16_FRAC));
    endfunction

    // Decode a Q6.12 code back to a real for display.
    function automatic real from_q6_12(input logic signed [STATE-1:0] c);
        from_q6_12 = real'(c) / (1 << Q6_12_FRAC);
    endfunction

    // ---- Test stimulus parameters ----
    localparam real CLK_PERIOD = 10.0;   // ns -> 100 MHz, dt = 10 ns
    localparam real VIN_V      = 12.0;   // input voltage
    localparam real KR_SCALE   = 3.0;    // KR_SCALE baked into verlet_pkg
    localparam real KR_PHYS    = 0.002;  // physical dt/(R*C); module sees KR_SCALE*kR_phys
    // 50% duty with PWM_PERIOD = 100 (set in top.sv) -> duty count = 50.
    localparam int  DUTY_CNT   = 50;
    localparam int  RUN_CLOCKS = 200_000; // ~2000 PWM periods, plenty to settle

    // ---- DUT signals ----
    logic                    clk;
    logic                    rst_n;
    logic signed [STATE-1:0] Vin;
    logic signed [STATE-1:0] kR;
    logic        [DUTY_W-1:0] duty;
    logic signed [STATE-1:0] Vout;
    logic signed [STATE-1:0] Iout;

    top #(
        .DUTY_W(DUTY_W),
        .state (STATE)
    ) dut (
        .clk  (clk),
        .rst_n(rst_n),
        .Vin  (Vin),
        .kR   (kR),
        .duty (duty),
        .Vout (Vout),
        .Iout (Iout)
    );

    // ---- Clock ----
    initial clk = 1'b0;
    always #(CLK_PERIOD/2.0) clk = ~clk;

    // ---- Stimulus ----
    initial begin
        // Constant inputs
        Vin  = to_q6_12(VIN_V);
        kR   = to_q2_16(KR_SCALE * KR_PHYS);
        duty = DUTY_W'(DUTY_CNT);

        // Hold reset for a few clocks
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        $display("--- buck twin: const 50%% duty, Vin = %0.2f V ---", VIN_V);
        $display("Vin code = %0d (0x%05h), duty = %0d/100, kR code = %0d",
                 Vin, Vin, DUTY_CNT, kR);
        $display(" cycle   v_out[V]   i_out[A]");

        // Run and periodically report v_out / i_out
        for (int c = 0; c < RUN_CLOCKS; c++) begin
            @(posedge clk);
            if (c % 10_000 == 0)
                $display("%7d  %8.4f  %8.4f", c, from_q6_12(Vout), from_q6_12(Iout));
        end

        $display("--- final: v_out = %0.4f V (expect ~%0.2f V), i_out = %0.4f A ---",
                 from_q6_12(Vout), 0.50 * VIN_V, from_q6_12(Iout));
        $finish;
    end

    // ---- Optional waveform dump for the xsim GUI ----
    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top);
    end

endmodule
