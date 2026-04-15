// tb_dab_vivado.sv  -  Self-checking Vivado/XSim testbench for dab_rtl
//
// Tests
//   T1  Reset / initial conditions       (cold start: V2=0, i_L=0)
//   T2  Steady-state V2 mean accuracy    (within +/-2% of 96.21 V theory at ~10 ms)
//   T3  i_L DC offset                    (|DC| < 50 mA at ~10 ms)
//   T4  V2 steady-state ripple           (peak-to-peak < 1% of V2_theory)
//   T5  No long-term V2 drift            (|V2_late18ms - V2_late10ms| < 10 mV)
//
// V1 is held constant at 400 V for the full simulation (no step event).
//
// Timeline (18.75 ms total, 750 000 clock cycles at 25 ns/cycle, 40 MHz):
//   0 - 1 000         : early window     (first 10 sw periods = 0.25 us window)
//   399 000 - 400 000 : T2/T3/T4 window  (10 sw periods at ~10 ms, 10*tau)
//   749 000 - 750 000 : T5 drift window  (10 sw periods at ~18.75 ms, 18.75*tau)
//
// Parameters match the Makefile (dt = 25 ns, 40 MHz clock, fsw = 400 kHz):
//   alpha = dt/(2L) = 25e-9/(2x20e-6)    = 6.25e-4   -> ALPHA  = 655
//   beta = dt/Co   = 25e-9/100e-6       = 2.50e-4   -> BETA   = 260
//   gamma = dt/(RCo)= 25e-9/(10x100e-6) = 2.50e-5   -> GAMMA  = 26
//   delta = R_L*dt/(2L)  (R_L~0.1 ohm)                -> DELTA_L= 66
//   GAMMA/BETA = 26/260 = 0.1 = 1/R  ->  R_eff = 10 ohm exactly
//
// Theoretical V2_ss (SPS formula):
//   V2_ss = N*V1*R*phi*(pi-phi) / (2pi^2*fsw*L)
//         = 2*400*10*(13pi/50)*(37pi/50) / (2pi^2*400e3*20e-6)
//         = 96.21 V
//
// Outputs dab_results_vivado.csv compatible with plot_results.py.
//   Format:  cycle,p1,p2,i_L_A,V2_V
//
// Usage in Vivado
//   1. Add rtl/dab_rtl.sv and tb/tb_dab_vivado.sv to the project.
//   2. Set tb_dab_vivado as the simulation top.
//   3. Click "Run All" so Vivado runs until $finish.

`timescale 1ns/1ps

module tb_dab_vivado;

    // Simulation knobs
    localparam int    FRAC        = 20;          // Q11.20 fractional bits
    localparam int    WIDTH       = 32;
    localparam int    HALF_PER    = 50;          // clock cycles per half-period (400 kHz)
    localparam int    PHASE_SH    = 13;          // p2 lag  (13/50*pi ~ 46.8deg)
    localparam int    FULL_PER    = 2 * HALF_PER;
    localparam int    SIM_CYCLES  = 750_000;     // 18.75 ms at 25 ns/cycle

    // Fixed-point constant (real x 2^20)
    localparam logic signed [WIDTH-1:0] V1_NOM  = 32'sd419430400;  // 400 V

    // RTL parameters - must match Makefile defaults (dt = 25 ns, fsw = 400 kHz)
    localparam logic signed [WIDTH-1:0] ALPHA    = 32'sd655;
    localparam logic signed [WIDTH-1:0] BETA     = 32'sd260;
    localparam logic signed [WIDTH-1:0] GAMMA    = 32'sd26;
    localparam logic signed [WIDTH-1:0] DELTA_L  = 32'sd66;
    localparam logic signed [WIDTH-1:0] N_RATIO  = 32'sd2097152;
    localparam logic signed [WIDTH-1:0] V2_INIT  = 32'sd0;   // cold start
    localparam logic signed [WIDTH-1:0] IL_INIT  = 32'sd0;   // cold start

    // Theoretical steady-state output voltage:
    //   V2_ss = N*V1*R*phi*(pi-phi) / (2pi^2*fsw*L)  = 96.21 V
    //   (phi = 13pi/50 rad, N=2, V1=400 V, R=10 ohm, fsw=400 kHz, L=20 uH)
    localparam real   V2_THEORY   = 96.21;  // V
    localparam real   V2_TOL      = 0.02;   // +/-2 %  (T2)
    localparam real   IL_DC_MAX   = 0.05;   // A     (T3, 50 mA)
    localparam real   RIPPLE_MAX  = 0.01;   // 1 % pp of V2_theory  (T4)
    localparam real   DRIFT_MAX   = 0.010;  // V     (T5, 10 mV between ~10 ms and ~18.75 ms)

    // Measurement window boundaries (clock cycles)
    localparam int    EARLY_START  = 0;
    localparam int    EARLY_END    = 10 * FULL_PER;              // first 10 sw periods

    // T2/T3/T4: steady-state window at ~10 ms (10 sw periods wide)
    localparam int    MID_END      = 400_000;
    localparam int    MID_START    = MID_END - 10 * FULL_PER;    // 399 000

    // T5: late window near end of sim at ~18.75 ms (10 sw periods wide)
    localparam int    LATE_END     = SIM_CYCLES;
    localparam int    LATE_START   = LATE_END - 10 * FULL_PER;   // 749 000

    // T3 i_L DC: one full switching period inside the mid window
    localparam int    T3_START     = MID_START;
    localparam int    T3_END       = T3_START + FULL_PER;

    // DUT I/O
    logic                    clk   = 1'b0;
    logic                    rst_n = 1'b0;
    logic signed [WIDTH-1:0] V1;
    logic                    p1, p2;
    logic signed [WIDTH-1:0] i_L_out, V2_out;

    // DUT instantiation
    dab_rtl #(
        .WIDTH   (WIDTH),
        .FRAC    (FRAC),
        .ALPHA   (ALPHA),
        .BETA    (BETA),
        .GAMMA   (GAMMA),
        .DELTA_L (DELTA_L),
        .N_RATIO (N_RATIO),
        .V2_INIT (V2_INIT),
        .IL_INIT (IL_INIT)
    ) dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .V1     (V1),
        .p1     (p1),
        .p2     (p2),
        .i_L_out(i_L_out),
        .V2_out (V2_out)
    );

    // 40 MHz clock  ->  25 ns period  =  one RTL timestep per cycle
    always #12.5 clk <= ~clk;

    // Fixed-point -> real  (Q11.20)
    function automatic real fp2r (input logic signed [WIDTH-1:0] v);
        return real'($signed(v)) / 1048576.0;
    endfunction

    // Square-wave generator:  1 -> +phase,  0 -> -phase
    function automatic logic sq_wave (input int cyc, input int lag);
        int ph;
        ph = ((cyc - lag) % FULL_PER + FULL_PER) % FULL_PER;
        return (ph < HALF_PER) ? 1'b1 : 1'b0;
    endfunction

    // Measurement accumulators
    real  v2_sum_mid,  v2_sum_late;
    int   v2_n_mid,    v2_n_late;
    real  v2_max_mid,  v2_min_mid;
    real  il_sum_t3;
    int   il_n_t3;

    // CSV file handle
    integer csv_fd;

    // Main stimulus  (V1 = 400 V constant throughout)
    integer cycle;

    initial begin : stimulus

        // -- CSV --
        csv_fd = $fopen("dab_results_vivado.csv", "w");
        if (csv_fd == 0)
            $display("WARNING: could not open dab_results_vivado.csv");
        else
            $fdisplay(csv_fd, "cycle,p1,p2,i_L_A,V2_V");

        // -- zero accumulators --
        v2_sum_mid   = 0.0;  v2_n_mid   = 0;
        v2_sum_late  = 0.0;  v2_n_late  = 0;
        v2_max_mid   = -1e9; v2_min_mid  = 1e9;
        il_sum_t3    = 0.0;  il_n_t3    = 0;

        V1 = V1_NOM;
        p1 = 1'b1;
        p2 = 1'b1;

        // T1: reset / cold-start initial conditions
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        #1;

        if (i_L_out !== IL_INIT || V2_out !== V2_INIT)
            $error("T1 FAIL: after reset  i_L_out=%0d (exp %0d)  V2_out=%0d (exp %0d)",
                   i_L_out, IL_INIT, V2_out, V2_INIT);
        else
            $display("T1 PASS: cold-start reset  i_L=%.4f A  V2=%.4f V",
                     fp2r(i_L_out), fp2r(V2_out));

        // release reset, run 18.75 ms with constant V1
        @(posedge clk);
        rst_n = 1'b1;

        for (cycle = 0; cycle < SIM_CYCLES; cycle++) begin

            // V1 constant for the whole simulation
            V1 = V1_NOM;
            p1 = sq_wave(cycle, 0);
            p2 = sq_wave(cycle, PHASE_SH);

            @(posedge clk);
            #1;

            // -- CSV (every 5 cycles -> 20 samples per switching period) --
            if (csv_fd != 0 && (cycle % 5 == 0))
                $fdisplay(csv_fd, "%0d,%0d,%0d,%.7f,%.7f",
                          cycle, p1, p2, fp2r(i_L_out), fp2r(V2_out));

            // -- mid window: T2 mean + T4 ripple + T3 DC (~10 ms) --
            if (cycle >= MID_START && cycle < MID_END) begin
                v2_sum_mid += fp2r(V2_out);
                v2_n_mid++;
                if (fp2r(V2_out) > v2_max_mid) v2_max_mid = fp2r(V2_out);
                if (fp2r(V2_out) < v2_min_mid) v2_min_mid = fp2r(V2_out);
            end

            // -- T3: i_L DC over one full switching period --
            if (cycle >= T3_START && cycle < T3_END) begin
                il_sum_t3 += fp2r(i_L_out);
                il_n_t3++;
            end

            // -- T5: late window for drift check (~18.75 ms) --
            if (cycle >= LATE_START && cycle < LATE_END) begin
                v2_sum_late += fp2r(V2_out);
                v2_n_late++;
            end
        end

        if (csv_fd != 0) $fclose(csv_fd);

        // T2: V2 mean accuracy at ~10 ms
        begin : t2
            real v2_mean, err;
            v2_mean = (v2_n_mid > 0) ? v2_sum_mid / real'(v2_n_mid) : 0.0;
            err     = (v2_mean - V2_THEORY) / V2_THEORY;
            if (err < -V2_TOL || err > V2_TOL)
                $error("T2 FAIL: V2 mean=%.3f V  theory=%.3f V  err=%+.2f%%",
                       v2_mean, V2_THEORY, err * 100.0);
            else
                $display("T2 PASS: V2 mean=%.3f V  theory=%.3f V  err=%+.2f%%",
                         v2_mean, V2_THEORY, err * 100.0);
        end

        // T3: i_L DC offset
        begin : t3
            real il_dc;
            il_dc = (il_n_t3 > 0) ? il_sum_t3 / real'(il_n_t3) : 999.0;
            if (il_dc > IL_DC_MAX || il_dc < -IL_DC_MAX)
                $error("T3 FAIL: i_L DC at ~10 ms = %.4f A  (limit +/-%.3f A)",
                       il_dc, IL_DC_MAX);
            else
                $display("T3 PASS: i_L DC at ~10 ms = %.4f A  (limit +/-%.3f A)",
                         il_dc, IL_DC_MAX);
        end

        // T4: steady-state V2 ripple
        begin : t4
            real rip_pp, rip_limit;
            rip_pp    = v2_max_mid - v2_min_mid;
            rip_limit = V2_THEORY * RIPPLE_MAX;
            if (rip_pp > rip_limit)
                $error("T4 FAIL: V2 ripple=%.4f V  (limit %.4f V = %.0f%% of V2_theory)",
                       rip_pp, rip_limit, RIPPLE_MAX * 100.0);
            else
                $display("T4 PASS: V2 ripple=%.4f V  (limit %.4f V, %.3f%% of V2_theory)",
                         rip_pp, rip_limit, rip_pp / V2_THEORY * 100.0);
        end

        // T5: long-term drift check (~10 ms vs ~18.75 ms)
        // Any real drift in V2 shows up here; both windows are deep in steady
        // state so the only source of difference is algorithmic accumulation.
        begin : t5
            real v2_mid, v2_late, drift;
            v2_mid  = (v2_n_mid  > 0) ? v2_sum_mid  / real'(v2_n_mid)  : 0.0;
            v2_late = (v2_n_late > 0) ? v2_sum_late / real'(v2_n_late) : 0.0;
            drift   = v2_late - v2_mid;
            $display("T5 INFO: V2 mean at ~10 ms = %.4f V,  at ~18.75 ms = %.4f V,  drift = %+.5f V",
                     v2_mid, v2_late, drift);
            if ((drift > DRIFT_MAX) || (drift < -DRIFT_MAX))
                $error("T5 FAIL: V2 drift=%+.5f V over ~8.75 ms  (limit +/-%.3f V)",
                       drift, DRIFT_MAX);
            else if (v2_late > V2_THEORY * (1.0 + V2_TOL))
                $error("T5 FAIL: V2 late=%.3f V has drifted PAST V2_theory=%.3f V + %0.0f%% tolerance",
                       v2_late, V2_THEORY, V2_TOL * 100.0);
            else
                $display("T5 PASS: no significant drift  (|%.5f V| < %.3f V, V2 stable below theory)",
                         drift, DRIFT_MAX);
        end

        $display("----------------------------------------------");
        $display("Done. CSV written to dab_results_vivado.csv");
        $finish;
    end

endmodule
