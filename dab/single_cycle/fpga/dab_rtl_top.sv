// dab_rtl_top.sv  -  Synthesisable FPGA top-level for dab_rtl digital twin
//
// Target board : AXU5EVB-E (Alinx)  -  XCZU5EV-2SFVC784I (Zynq UltraScale+)
// PL_CLK0 is a 200 MHz differential pair; CLK_DIV = 2 divides it to 50 MHz
// (same divider chain as buck_verlet_top).  IBUFDS converts the differential
// input to single-ended before the divider.
//
// Note on clock frequency
//   40 MHz (dt = 25 ns) cannot be derived from 200 MHz with a toggle-FF
//   divider - that scheme gives f_out = f_sys / (2 x CLK_DIV), requiring
//   CLK_DIV = 2.5 (non-integer).  Use the 50 MHz path (dt = 20 ns) here
//   and keep the DAB core parameters at their 20 ns defaults.  If 25 ns is
//   needed on silicon, replace the toggle-FF block with an MMCM.
//
// DAB operating point (dt = 20 ns, fsw = 500 kHz, HALF_PERIOD = 50)
//   V1 = 400 V,  N = 2,  L = 20 uH,  Co = 100 uF,  R = 10 ohm
//   phi  = 13/50 * pi ~ 46.8deg  ->  V2_ss ~ 76.97 V  (open-loop, no controller)
//
// Bridge drive generation
//   A free-running 100-cycle counter produces p1 and p2:
//     p1 : high for sw_cnt  in  [0, HALF_PERIOD)
//     p2 : high for sw_cnt  in  [PHASE_SHIFT, HALF_PERIOD + PHASE_SHIFT)
//          -> p2 lags p1 by PHASE_SHIFT cycles (13/50 * pi ~ 46.8deg)
//
// ILA probe hints (mark_debug attributes are in dab_rtl_ila.sv)
//   Use dab_rtl_ila.sv in the project instead of dab_rtl.sv to get:
//     p1, p2, i_L_reg, V2_reg, i_L_half, V2_nd
//   Suggested depth: 8192 samples  ->  8192 x 20 ns = 163.8 us ~ 82 sw periods
//   Trigger: p1 rising edge (switching cycle boundary)
//
// LED mapping
//   led is asserted when V2_reg bit 26 is set, i.e. V2 >= 64 V (Q11.20).
//   At steady-state V2 ~ 77 V -> bit 26 set -> LED on.
//
// Power-on reset
//   Holds rst high for POR_CYCLES clk_50 cycles after bitstream load.
//   At 50 MHz, 150_000_000 cycles = 3 s - enough time to arm the ILA
//   trigger before the converter starts switching.

module dab_rtl_top #(
    parameter int CLK_DIV    = 2,   // 200 MHz / (2x2) = 50 MHz  (dt = 20 ns)
    parameter int HALF_PERIOD = 50, // cycles at 50 MHz -> fsw = 500 kHz
    parameter int PHASE_SHIFT = 13  // p2 lag in cycles  (13/50*pi ~ 46.8deg)
) (
    input  logic sys_clk_p,         // PL_CLK0_P - differential 200 MHz
    input  logic sys_clk_n,         // PL_CLK0_N
    input  logic sys_rst_n,         // PL_KEY1 - active-low reset
    output logic led                // PL_LED1  - on when V2 >= 64 V
);

    // IBUFDS: differential 200 MHz PL clock -> single-ended sys_clk
    logic sys_clk;
    IBUFDS #(.DIFF_TERM("FALSE"), .IBUF_LOW_PWR("FALSE"))
        u_ibufds (.I(sys_clk_p), .IB(sys_clk_n), .O(sys_clk));

    // 50 MHz clock generation (register-based toggle divider)
    //   f_out = f_sys / (2 x CLK_DIV) = 200 / 4 = 50 MHz  (period 20 ns)
    //   Replace with MMCM for better jitter on silicon.
    logic        clk_50;
    logic [$clog2(CLK_DIV)-1:0] clk_cnt;

    generate
        if (CLK_DIV > 1) begin : gen_clkdiv
            always_ff @(posedge sys_clk) begin
                if (clk_cnt == CLK_DIV - 1) begin
                    clk_cnt <= '0;
                    clk_50  <= ~clk_50;
                end else begin
                    clk_cnt <= clk_cnt + 1;
                end
            end
        end else begin : gen_clkpass
            assign clk_50 = sys_clk;
        end
    endgenerate

    // Power-on reset counter
    //   150 000 000 x 20 ns = 3 s hold after bitstream load -> time to arm ILA
    localparam int POR_CYCLES = 150_000_000;
    logic [$clog2(POR_CYCLES)-1:0] por_cnt;
    logic por_done;

    always_ff @(posedge clk_50) begin
        if (!por_done) begin
            if (por_cnt == POR_CYCLES - 1)
                por_done <= 1'b1;
            else
                por_cnt <= por_cnt + 1;
        end
    end

    // Reset synchroniser (active-high rst, synchronised to clk_50 domain)
    //   rst asserts if button pressed OR POR not yet complete.
    logic rst_sync_0, rst;
    always_ff @(posedge clk_50 or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            rst_sync_0 <= 1'b1;
            rst        <= 1'b1;
        end else begin
            rst_sync_0 <= ~por_done;
            rst        <= rst_sync_0;
        end
    end

    // Bridge drive generator
    //   Free-running 100-cycle counter (one switching period).
    //   p1 : high while sw_cnt < HALF_PERIOD           (first 50 cycles)
    //   p2 : high while sw_cnt in [PHASE_SHIFT,         (cycles 13-62)
    //                               HALF_PERIOD + PHASE_SHIFT)
    //        -> p2 lags p1 by PHASE_SHIFT cycles
    localparam int FULL_PERIOD = 2 * HALF_PERIOD;   // 100 cycles

    logic [$clog2(FULL_PERIOD)-1:0] sw_cnt;

    always_ff @(posedge clk_50 or posedge rst) begin
        if (rst)
            sw_cnt <= '0;
        else if (sw_cnt == FULL_PERIOD - 1)
            sw_cnt <= '0;
        else
            sw_cnt <= sw_cnt + 1;
    end

    logic p1, p2;
    assign p1 = (sw_cnt < HALF_PERIOD);
    // p2 lags p1 by PHASE_SHIFT: high for [PHASE_SHIFT, HALF_PERIOD+PHASE_SHIFT)
    // Valid for PHASE_SHIFT  in  [0, HALF_PERIOD].
    assign p2 = (sw_cnt >= PHASE_SHIFT) && (sw_cnt < HALF_PERIOD + PHASE_SHIFT);

    // Primary bus voltage:  V1 = 400 V  (Q11.20 -> 400 x 2^20 = 419 430 400)
    localparam logic signed [31:0] V1_FP = 32'sd419430400;

    // DAB core  (instantiate dab_rtl_ila.sv for ILA probes, dab_rtl.sv otherwise)
    logic signed [31:0] i_L_out, V2_out;

    dab_rtl #(
        // 20 ns / 50 MHz defaults - matches HALF_PERIOD = 50 above
        //   ALPHA   = 524   (alpha = dt/(2L)    = 20e-9/(2x20e-6))
        //   BETA    = 210   (beta = dt/Co      = 20e-9/100e-6)
        //   GAMMA   = 21    (gamma = dt/(R*Co)  = 20e-9/(10x100e-6))
        //   DELTA_L = 52    (delta = R_L*dt/(2L), R_L ~ 0.1 ohm)
        // All kept at RTL defaults; no override needed.
        .V2_INIT (32'sd0),
        .IL_INIT (32'sd0)
    ) u_dab (
        .clk    (clk_50),
        .rst_n  (~rst),
        .V1     (V1_FP),
        .p1     (p1),
        .p2     (p2),
        .i_L_out(i_L_out),
        .V2_out (V2_out)
    );

    // LED: asserted when V2 >= 64 V (Q11.20 bit 26 set)
    //   At steady-state V2 ~ 77 V -> bit 26 set -> LED on during operation.
    //   After reset or before POR completes: V2 = 0 -> LED off.
    assign led = V2_out[26];

endmodule
