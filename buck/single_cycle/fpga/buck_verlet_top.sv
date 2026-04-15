// Synthesizable FPGA top-level for buck_verlet digital twin.
//
// Target board: AXU5EVB-E (Alinx) - XCZU5EV-2SFVC784I (Zynq UltraScale+).
// PL_CLK0 is a 200 MHz differential pair; CLK_DIV = 2 divides it to 50 MHz.
// An IBUFDS converts the differential input to single-ended before the divider.
//
// The module generates a 50 MHz internal clock (clk_50) by dividing sys_clk
// by CLK_DIV (default 4 for the 200 MHz AXU5EVB-E PL oscillator).
// All buck_verlet logic runs on clk_50 so dt = 20 ns matches buck_params.sv.
//
// Switching parameters:
//   fsw     = 500 kHz   (configurable via FSW_PERIOD)
//   duty    = 50 %      (fixed for this wrapper; duty is wired to half-period)
//   Vin     = 12 V      (hardcoded, change VIN_Q824 for other voltages)
//   R_load  = 5 ohm       (change G_LOAD_Q824 for other loads)
//
// ILA probe hints:
//   All signals tagged with (* MARK_DEBUG="TRUE" *) are captured by Vivado ILA.
//   Add an ILA core in the IP integrator or with insert_ila_debug_hub_tcl.
//   Suggested capture depth: 4096 samples -> 4096 x 20 ns = 81.9 us per trigger.
//   Trigger on: rising edge of pwm_sk (switch-on event).
//
// LED mapping (Arty A7 4 x RGB LED or 4 x plain LED):
//   led[3:0] = upper 4 bits of v_out integer part.
//   At v_out = 6 V (Q8.24 = 0x06000000), bits [27:24] = 4'h6 -> leds show 0b0110.
//
// Note: This file is the RTL wrapper only.  The PWM counter runs open-loop
// (no closed-loop controller).  For closed-loop testing, replace the sk
// assignment with output from your digital controller.

module buck_verlet_top #(
    parameter int CLK_DIV   = 2,          // 2 for 200 MHz PL clock (AXU5EVB-E): 200/(2x2)=50 MHz
    parameter int FSW_PERIOD = 100,       // cycles at 50 MHz -> 500 kHz switching
    parameter int DUTY_ON    = 50         // high-side on cycles per period (50 %)
) (
    input  logic        sys_clk_p,        // PL_CLK0_P - differential 200 MHz
    input  logic        sys_clk_n,        // PL_CLK0_N
    input  logic        sys_rst_n,        // active-low reset (button or pin)
    output logic        led               // PL_LED1: high when v_out >= ~4 V
);

    // IBUFDS: differential 200 MHz PL clock -> single-ended sys_clk
    logic sys_clk;
    IBUFDS #(.DIFF_TERM("FALSE"), .IBUF_LOW_PWR("FALSE"))
        u_ibufds (.I(sys_clk_p), .IB(sys_clk_n), .O(sys_clk));

    // 50 MHz clock generation (register-based divider)
    // Replace with an MMCM for better jitter performance on silicon.
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
    //   Holds rst high for POR_CYCLES clk_50 cycles after bitstream loads.
    //   At 50 MHz, 150_000_000 cycles = 3 seconds - enough time to arm the ILA.
    //   The button (sys_rst_n) can re-assert rst at any time as normal.
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

    // Reset synchroniser (synchronise active-high rst to clk_50 domain)
    //   rst asserts if button pressed OR POR not yet complete.
    logic rst_sync_0, rst;
    always_ff @(posedge clk_50 or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            rst_sync_0 <= 1;
            rst        <= 1;
        end else begin
            rst_sync_0 <= ~por_done;
            rst        <= rst_sync_0;
        end
    end

    // PWM counter -> sk (open-loop, D = DUTY_ON / FSW_PERIOD)
    logic [$clog2(FSW_PERIOD)-1:0] pwm_cnt;
    logic pwm_sk;

    always_ff @(posedge clk_50 or posedge rst) begin
        if (rst)
            pwm_cnt <= '0;
        else if (pwm_cnt == FSW_PERIOD - 1)
            pwm_cnt <= '0;
        else
            pwm_cnt <= pwm_cnt + 1;
    end

    assign pwm_sk = (pwm_cnt < DUTY_ON);

    // Plant inputs (Q8.24 fixed-point)
    //   Vin = 12.0 V  -> 0x0C000000
    //   G   =  0.2 S  -> round(0.2 * 2^24) = 3355443   (R = 5 ohm)
    localparam logic signed [31:0] VIN_Q824    = 32'h0C000000;
    localparam logic signed [31:0] G_LOAD_Q824 = 32'sd3_355_443;

    // DUT
    (* MARK_DEBUG = "TRUE" *) logic signed [31:0] i_out;
    (* MARK_DEBUG = "TRUE" *) logic signed [31:0] v_out;
    (* MARK_DEBUG = "TRUE" *) logic                sk_dbg;

    assign sk_dbg = pwm_sk;

    buck_verlet u_buck (
        .clk   (clk_50),
        .rst   (rst),
        .sk    (pwm_sk),
        .g_load(G_LOAD_Q824),
        .v_in  (VIN_Q824),
        .i_out (i_out),
        .v_out (v_out)
    );

    // LED: PL_LED1 - on when v_out integer part has the "4 V" bit set,
    //   i.e. v_out >= ~4 V.  At steady-state 6 V (Q8.24 bit 26 = 1) -> LED on.
    assign led = v_out[26];

endmodule
