// Buck converter digital twin - AXI4-Lite slave wrapper for CIL testing.
//
// Architecture (controller-in-loop):
//   PS (Cortex-A53 / R5F) runs the PID in C.  PL contains the plant, the
//   PWM generator, and this AXI4-Lite register file.  PS <-> PL traffic flows
//   through Zynq UltraScale+ M_AXI_HPM0_FPD.
//
// Clocking:
//   s_axi_aclk also clocks the plant.  Same domain = no CDC.  Drive this
//   from PL_CLK0 = 50 MHz so dt = 20 ns matches buck_params.sv.
//
// Register map (byte offset, 32 B aperture so addr[4:2] selects 8 regs):
//   0x00  CTRL        RW   bit0 enable       - gate PWM (sk = 0 when low)
//                          bit1 reset_plant  - holds plant i_k, v_k at 0
//                          bit2 irq_enable   - gates irq output
//   0x04  STATUS      RW   bit0 period_tick  - W1C: set on period rollover,
//                          bit1 running      - mirrors CTRL.enable (RO)
//   0x08  V_IN        RW   Q8.24 input voltage   (default 12.0 V)
//   0x0C  G_LOAD      RW   Q8.24 load conductance (default 0.2 S, R = 5 ohm)
//   0x10  DUTY_CYC    RW   integer cycles out of FSW_PERIOD (1..FSW_PERIOD-1)
//   0x14  V_OUT       RO   Q8.24 capacitor voltage, end-of-period snapshot
//   0x18  I_OUT       RO   Q8.24 inductor current, end-of-period snapshot
//   0x1C  PERIOD_CNT  RO   32-bit free-running count of completed periods
//
// PWM / snapshot semantics:
//   DUTY_CYC write goes to a staging register (duty_cyc_wr).  On each period
//   rollover (pwm_cnt == FSW_PERIOD-1) the hardware shadow-latches the new
//   duty into duty_cyc_reg and snapshots v_out / i_out.  This gives the PS
//   a coherent view and glitch-free PWM even if the CPU writes mid-period.
//
// IRQ:
//   irq = STATUS[0] & CTRL[2].  STATUS[0] sets on each period rollover and
//   is cleared by a PS W1C (write 1 to bit 0 of STATUS).  Level-sensitive;
//   configure the GIC entry as level-high for the corresponding pl_ps_irq.
//
// Base address:
//   M_AXI_HPM0_FPD on ZynqMP defaults to 0xA000_0000.  Vivado Address Editor
//   will map this IP there unless overridden.

`timescale 1ns/1ps

module buck_axi_wrapper #(
    parameter int C_S_AXI_DATA_WIDTH = 32,
    parameter int C_S_AXI_ADDR_WIDTH = 5,          // 32-byte aperture
    parameter int FSW_PERIOD         = 100         // cycles per switching period
) (
    // AXI-Lite slave / plant clock (same domain)
    input  logic                              s_axi_aclk,
    input  logic                              s_axi_aresetn,

    // Write address
    input  logic [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  logic [2:0]                        s_axi_awprot,
    input  logic                              s_axi_awvalid,
    output logic                              s_axi_awready,

    // Write data
    input  logic [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  logic [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  logic                              s_axi_wvalid,
    output logic                              s_axi_wready,

    // Write response
    output logic [1:0]                        s_axi_bresp,
    output logic                              s_axi_bvalid,
    input  logic                              s_axi_bready,

    // Read address
    input  logic [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    input  logic [2:0]                        s_axi_arprot,
    input  logic                              s_axi_arvalid,
    output logic                              s_axi_arready,

    // Read data
    output logic [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata,
    output logic [1:0]                        s_axi_rresp,
    output logic                              s_axi_rvalid,
    input  logic                              s_axi_rready,

    // To PS GIC (level-sensitive)
    output logic                              irq
);

    // Register decode / defaults
    localparam logic [2:0] R_CTRL       = 3'd0;
    localparam logic [2:0] R_STATUS     = 3'd1;
    localparam logic [2:0] R_V_IN       = 3'd2;
    localparam logic [2:0] R_G_LOAD     = 3'd3;
    localparam logic [2:0] R_DUTY_CYC   = 3'd4;
    localparam logic [2:0] R_V_OUT      = 3'd5;
    localparam logic [2:0] R_I_OUT      = 3'd6;
    localparam logic [2:0] R_PERIOD_CNT = 3'd7;

    localparam logic signed [31:0] V_IN_RST   = 32'h0C00_0000;  // 12.0 V Q8.24
    localparam logic signed [31:0] G_LOAD_RST = 32'sd3_355_443; // 0.2 S   Q8.24
    localparam int DUTY_MIN = 1;
    localparam int DUTY_MAX = FSW_PERIOD - 1;
    localparam int PWM_W    = $clog2(FSW_PERIOD);

    // Registers
    logic [31:0]         ctrl_reg;
    logic                status_period_tick;   // W1C bit 0 of STATUS
    logic signed [31:0]  v_in_reg;
    logic signed [31:0]  g_load_reg;
    logic [PWM_W-1:0]    duty_cyc_wr;          // staged (PS writes)
    logic [PWM_W-1:0]    duty_cyc_reg;         // shadow-latched (drives PWM)
    logic signed [31:0]  v_out_snap;
    logic signed [31:0]  i_out_snap;
    logic [31:0]         period_cnt;

    logic                plant_rst;
    logic [PWM_W-1:0]    pwm_cnt;
    logic                period_tick_pulse;
    logic                sk;
    logic signed [31:0]  v_out_live, i_out_live;

    // AXI4-Lite write handshake (Xilinx template idiom)
    logic [C_S_AXI_ADDR_WIDTH-1:0] aw_addr_q;
    logic                          aw_en;

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            aw_addr_q     <= '0;
            aw_en         <= 1'b1;
        end else begin
            if (!s_axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en) begin
                s_axi_awready <= 1'b1;
                aw_addr_q     <= s_axi_awaddr;
                aw_en         <= 1'b0;
            end else if (s_axi_bready && s_axi_bvalid) begin
                aw_en         <= 1'b1;
                s_axi_awready <= 1'b0;
            end else begin
                s_axi_awready <= 1'b0;
            end
        end
    end

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn)
            s_axi_wready <= 1'b0;
        else if (!s_axi_wready && s_axi_wvalid && s_axi_awvalid && aw_en)
            s_axi_wready <= 1'b1;
        else
            s_axi_wready <= 1'b0;
    end

    logic write_fire;
    assign write_fire = s_axi_awready && s_axi_awvalid &&
                        s_axi_wready  && s_axi_wvalid;

    // Register write decode - plain writable registers
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            ctrl_reg    <= 32'd0;
            v_in_reg    <= V_IN_RST;
            g_load_reg  <= G_LOAD_RST;
            duty_cyc_wr <= PWM_W'(DUTY_MIN);
        end else if (write_fire) begin
            case (aw_addr_q[4:2])
                R_CTRL:   ctrl_reg   <= s_axi_wdata;
                R_V_IN:   v_in_reg   <= s_axi_wdata;
                R_G_LOAD: g_load_reg <= s_axi_wdata;
                R_DUTY_CYC: begin
                    if (s_axi_wdata[15:0] > 16'(DUTY_MAX))
                        duty_cyc_wr <= PWM_W'(DUTY_MAX);
                    else if (s_axi_wdata[15:0] < 16'(DUTY_MIN))
                        duty_cyc_wr <= PWM_W'(DUTY_MIN);
                    else
                        duty_cyc_wr <= s_axi_wdata[PWM_W-1:0];
                end
                default: ;
            endcase
        end
    end

    // STATUS[0] - period_tick (set by plant, W1C by PS)
    // Setting (period_tick_pulse) takes priority over W1C clear so a rollover
    // that coincides with a PS write never gets silently dropped.
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            status_period_tick <= 1'b0;
        end else begin
            if (write_fire && aw_addr_q[4:2] == R_STATUS && s_axi_wdata[0])
                status_period_tick <= 1'b0;
            if (period_tick_pulse)
                status_period_tick <= 1'b1;
        end
    end

    // Write response (B) channel
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
        end else if (write_fire && !s_axi_bvalid) begin
            s_axi_bvalid <= 1'b1;
            s_axi_bresp  <= 2'b00;                  // OKAY
        end else if (s_axi_bready && s_axi_bvalid) begin
            s_axi_bvalid <= 1'b0;
        end
    end

    // AXI4-Lite read handshake
    logic [C_S_AXI_ADDR_WIDTH-1:0] ar_addr_q;

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            ar_addr_q     <= '0;
        end else if (!s_axi_arready && s_axi_arvalid) begin
            s_axi_arready <= 1'b1;
            ar_addr_q     <= s_axi_araddr;
        end else begin
            s_axi_arready <= 1'b0;
        end
    end

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rresp  <= 2'b00;
        end else if (s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
            s_axi_rvalid <= 1'b1;
            s_axi_rresp  <= 2'b00;
        end else if (s_axi_rvalid && s_axi_rready) begin
            s_axi_rvalid <= 1'b0;
        end
    end

    // Read mux - latched one cycle after the AR handshake
    logic [31:0] rdata_next;
    always_comb begin
        unique case (ar_addr_q[4:2])
            R_CTRL:       rdata_next = ctrl_reg;
            R_STATUS:     rdata_next = {30'd0, ctrl_reg[0], status_period_tick};
            R_V_IN:       rdata_next = v_in_reg;
            R_G_LOAD:     rdata_next = g_load_reg;
            R_DUTY_CYC:   rdata_next = 32'(duty_cyc_wr);
            R_V_OUT:      rdata_next = v_out_snap;
            R_I_OUT:      rdata_next = i_out_snap;
            R_PERIOD_CNT: rdata_next = period_cnt;
            default:      rdata_next = 32'd0;
        endcase
    end

    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn)
            s_axi_rdata <= 32'd0;
        else if (s_axi_arready && s_axi_arvalid)
            s_axi_rdata <= rdata_next;
    end

    // Plant reset composition - asserted on AXI reset OR CTRL.reset_plant
    assign plant_rst = !s_axi_aresetn || ctrl_reg[1];

    // PWM counter + shadow latch + end-of-period snapshot
    always_ff @(posedge s_axi_aclk) begin
        if (plant_rst) begin
            pwm_cnt           <= '0;
            duty_cyc_reg      <= PWM_W'(DUTY_MIN);
            v_out_snap        <= '0;
            i_out_snap        <= '0;
            period_cnt        <= '0;
            period_tick_pulse <= 1'b0;
        end else begin
            period_tick_pulse <= 1'b0;
            if (pwm_cnt == PWM_W'(FSW_PERIOD - 1)) begin
                pwm_cnt           <= '0;
                duty_cyc_reg      <= duty_cyc_wr;
                v_out_snap        <= v_out_live;
                i_out_snap        <= i_out_live;
                period_cnt        <= period_cnt + 32'd1;
                period_tick_pulse <= 1'b1;
            end else begin
                pwm_cnt <= pwm_cnt + 1'b1;
            end
        end
    end

    assign sk = ctrl_reg[0] ? (pwm_cnt < duty_cyc_reg) : 1'b0;

    // Plant
    buck_verlet u_plant (
        .clk   (s_axi_aclk),
        .rst   (plant_rst),
        .sk    (sk),
        .g_load(g_load_reg),
        .v_in  (v_in_reg),
        .i_out (i_out_live),
        .v_out (v_out_live)
    );

    // IRQ - level-sensitive to the PS GIC
    assign irq = status_period_tick & ctrl_reg[2];

endmodule
