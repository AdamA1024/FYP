// Testbench for buck_axi_wrapper.sv
//
// Exercises the CIL datapath the way the PS will: drive AXI-Lite writes to
// configure V_IN / G_LOAD / DUTY_CYC, enable the plant, then poll STATUS for
// period_tick and sample V_OUT after each period.  Pass criterion: v_out
// converges to ~6 V for Vin = 12 V, D = 0.5.
//
// Run with xsim:
//   vivado -mode batch -source vivado_project.tcl -tclargs sim tb_buck_axi_wrapper
// Or Verilator / other SV2012-compatible simulator.

`timescale 1ns/1ps

module tb_buck_axi_wrapper;

    // Parameters
    localparam int  FSW_PERIOD = 100;
    localparam real TCLK_NS    = 20.0;                // 50 MHz

    // Clock / reset
    logic aclk = 0;
    logic aresetn = 0;
    always #(TCLK_NS/2) aclk = ~aclk;

    // AXI-Lite master signals
    logic [4:0]  awaddr;  logic awvalid, awready;
    logic [31:0] wdata;   logic wvalid,  wready;
    logic [3:0]  wstrb;
    logic [1:0]  bresp;   logic bvalid,  bready;
    logic [4:0]  araddr;  logic arvalid, arready;
    logic [31:0] rdata;   logic [1:0] rresp;
    logic rvalid, rready;
    logic irq;

    buck_axi_wrapper #(.FSW_PERIOD(FSW_PERIOD)) dut (
        .s_axi_aclk    (aclk),      .s_axi_aresetn(aresetn),
        .s_axi_awaddr  (awaddr),    .s_axi_awprot (3'b000),
        .s_axi_awvalid (awvalid),   .s_axi_awready(awready),
        .s_axi_wdata   (wdata),     .s_axi_wstrb  (wstrb),
        .s_axi_wvalid  (wvalid),    .s_axi_wready (wready),
        .s_axi_bresp   (bresp),     .s_axi_bvalid (bvalid),
        .s_axi_bready  (bready),
        .s_axi_araddr  (araddr),    .s_axi_arprot (3'b000),
        .s_axi_arvalid (arvalid),   .s_axi_arready(arready),
        .s_axi_rdata   (rdata),     .s_axi_rresp  (rresp),
        .s_axi_rvalid  (rvalid),    .s_axi_rready (rready),
        .irq           (irq)
    );

    // Register byte offsets
    localparam logic [4:0] OFF_CTRL       = 5'h00;
    localparam logic [4:0] OFF_STATUS     = 5'h04;
    localparam logic [4:0] OFF_V_IN       = 5'h08;
    localparam logic [4:0] OFF_G_LOAD     = 5'h0C;
    localparam logic [4:0] OFF_DUTY_CYC   = 5'h10;
    localparam logic [4:0] OFF_V_OUT      = 5'h14;
    localparam logic [4:0] OFF_I_OUT      = 5'h18;
    localparam logic [4:0] OFF_PERIOD_CNT = 5'h1C;

    // Helpers
    function automatic real fp_to_real(input logic [31:0] x);
        return $signed(x) / real'(1 << 24);
    endfunction

    // AXI-Lite BFM
    task automatic axi_write(input logic [4:0] addr, input logic [31:0] data);
        @(posedge aclk); #1;
        awaddr = addr;  awvalid = 1'b1;
        wdata  = data;  wvalid  = 1'b1; wstrb = 4'hF;
        bready = 1'b1;
        do @(posedge aclk); while (!(awready && wready));
        #1; awvalid = 1'b0; wvalid = 1'b0;
        do @(posedge aclk); while (!bvalid);
        #1; bready = 1'b0;
    endtask

    task automatic axi_read(input logic [4:0] addr, output logic [31:0] data);
        @(posedge aclk); #1;
        araddr = addr; arvalid = 1'b1; rready = 1'b1;
        do @(posedge aclk); while (!arready);
        #1; arvalid = 1'b0;
        do @(posedge aclk); while (!rvalid);
        data = rdata; #1; rready = 1'b0;
    endtask

    // Stimulus
    logic [31:0] rb, v_reg, i_reg, pc_reg, st_reg;
    int          ticks;
    real         last_v;

    initial begin
        // Init tb-driven signals
        awaddr=0; awvalid=0; wdata=0; wvalid=0; wstrb=0; bready=0;
        araddr=0; arvalid=0; rready=0;

        repeat (10) @(posedge aclk);
        aresetn = 1;
        repeat (5) @(posedge aclk);

        // Sanity: V_IN write-readback
        axi_write(OFF_V_IN, 32'h0C00_0000);           // 12.0 V Q8.24
        axi_read (OFF_V_IN, rb);
        if (rb !== 32'h0C00_0000) begin
            $display("[tb] FAIL V_IN readback: got %08x", rb);
            $fatal(1);
        end
        $display("[tb] V_IN sanity OK: %.3f V", fp_to_real(rb));

        // Configure plant
        axi_write(OFF_G_LOAD,   32'd3_355_443);       // 0.2 S -> R = 5 ohm
        axi_write(OFF_DUTY_CYC, 32'd50);              // D = 50 %
        axi_write(OFF_CTRL,     32'h0000_0001);       // enable
        $display("[tb] Plant enabled, DUTY_CYC=50 -> expect v_out -> 6 V");

        // Poll for period_tick and sample V_OUT (mimics a PS polling loop)
        ticks = 0; last_v = 0.0;
        while (ticks < 2000) begin
            axi_read(OFF_STATUS, st_reg);
            if (st_reg[0]) begin
                axi_read (OFF_V_OUT,      v_reg);
                axi_read (OFF_I_OUT,      i_reg);
                axi_read (OFF_PERIOD_CNT, pc_reg);
                axi_write(OFF_STATUS, 32'h1);          // W1C clear
                last_v = fp_to_real(v_reg);
                if (ticks % 200 == 0)
                    $display("[tb] period=%0d  v_out=%.3f V  i_L=%.3f A",
                             pc_reg, last_v, fp_to_real(i_reg));
                ticks++;
            end
        end

        if (last_v < 5.5 || last_v > 6.5) begin
            $display("[tb] FAIL: v_out=%.3f V (expected ~6 V)", last_v);
            $fatal(1);
        end
        $display("[tb] PASS: v_out=%.3f V after 2000 periods", last_v);
        $finish;
    end

    // Timeout guard
    initial begin
        #50ms;
        $display("[tb] TIMEOUT");
        $fatal(1);
    end

endmodule
