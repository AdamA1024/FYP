// dab_top.sv  -  Top-level wrapper that pairs the AXI-driven phase generator
//                with the Velocity-Verlet DAB core.
//
// Datapath (one clk cycle, end-to-end):
//   phase (AXI) --> phaseGen --> p1, p2 (combinational) --> dab_rtl
//                                                            |
//                                                  i_L_out / V2_out (registered)
//
// PERIOD is in core-clock ticks (same units as phase).  Defaults match
// dab_rtl: PERIOD = 100 @ 100 MHz -> Tsw = 1 us, fsw = 1 MHz.
//
// The dab_rtl core uses its built-in default operating-point constants
// (ALPHA/BETA/GAMMA/DELTA_L/N_RATIO).  Only WIDTH/FRAC are propagated here;
// override the operating point at this level if a different power stage
// is needed.

module dab_top #(
    parameter int WIDTH  = 32,
    parameter int FRAC   = 20,
    parameter int PERIOD = 100
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic signed [WIDTH-1:0] V1,
    input  logic        [31:0]      phase,
    output logic signed [WIDTH-1:0] i_L_out,
    output logic signed [WIDTH-1:0] V2_out
);

    logic p1, p2;

    phaseGen #(
        .PERIOD(PERIOD)
    ) u_phaseGen (
        .clk   (clk),
        .rst_n (rst_n),
        .phase (phase),
        .p1    (p1),
        .p2    (p2)
    );

    dab_rtl #(
        .WIDTH(WIDTH),
        .FRAC (FRAC)
    ) u_dab_rtl (
        .clk    (clk),
        .rst_n  (rst_n),
        .V1     (V1),
        .p1     (p1),
        .p2     (p2),
        .i_L_out(i_L_out),
        .V2_out (V2_out)
    );

endmodule
