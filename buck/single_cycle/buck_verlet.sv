// Real-time digital twin of a buck converter - Verlet integrator top level.
//
// Each clock cycle executes one full Verlet step combinationally:
//   1. ik_half:  i_{k+1/2} from (i_k, v_k, v_in, sk)
//      [parallel] G*v_k term in vk_new (depends only on g_load, v_k)
//   2. vk_new:   v_{k+1}   from (v_k, i_{k+1/2}, g_load)
//   3. ik_new:   i_{k+1}   from (i_{k+1/2}, v_{k+1}, v_in, sk)
// State registers update on the rising clock edge.
//
// g_load = 1/R (load conductance) in Q8.24.  The caller should set
// g_load = round((1/R) * 2^24) whenever R changes.  For R = 5 ohm:
//   g_load = round(0.2 * 2^24) = 3_355_443.
//
// v_in = input voltage in Q8.24.  The caller should set
// v_in = round(Vin * 2^24) whenever Vin changes.  For Vin = 12 V:
//   v_in = 0x0C00_0000.
//
// Outputs are the registered state (valid one cycle after rst deasserts).
// Format: Q8.24 signed fixed-point. To convert to real: value / 2^24.

module buck_verlet (
    input  logic clk,
    input  logic rst,
    input  logic sk,
    input  logic signed [31:0] g_load,  // load conductance = 1/R, Q8.24
    input  logic signed [31:0] v_in,    // input voltage, Q8.24
    output logic signed [31:0] i_out,   // inductor current, Q8.24
    output logic signed [31:0] v_out    // capacitor voltage, Q8.24
);
    logic signed [31:0] i_k,     v_k;
    logic signed [31:0] i_half_w, v_new_w, i_new_w;

    ik_half u_ihalf (
        .i_k   (i_k),
        .v_k   (v_k),
        .v_in  (v_in),
        .sk    (sk),
        .i_half(i_half_w)
    );

    vk_new u_vknew (
        .v_k   (v_k),
        .i_half(i_half_w),
        .g_load(g_load),
        .v_new (v_new_w)
    );

    ik_new u_iknew (
        .i_half(i_half_w),
        .v_new (v_new_w),
        .v_in  (v_in),
        .sk    (sk),
        .i_new (i_new_w)
    );

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            i_k <= 32'sd0;
            v_k <= 32'sd0;
        end else begin
            i_k <= i_new_w;
            v_k <= v_new_w;
        end
    end

    assign i_out = i_k;
    assign v_out = v_k;

endmodule
