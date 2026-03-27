// Verlet half-step for inductor current:
//   i_{k+1/2} = i_k + (k_L/2) * (s_k*v_in - v_k)
//
// v_in is supplied as a runtime Q8.24 input (same timing as v_k).
//
// Format: Q8.24 signed fixed-point (32-bit)
// Constant scaling: each coefficient C is stored as round(C * 2^32).
// Multiplying a Q8.24 variable (scaled by 2^24) gives a 64-bit product
// scaled by 2^56; taking bits [63:32] divides by 2^32, leaving Q8.24.

module ik_half
    import buck_params::*;
(
    input  logic signed [31:0] i_k,
    input  logic signed [31:0] v_k,
    input  logic signed [31:0] v_in,   // input voltage, Q8.24
    input  logic               sk,
    output logic signed [31:0] i_half
);

    logic signed [31:0] diff;
    logic signed [63:0] prod;

    assign diff   = (sk ? v_in : 32'sd0) - v_k;
    assign prod   = $signed(KL2_SCALED) * diff;
    assign i_half = i_k + $signed(prod[63:32]);

endmodule
