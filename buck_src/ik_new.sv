// Verlet second half-step for inductor current:
//   i_{k+1} = i_{k+1/2} + (k_L/2) * (s_k*v_in - v_{k+1})
//
// v_in is supplied as a runtime Q8.24 input (same timing as other cycle-start values).
//
// Format: Q8.24 signed fixed-point (32-bit)
// Same coefficient (k_L/2) as ik_half -- both half-steps are symmetric.

module ik_new
    import buck_params::*;
(
    input  logic signed [31:0] i_half,
    input  logic signed [31:0] v_new,
    input  logic signed [31:0] v_in,   // input voltage, Q8.24
    input  logic               sk,
    output logic signed [31:0] i_new
);

    logic signed [31:0] diff;
    logic signed [63:0] prod;

    assign diff  = (sk ? v_in : 32'sd0) - v_new;
    assign prod  = $signed(KL2_SCALED) * diff;
    assign i_new = i_half + $signed(prod[63:32]);

endmodule
