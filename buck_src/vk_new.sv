// Verlet full step for capacitor voltage:
//   v_{k+1} = v_k + kC * (i_{k+1/2} - G*v_k)
//
// where G = 1/R (load conductance), supplied as g_load in Q8.24.
//
// Derivation:
//   v_{k+1} = v_k + kC*i_{k+1/2} - kR*v_k
//           = v_k + kC*i_{k+1/2} - kC*G*v_k   (since kR = kC*G)
//           = v_k + kC * (i_{k+1/2} - G*v_k)
//
// The term G*v_k (prod_gv) depends only on g_load and v_k, which are both
// cycle-start values.  It therefore resolves in parallel with the ik_half
// computation in the top-level datapath, keeping critical-path depth
// identical to the fixed-R design.
//
// Fixed-point arithmetic (Q8.24 signed, 32-bit):
//   g_load * v_k  : two Q8.24 values -> 64-bit product; bits[55:24] = Q8.24
//   KC_SCALED * x : 32-bit scaled coeff * Q8.24 -> 64-bit; bits[63:32] = Q8.24

module vk_new
    import buck_params::*;
(
    input  logic signed [31:0] v_k,
    input  logic signed [31:0] i_half,
    input  logic signed [31:0] g_load,   // conductance = 1/R, Q8.24
    output logic signed [31:0] v_new
);

    // --- parallel with ik_half: G*v_k (depends only on cycle-start values) ---
    logic signed [63:0] prod_gv;
    logic signed [31:0] gv;              // G * v_k in Q8.24

    assign prod_gv = g_load * v_k;
    assign gv      = prod_gv[55:24];

    // --- after ik_half: single kC multiply on the effective current ---
    logic signed [31:0] cur_eff;         // i_{k+1/2} - G*v_k, Q8.24
    logic signed [63:0] prod_c;

    assign cur_eff = i_half - gv;
    assign prod_c  = $signed(KC_SCALED) * cur_eff;
    assign v_new   = v_k + $signed(prod_c[63:32]);

endmodule
