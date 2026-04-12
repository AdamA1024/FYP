// Centralised parameters for the buck converter digital twin.
//
// Physical parameters:
//   Vin  = runtime input (Q8.24), supplied via v_in port
//   L    = 10 µH
//   C    = 100 µF
//   R    = runtime input (load resistance, Ω)
//   fsw  = 100 kHz  ->  Tsw = 10 µs
//   duty = 0.5      ->  v_out ≈ 6 V
//   dt   = 50 ns    (one clock cycle)
//
// Verlet coefficients:
//   kL = dt / L       = 50e-9 / 10e-6  = 5.0e-3
//   kC = dt / C       = 50e-9 / 100e-6 = 5.0e-4
//   kR = dt / (R*C)   = kC / R         = kC * G   (G = 1/R, conductance)
//
// kR is no longer a compile-time constant.  The load conductance G = 1/R is
// supplied at runtime as g_load (Q8.24).  vk_new exploits the factorisation:
//
//   v_{k+1} = v_k + kC*i_{k+1/2} - kC*G*v_k
//           = v_k + kC * (i_{k+1/2} - G*v_k)
//
// so only KC_SCALED is needed.  The term G*v_k is computed in parallel with
// ik_half (both depend only on cycle-start values), keeping critical-path
// depth unchanged relative to the fixed-R design.
//
// Fixed-point format: Q8.24 signed (32-bit).
//   1.0 = 2^24 = 16_777_216,  12.0 = 0x0C00_0000
//
// Scaled constants: round(coefficient * 2^32).
// Multiplying a Q8.24 variable (×2^24) gives a 64-bit product ×2^56;
// taking bits [63:32] divides by 2^32, recovering Q8.24.
// For two Q8.24 operands (×2^48 product), bits [55:24] recover Q8.24.

package buck_params;
    // kL/2 = 0.0025 -> round(0.0025 * 2^32) = 10_737_418
    parameter [31:0]        KL2_SCALED = 32'd10_737_418;

    // kC = 5e-4 -> round(5e-4 * 2^32) = 2_147_484
    parameter [31:0]        KC_SCALED  = 32'd2_147_484;

endpackage
