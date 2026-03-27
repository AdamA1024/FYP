// Centralised parameters for the buck converter digital twin.
//
// Physical parameters:
//   Vin  = runtime input (Q8.24), supplied via v_in port
//   L    = 10 uH
//   C    = 100 uF
//   R    = runtime input (load resistance, ohm)
//   fsw  = 500 kHz  ->  Tsw = 2000 ns
//   duty = 0.5      ->  v_out ~ 6 V
//   dt   = 20 ns    (one clock cycle @ 50 MHz)
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
// Multiplying a Q8.24 variable (x2^24) gives a 64-bit product x2^56;
// taking bits [63:32] divides by 2^32, recovering Q8.24.
// For two Q8.24 operands (x2^48 product), bits [55:24] recover Q8.24.

package buck_params;
    // kL/2 = dt/(2*L) = 20e-9/(2*10e-6) = 1e-3 -> round(1e-3 * 2^32) = 4_294_967
    parameter [31:0]        KL2_SCALED = 32'd4_294_967;

    // kC = dt/C = 20e-9/100e-6 = 2e-4 -> round(2e-4 * 2^32) = 858_993
    parameter [31:0]        KC_SCALED  = 32'd858_993;

endpackage
