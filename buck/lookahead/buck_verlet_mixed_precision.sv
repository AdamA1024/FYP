// Design: Two-Step Mixed-Precision Verlet Digital Twin with Hot Bootstrapping
// State Format:      18-bit Signed Fixed-Point (Q6.12) -> Range [-32.0, +31.999]
// Coefficient Format: 18-bit Signed Fixed-Point (Q2.16) -> Maximum fractional resolution
// Target: 100 MHz+ Clock (10ns Cycle Time)

package verlet_pkg;
    typedef logic signed [17:0] q6_12; // Runtime States (v, i, Vin)
    typedef logic signed [17:0] q2_16; // Static Core Matrix Constants

    // Buck 2-step look-ahead Verlet coefficients (single switch input).
    //
    // Project constraints: f_clk = 100 MHz (dt = 10 ns),
    //                      100 update steps per PWM period -> f_sw = 1 MHz.
    // Target plant: f_LC = 100 kHz  (-> f_sw / f_LC = 10x, normal buck regime
    //                                 where the LC filter strongly attenuates
    //                                 switching ripple).
    //   -> L = 2.25 uH, C = 1.125 uF, Z = sqrt(L/C) = sqrt(2) ohm
    //   -> a = kC*hL = dt^2/(2 L C) = 1.974e-5
    //
    // Single-step  M(kR) = [[1-kR-a, kC], [-hL*(2-kR-a), 1-a]]
    // Input        u_hat(s) = s*[a, hL*(2-a)]
    // 2-step   x_{k+2} = M^2 x_k + M*u_hat(s_k)*Vin + u_hat(s_{k+1})*Vin
    // Split M^2 = M2_base + kR*D.
    // Input term split per sub-step (kR=0):
    //   B1 = M*u_hat   -> first  sub-step switch s_k
    //   B2 =   u_hat   -> second sub-step switch s_{k+1}
    // The old single coeff was B = B1 + B2 = (M+I)*u_hat, i.e. the
    // s_k == s_{k+1} approximation that is only wrong at duty-cycle edges.
    // B1 + B2 still equals the old B, so DC gain v_in_s -> v_out stays 1.0.
    //
    // D-row coefficients scaled by 1/S=3 so the Q6.12 damp_sum can't
    // overflow at worst-case state (|D[0,0]|*v_max + |D[0,1]|*i_max ~ 64 V).
    // User drives kR_in = S * dt/(R*C); valid kR_phys range: [0, 2/S) = [0, 0.667).

    localparam int unsigned KR_SCALE = 3;  // kR_in = KR_SCALE * dt/(R*C)

    // M2_base entries for the v row + scaled D-row
    localparam q2_16 C_VA  = 18'sh0FFFB; //  +0.999924  M2_base[0,0]
    localparam q2_16 C_VB  = 18'sh0048D; //  +0.017776  M2_base[0,1]
    // Per-substep input coefficients (replaces single C_VC).  Sum = old C_VC.
    localparam q2_16 C_VC1 = 18'sh00004; //  +6.10e-5   B1[0] = a*(3-2a)  -> s_k (registered: switch at substep 1)
    localparam q2_16 C_VC2 = 18'sh00001; //  +1.53e-5   B2[0] = a         -> s_k (current:    switch at substep 2)
    localparam q2_16 C_VD  = 18'sh35557; //  -0.666641  D[0,0]/S
    localparam q2_16 C_VE  = 18'sh3FF3E; //  -2.96e-3   D[0,1]/S

    // M2_base entries for the i row + scaled D-row
    localparam q2_16 C_IA  = 18'sh3FDBA; //  -8.88e-3   M2_base[1,0]
    localparam q2_16 C_IB  = 18'sh0FFFB; //  +0.999924  M2_base[1,1]
    localparam q2_16 C_IC1 = 18'sh00123; //  +4.44e-3   B1[1] = h*(2-a)*(1-2a)  -> s_k registered
    localparam q2_16 C_IC2 = 18'sh00123; //  +4.44e-3   B2[1] = h*(2-a)         -> s_k current
    localparam q2_16 C_ID  = 18'sh000C2; //  +2.96e-3   D[1,0]/S
    localparam q2_16 C_IE  = 18'sh00000; //   0  (below LSB)   D[1,1]/S
endpackage

import verlet_pkg::*;

module buck_verlet_mixed_precision (
    input  logic        clk,
    input  logic        rst_n,        // Active-low synchronous reset

    // Runtime Dynamic Inputs
    input  q2_16        kR,           // Dynamic damping factor (dt/RC) in high precision Q2.16
    input  logic        s_k,          // 1-bit switch state (gate signal)
    input  q6_12        Vin,          // DC input voltage (Q6.12)

    // State Outputs
    output q6_12        v_out,        // Emulated Voltage (Q6.12)
    output q6_12        i_out         // Emulated Current (Q6.12)
);

    // Hot Bootstrapping Parameters (Q6.12) - hardcoded to zero to keep the
    // Vivado-side I/O pin budget below the 125-pin limit.  The testbench
    // always drove these to '0, so a cold start from rest is the only
    // bootstrapping configuration we ever exercise.  If non-zero seeding is
    // ever needed again, promote these back to ports (or to runtime-loadable
    // registers behind a small AXI/control interface).
    localparam q6_12 V_INIT_0 = '0;
    localparam q6_12 I_INIT_0 = '0;
    localparam q6_12 V_INIT_1 = '0;
    localparam q6_12 I_INIT_1 = '0;

    // Recursive State Registers (Q6.12)
    q6_12 v_reg, i_reg;

    // Switch-modulated inputs.  The 2-step look-ahead recurrence is
    //    x_{k+2} = M^2 x_k + M*u_hat * s_k * Vin + u_hat * s_{k+1} * Vin,
    // i.e. the two sub-steps see independent switch samples.  In pipeline
    // time the source state v_reg corresponds to step "n"; the s_k driven
    // by the testbench at the same clock corresponds to s_{n+1} (substep 2),
    // and s_k registered for one cycle (s_k_d1) corresponds to s_n (substep 1).
    // This recovers the duty-edge behaviour the old single-coeff form approximated.
    logic s_k_d1;
    q6_12 v_in_s1, v_in_s2;
    assign v_in_s1 = s_k_d1 ? Vin : '0;  // pairs with B1 (= M*u_hat)
    assign v_in_s2 = s_k    ? Vin : '0;  // pairs with B2 (=   u_hat)

    // --- PIPELINE STAGE 1 SIGNALS (36-bit Intermediate Products) ---
    // Q6.12 (State) * Q2.16 (Coef) = Q8.28 intermediate result
    logic signed [35:0] prod_v_base_a, prod_v_base_b, prod_v_base_c1, prod_v_base_c2;
    logic signed [35:0] prod_v_damp_a, prod_v_damp_b;
    logic signed [35:0] prod_i_base_a, prod_i_base_b, prod_i_base_c1, prod_i_base_c2;
    logic signed [35:0] prod_i_damp_a, prod_i_damp_b;

    q6_12 v_damp_sum, i_damp_sum;
    logic signed [35:0] prod_v_kR, prod_i_kR; // Q6.12 (Sum) * Q2.16 (kR) = Q8.28

    // Inter-stage Pipeline Registers (Q6.12)
    // Pipeline balancing: the base sum is split into two 2-input partial sums
    // (state-row and input-row), each registered separately.  This drops the
    // Stage-1 critical path from (Tmult + 2*Tadd) to (Tmult + Tadd), matching
    // the damp path and Stage 2.  Stage 2 combines them in parallel with the
    // kR multiply, so its critical path stays at (Tmult + Tadd).
    q6_12 r1_v_state, r1_v_input, r1_v_damp;
    q6_12 r1_i_state, r1_i_input, r1_i_damp;
    q2_16 r1_kR;

    // --- STAGE 1: Mixed-Precision Multiplication ---
    always_comb begin
        // Voltage Row Operations
        prod_v_base_a  = v_reg   * C_VA;
        prod_v_base_b  = i_reg   * C_VB;
        prod_v_base_c1 = v_in_s1 * C_VC1;  // first  substep input (registered switch)
        prod_v_base_c2 = v_in_s2 * C_VC2;  // second substep input (current   switch)
        prod_v_damp_a  = v_reg   * C_VD;
        prod_v_damp_b  = i_reg   * C_VE;

        // Current Row Operations
        prod_i_base_a  = v_reg   * C_IA;
        prod_i_base_b  = i_reg   * C_IB;
        prod_i_base_c1 = v_in_s1 * C_IC1;
        prod_i_base_c2 = v_in_s2 * C_IC2;
        prod_i_damp_a  = v_reg   * C_ID;
        prod_i_damp_b  = i_reg   * C_IE;

        // Shift by 16 to drop the extra Q2.16 fractional component, scaling back to Q6.12
        v_damp_sum = q6_12'((prod_v_damp_a + prod_v_damp_b) >>> 16);
        i_damp_sum = q6_12'((prod_i_damp_a + prod_i_damp_b) >>> 16);
    end

    // --- PIPELINE LAYER 1 (Intermediate Storage & Bootstrapping) ---
    // The base sum that used to be a single 4-input add is now split into two
    // 2-input pair-sums (state row and input row), each truncated to Q6.12 and
    // registered.  V_INIT_1 / I_INIT_1 are zero, so splitting the seed across
    // the two registers is trivial (both reset to 0).
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r1_v_state <= V_INIT_1;     // V_INIT_1 = 0; both halves go to 0
            r1_v_input <= '0;
            r1_v_damp  <= '0;
            r1_i_state <= I_INIT_1;
            r1_i_input <= '0;
            r1_i_damp  <= '0;
            r1_kR      <= '0;
            s_k_d1     <= 1'b0;         // substep-1 switch sample (one cycle stale)
        end else begin
            // Each pair-sum: multiply + 2-input add + >>>16 trunc -> Q6.12.
            // Stage-1 critical path is now Tmult + Tadd on every path.
            r1_v_state <= q6_12'((prod_v_base_a  + prod_v_base_b ) >>> 16);
            r1_v_input <= q6_12'((prod_v_base_c1 + prod_v_base_c2) >>> 16);
            r1_v_damp  <= v_damp_sum;   // store D*x; *kR happens in Stage 2
            r1_i_state <= q6_12'((prod_i_base_a  + prod_i_base_b ) >>> 16);
            r1_i_input <= q6_12'((prod_i_base_c1 + prod_i_base_c2) >>> 16);
            r1_i_damp  <= i_damp_sum;
            r1_kR      <= kR;           // align kR with the registered damp sum
            s_k_d1     <= s_k;          // delay the switch one cycle for B1 pairing
        end
    end

    // --- STAGE 2: combine pair-sums + kR multiply + final accumulation ---
    // Combining (r1_*_state + r1_*_input) runs in parallel with (kR * r1_*_damp),
    // so the critical path is max(Tadd, Tmult) + Tadd = Tmult + Tadd.
    q6_12 v_next, i_next;
    always_comb begin
        prod_v_kR = r1_v_damp * r1_kR;   // Q6.12 * Q2.16 = Q8.28
        prod_i_kR = r1_i_damp * r1_kR;
    end
    assign v_next = (r1_v_state + r1_v_input) + q6_12'(prod_v_kR >>> 16);
    assign i_next = (r1_i_state + r1_i_input) + q6_12'(prod_i_kR >>> 16);

    // --- PIPELINE LAYER 2 (Core Recursive State Registers) ---
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_reg <= V_INIT_0; // Seed with t=0 state directly
            i_reg <= I_INIT_0; // Seed with t=0 state directly
        end else begin
            v_reg <= v_next; 
            i_reg <= i_next; 
        end
    end

    // Assigning Outputs
    assign v_out = v_reg;
    assign i_out = i_reg;

endmodule