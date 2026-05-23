// buck_lookahead.sv  -  Buck converter N=3 look-ahead digital twin core.
//
// Computes one Verlet state per clock cycle (steady state) using the
// 3-step look-ahead recurrence:
//
//     x[k+3] = M3 * x[k] + u3(s_k, s_k1, s_k2)
//            = x[k] + D3 * x[k] + u3            (I+D form, D3 = M3 - I)
//
// THREE interleaved trajectories (index mod 3) keep the pipeline full:
//     A: 0,3,6,...   B: 1,4,7,...   C: 2,5,8,...
// Pipeline latency = 3 (matches look-ahead distance 3): no stalls.
//
// Pipeline stages (latency 3):
//   S1a: multiply partial      (fpmul stage 1)   } fpmul has 2 internal stages
//   S1b: multiply complete     (fpmul stage 2)   }  -> covers cascaded DSP
//   S2 : adder tree + x + u3   (this module)     -> commit x_next
//
// Coefficient format Q*.FRAC_C, state Q*.FRAC_S (see buck_coeffs_pkg).
import buck_coeffs_pkg::*;

module buck_lookahead #(
    parameter int WIDTH = 32,
    parameter int STEPS = 99            // steps/period, multiple of 3
)(
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic signed [WIDTH-1:0]  Vin,        // Q*.FRAC_S
    input  logic [$clog2(STEPS+1)-1:0] duty_steps,
    input  logic signed [WIDTH-1:0]  v_init,     // Q*.FRAC_S
    input  logic signed [WIDTH-1:0]  i_init,
    // ---- test-only prime injection (hacky; real prime FSM comes later) ----
    input  logic                     prime_we,    // when high, load traj regs below
    input  logic signed [WIDTH-1:0]  prime_v1,    // x[1] v
    input  logic signed [WIDTH-1:0]  prime_i1,
    input  logic signed [WIDTH-1:0]  prime_v2,    // x[2] v
    input  logic signed [WIDTH-1:0]  prime_i2,
    output logic signed [WIDTH-1:0]  v_out,      // latest committed v
    output logic signed [WIDTH-1:0]  i_out       // latest committed i
);
    // Switch triple for the source index entering the pipe this cycle
    logic s_k, s_k1, s_k2;
    logic run;                  // pipeline running (after priming)

    duty_gen #(.STEPS(STEPS)) u_duty (
        .clk(clk), .rst_n(rst_n), .valid(run),
        .duty_steps(duty_steps),
        .s_k(s_k), .s_k1(s_k1), .s_k2(s_k2)
    );

    // State store: one (v,i) register per trajectory residue (mod 3).
    // The "source" presented to the pipe each cycle rotates A->B->C->A...
    logic signed [WIDTH-1:0] v_traj [3];
    logic signed [WIDTH-1:0] i_traj [3];
    logic [1:0] src_sel;        // which trajectory feeds the pipe this cycle (0,1,2)

    // Source state presented to the datapath
    logic signed [WIDTH-1:0] v_src, i_src;
    always_comb begin
        v_src = v_traj[src_sel];
        i_src = i_traj[src_sel];
    end

    // Stage 1: four coefficient multiplies (D3 * x), each via fpmul (2-stage).
    //   d_vv = D3_00 * v ; d_vi = D3_01 * i
    //   d_iv = D3_10 * v ; d_ii = D3_11 * i
    // Plus u3 prep: U3*_idx * Vin.
    logic signed [WIDTH-1:0] d_vv, d_vi, d_iv, d_ii;

    fpmul #(.SHIFT(FRAC_C)) m_vv (.clk,.rst_n,.a(D3_00),.b(v_src),.p(d_vv));
    fpmul #(.SHIFT(FRAC_C)) m_vi (.clk,.rst_n,.a(D3_01),.b(i_src),.p(d_vi));
    fpmul #(.SHIFT(FRAC_C)) m_iv (.clk,.rst_n,.a(D3_10),.b(v_src),.p(d_iv));
    fpmul #(.SHIFT(FRAC_C)) m_ii (.clk,.rst_n,.a(D3_11),.b(i_src),.p(d_ii));

    // u3 coefficient select (combinational ROM on the 3 switch bits)
    logic signed [WIDTH-1:0] u3v_coeff, u3i_coeff;
    logic [2:0] sidx;
    assign sidx = {s_k, s_k1, s_k2};
    always_comb begin
        unique case (sidx)
            3'd0: begin u3v_coeff=U3V_0; u3i_coeff=U3I_0; end
            3'd1: begin u3v_coeff=U3V_1; u3i_coeff=U3I_1; end
            3'd2: begin u3v_coeff=U3V_2; u3i_coeff=U3I_2; end
            3'd3: begin u3v_coeff=U3V_3; u3i_coeff=U3I_3; end
            3'd4: begin u3v_coeff=U3V_4; u3i_coeff=U3I_4; end
            3'd5: begin u3v_coeff=U3V_5; u3i_coeff=U3I_5; end
            3'd6: begin u3v_coeff=U3V_6; u3i_coeff=U3I_6; end
            3'd7: begin u3v_coeff=U3V_7; u3i_coeff=U3I_7; end
        endcase
    end
    // u3 = u3_coeff * Vin  (2-stage fpmul, same latency as D3 multiplies)
    logic signed [WIDTH-1:0] u3v, u3i;
    fpmul #(.SHIFT(FRAC_C)) m_u3v (.clk,.rst_n,.a(u3v_coeff),.b(Vin),.p(u3v));
    fpmul #(.SHIFT(FRAC_C)) m_u3i (.clk,.rst_n,.a(u3i_coeff),.b(Vin),.p(u3i));

    // We must carry the SOURCE state (v_src,i_src) and the target trajectory
    // alongside the 2-cycle multiply so Stage 2 can compute x + Dx + u3 and
    // write it back to the right trajectory register.
    // Delay line: 2 cycles to match fpmul latency.
    logic signed [WIDTH-1:0] v_src_d1, i_src_d1, v_src_d2, i_src_d2;
    logic [1:0]              sel_d1, sel_d2;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_src_d1<=0; i_src_d1<=0; v_src_d2<=0; i_src_d2<=0;
            sel_d1<=0; sel_d2<=0;
        end else begin
            v_src_d1<=v_src; i_src_d1<=i_src; sel_d1<=src_sel;
            v_src_d2<=v_src_d1; i_src_d2<=i_src_d1; sel_d2<=sel_d1;
        end
    end

    // Stage 2: x_next = x + D3*x + u3   (adder tree), commit to trajectory reg
    // d_** and u3* are valid 2 cycles after their source was presented, which
    // aligns with v_src_d2/i_src_d2/sel_d2.
    logic signed [WIDTH-1:0] v_next, i_next;
    always_comb begin
        v_next = v_src_d2 + d_vv + d_vi + u3v;
        i_next = i_src_d2 + d_iv + d_ii + u3i;
    end

    // Priming + run control.
    //   We need x[1], x[2] before trajectories B, C can start from M3 jumps.
    //   Simplest robust approach: a small priming FSM that performs the first
    //   few single-step updates to fill v_traj[1], v_traj[2] (and v_traj[0] is
    //   the initial condition). After priming, assert run and the pipeline
    //   self-sustains. (Priming detail left light here; see notes.)
    // src_sel rotates 0->1->2 each running cycle
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) src_sel <= 2'd0;
        else if (run) src_sel <= (src_sel==2'd2) ? 2'd0 : src_sel+2'd1;
    end

    // commit: write x_next back to the trajectory that was the source 3 cycles
    // ago (sel_d2 is the source-2-cycles-ago; commit happens this cycle = +3
    // relative to launch once registered below).
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_traj[0]<=v_init; i_traj[0]<=i_init;
            v_traj[1]<=0; i_traj[1]<=0;
            v_traj[2]<=0; i_traj[2]<=0;
            v_out<=v_init; i_out<=i_init;
        end else if (prime_we) begin
            // test-only: load correct x[1], x[2]; x[0] stays at init
            v_traj[1]<=prime_v1; i_traj[1]<=prime_i1;
            v_traj[2]<=prime_v2; i_traj[2]<=prime_i2;
        end else if (run) begin
            v_traj[sel_d2] <= v_next;
            i_traj[sel_d2] <= i_next;
            v_out <= v_next;
            i_out <= i_next;
        end
    end

    // Run control: hold 'run' low for the priming interval, then assert.
    // Priming is performed by a separate prime block (see buck_prime.sv) which
    // must load v_traj[1..2] with single-step results before run goes high.
    // For now: a counter that waits PRIME_CYCLES then runs. Replace with the
    // proper prime FSM in integration.
    localparam int PRIME_CYCLES = 8;
    logic [$clog2(PRIME_CYCLES+1)-1:0] prime_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin prime_cnt<=0; run<=1'b0; end
        else if (prime_cnt < PRIME_CYCLES) prime_cnt<=prime_cnt+1;
        else run<=1'b1;
    end
endmodule