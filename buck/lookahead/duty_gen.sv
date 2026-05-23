// duty_gen.sv  -  Switch pattern generator for the N=3 look-ahead Buck.
//
// Produces the THREE switch states (s_k, s_k1, s_k2) consumed by one M3 jump,
// for the source index currently entering the pipeline. Because each jump
// advances the simulation by 3 timesteps, the per-period step counter
// advances by 3 each valid cycle.
//
// Switch is high while (step within period) < duty_steps, else low.
// duty_steps is the number of "on" steps per period (= round(duty * STEPS)).
//
// NOTE: each interleaved trajectory (mod 3) has its own phase within the
// period. We track a separate step counter per trajectory so the s-triple
// always matches the source index x that is being launched this cycle.
module duty_gen #(
    parameter int STEPS = 99           // steps per switching period (mult. of 3)
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid,         // high when a jump launches this cycle
    input  logic [$clog2(STEPS+1)-1:0] duty_steps, // on-steps per period
    output logic        s_k,           // switch at source index
    output logic        s_k1,          // switch at source+1
    output logic        s_k2           // switch at source+2
);
    // We launch source indices 0,1,2,3,... one per valid cycle (interleaved
    // across the three trajectories automatically, since index == cycle).
    // Track the current source index modulo STEPS.
    logic [$clog2(STEPS+1)-1:0] step;  // current source index within period

    function automatic logic sw(input logic [$clog2(STEPS+1)-1:0] pos);
        // pos already reduced mod STEPS
        return (pos < duty_steps);
    endfunction

    // next two positions (mod STEPS)
    logic [$clog2(STEPS+1)-1:0] step1, step2;
    always_comb begin
        step1 = (step + 1 >= STEPS) ? (step + 1 - STEPS) : (step + 1);
        step2 = (step + 2 >= STEPS) ? (step + 2 - STEPS) : (step + 2);
        s_k   = sw(step);
        s_k1  = sw(step1);
        s_k2  = sw(step2);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) step <= '0;
        else if (valid) begin
            // advance source index by 1 per launch (NOT by 3): consecutive
            // cycles launch consecutive indices; the +3 jump is in the math,
            // not the launch cadence.
            step <= (step + 1 >= STEPS) ? (step + 1 - STEPS) : (step + 1);
        end
    end
endmodule