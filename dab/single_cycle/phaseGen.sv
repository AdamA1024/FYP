// phase generator driven by a phase register written from the PS over
// AXI-Lite.  The phase value is latched at the start of each switching
// period so AXI writes are atomic from the converter's point of view:
// a phase update that arrives mid-period takes effect at the next period
// boundary, not on the current pulse.
//
// PERIOD = number of core-clock ticks per switching period.
// e.g. PERIOD=100 @ 100 MHz core -> Tsw = 1 us -> fsw = 1 MHz.
//
// phase is taken from a 32-bit AXI register; only the low CW bits matter,
// where CW = $clog2(PERIOD).  Writes outside [0, PERIOD-1] are silently
// truncated to the low CW bits (caller's responsibility to clamp).
//
// Outputs:
//   p1  high for counter in [0, HALF_PERIOD)              (leading bridge)
//   p2  high for counter in [phase, phase+HALF_PERIOD)    (trailing bridge,
//       modulo PERIOD - window wraps when phase > HALF_PERIOD)
//
// Latency: p1/p2 are combinational off the free-running counter, so the
// downstream dab_rtl sees a new drive vector every clk edge with no extra
// pipeline stages.

module phaseGen #(
    parameter int PERIOD = 100
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [31:0] phase,
    output logic        p1,
    output logic        p2
);

    localparam int HALF_PERIOD = PERIOD / 2;
    localparam int CW          = $clog2(PERIOD);

    logic [CW-1:0] counter;
    logic [CW-1:0] phase_latched;
    logic [CW-1:0] p2_end;       // (phase + HALF_PERIOD) mod PERIOD
    logic          p2_wrap;      // 1 if the p2 window straddles the period boundary

    // Precompute the wrapped window end from the *incoming* phase request.
    // Only the value sampled at the period boundary is registered, so AXI
    // writes mid-period don't disturb the running pulse.
    logic [CW-1:0] phi;
    logic [CW:0]   sum;          // CW+1 bits: up to PERIOD + HALF_PERIOD - 1
    logic [CW-1:0] new_p2_end;
    logic          new_p2_wrap;

    assign phi         = phase[CW-1:0];   // upper bits intentionally ignored
    assign sum         = {1'b0, phi} + (CW+1)'(HALF_PERIOD);
    assign new_p2_wrap = (sum >= (CW+1)'(PERIOD));
    assign new_p2_end  = new_p2_wrap ? CW'(sum - (CW+1)'(PERIOD)) : CW'(sum);

    // Free-running 0..PERIOD-1 counter; latch phase atomically at the period
    // boundary so a mid-period AXI write takes effect at the next pulse.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter       <= '0;
            phase_latched <= '0;
            p2_end        <= CW'(HALF_PERIOD);
            p2_wrap       <= 1'b0;
        end else if (counter == CW'(PERIOD - 1)) begin
            counter       <= '0;
            phase_latched <= phi;
            p2_end        <= new_p2_end;
            p2_wrap       <= new_p2_wrap;
        end else begin
            counter <= counter + 1'b1;
        end
    end

    // Combinational bridge drives - single-comparator path from counter to p1,
    // two comparators + AND/OR/mux to p2.  No extra register stages.
    assign p1 = (counter < CW'(HALF_PERIOD));
    assign p2 = p2_wrap ? ((counter >= phase_latched) || (counter < p2_end))
                        : ((counter >= phase_latched) && (counter < p2_end));

endmodule
