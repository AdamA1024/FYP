// PWM duty-cycle generator.
//
// Free-running modulo-PWM_PERIOD counter compared against `duty`.  Lets the
// AXI side write `duty` once per setpoint change instead of toggling s_k
// every clock.
//
// Convention: s_k is high while cnt < duty, so
//   duty = 0           -> s_k always low  (0% duty)
//   duty = PWM_PERIOD  -> s_k always high (100% duty)
//   duty = N           -> N high clocks per PWM_PERIOD-clock period
//
// Default PWM_PERIOD=100 matches the project's 100 MHz clk / 1 MHz f_sw.

module pwm_duty_gen #(
    parameter int PWM_PERIOD = 100,
    parameter int DUTY_W     = 32
) (
    input  logic              clk,
    input  logic              rst_n,
    input  logic [DUTY_W-1:0] duty,
    output logic              s_k
);
    logic [DUTY_W-1:0] cnt;
    localparam logic [DUTY_W-1:0] CNT_MAX = DUTY_W'(PWM_PERIOD - 1);

    always_ff @(posedge clk) begin
        if (!rst_n)            cnt <= '0;
        else if (cnt == CNT_MAX) cnt <= '0;
        else                   cnt <= cnt + 1'b1;
    end

    assign s_k = (cnt < duty);
endmodule
