import verlet_pkg::*;

module top #(
    parameter int DUTY_W = 32,
    parameter int state   = 18
) (
    input  logic              clk,
    input  logic              rst_n,
    input  logic [state-1:0] Vin,
    input  logic [state-1:0]      kR,
    input  logic [DUTY_W-1:0] duty,
    output logic [state-1:0]   Vout,
    output logic [state-1:0]   Iout
);

    logic s_k;

    pwm_duty_gen #(
        .PWM_PERIOD(100),
        .DUTY_W(DUTY_W)
    ) duty_gen_inst (
        .clk   (clk),
        .rst_n (rst_n),
        .duty  (duty),
        .s_k   (s_k)
    );

    buck_verlet_mixed_precision buck_inst (
        .clk   (clk),
        .rst_n (rst_n),
        .kR    (kR),
        .s_k   (s_k),
        .Vin   (Vin),
        .v_out (Vout),
        .i_out (Iout)
    );

endmodule
