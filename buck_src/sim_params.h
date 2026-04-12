#pragma once
// Centralised parameters for the Verilator testbench (sim_main.cpp).
//
// Physical / switching
//   R    = 5.0 Ω,  L = 10 µH,  C = 100 µF
//   fsw  = 100 kHz  ->  Tsw = 10 µs
//   duty = 0.5      ->  v_out = D*Vin  (varies with Vin steps)
//   dt   = 50 ns    (one clock cycle)

// PWM_PERIOD  : Tsw / dt = 10e-6 / 50e-9 = 200 cycles
// DUTY_CYCLES : duty * PWM_PERIOD = 0.5 * 200 = 100 cycles
static const int PWM_PERIOD  = 200;
static const int DUTY_CYCLES = 100;

// Input voltage in Q8.24: round(Vin * 2^24)
//   Vin = 12.0 V  ->  0x0C000000  (nominal,   v_out = 6.0 V)
//   Vin =  9.0 V  ->  0x09000000  (step down,  v_out = 4.5 V)
//   Vin = 15.0 V  ->  0x0F000000  (step up,    v_out = 7.5 V)
static const int32_t VIN0_FP = 0x0C000000;   // Vin = 12.0 V  (Phase 0 – initial)
static const int32_t VIN1_FP = 0x09000000;   // Vin =  9.0 V  (Phase 1 – step down)
static const int32_t VIN2_FP = 0x0F000000;   // Vin = 15.0 V  (Phase 2 – step up)

// Fixed load conductance g_load = 1/R in Q8.24 = round((1/5.0) * 2^24)
static const int32_t GLOAD_FIXED = 3355443;  // R = 5.0 Ω

// Simulation phases (each 2 ms = 40_000 cycles at dt=50 ns):
//   Phase 0:  cycles [0,           STEP1_CYCLE)  Vin = 12.0 V  (initial convergence)
//   Phase 1:  cycles [STEP1_CYCLE, STEP2_CYCLE)  Vin =  9.0 V  (input voltage step down)
//   Phase 2:  cycles [STEP2_CYCLE, SIM_CYCLES)   Vin = 15.0 V  (input voltage step up)
static const int STEP1_CYCLE = 40000;   // t = 2.0 ms
static const int STEP2_CYCLE = 80000;   // t = 4.0 ms
static const int SIM_CYCLES  = 120000;  // t = 6.0 ms

// LOG_EVERY: write one CSV row every N clock cycles
static const int LOG_EVERY = 10;       // 10 * 50 ns = 500 ns between rows
