## ============================================================================
## DAB Verlet Digital Twin (ILA build) — Pin & Timing Constraints
## Board  : AXU5EVB-E (Alinx)
## Device : XCZU5EV-2SFVC784I  (Zynq UltraScale+)
## Top    : dab_rtl_top  (with dab_rtl_ila.sv for ILA-marked signals)
##
## Clock chain (identical to buck_verlet_top):
##   200 MHz diff OSC  →  CLK_DIV = 2  →  50 MHz  (Δt = 20 ns per Verlet step)
##   f_out = f_sys / (2 × CLK_DIV) = 200 / 4 = 50 MHz
## ============================================================================

## ----------------------------------------------------------------------------
## PL Clock  —  200 MHz differential oscillator
##   PL_CLK0_P  AE5  (sys_clk_p)
##   PL_CLK0_N  AF5  (sys_clk_n)
##   IBUFDS is instantiated in dab_rtl_top; constrain the ports here.
## ----------------------------------------------------------------------------
set_property PACKAGE_PIN AE5   [get_ports sys_clk_p]
set_property PACKAGE_PIN AF5   [get_ports sys_clk_n]
set_property IOSTANDARD  LVDS  [get_ports sys_clk_p]
set_property IOSTANDARD  LVDS  [get_ports sys_clk_n]

## Primary clock: 200 MHz (5 ns period) on the P pin of the diff pair.
create_clock -period 5.000 -name sys_clk [get_ports sys_clk_p]

## Generated 50 MHz clock from the register divider (CLK_DIV = 2).
##   Effective divide ratio = 2 × CLK_DIV = 4  →  200 / 4 = 50 MHz (20 ns).
##   One Verlet timestep per cycle  →  fsw = 1 / (2 × 50 × 20 ns) = 500 kHz.
create_generated_clock -name clk_50 \
    -source [get_ports sys_clk_p]   \
    -divide_by 4                    \
    [get_pins gen_clkdiv.clk_50_reg/Q]

## The divider output drives only PL fabric; treat sys_clk and clk_50 as
## asynchronous groups so cross-domain paths are flagged, not analysed.
set_clock_groups -asynchronous \
    -group [get_clocks sys_clk] \
    -group [get_clocks clk_50]

## ----------------------------------------------------------------------------
## Reset button  —  PL_KEY1, active-low
##   B43_L5_N  AF12  (bank 43, 3.3 V)
## ----------------------------------------------------------------------------
set_property PACKAGE_PIN AF12     [get_ports sys_rst_n]
set_property IOSTANDARD  LVCMOS33 [get_ports sys_rst_n]

## ----------------------------------------------------------------------------
## Status LED  —  PL_LED1
##   B43_L5_P  AE12  (bank 43, 3.3 V)
##   LED on when V2_out[26] = 1, i.e. V2 ≥ 64 V (Q11.20).
##   At steady-state V2 ≈ 77 V  →  LED on during normal operation.
## ----------------------------------------------------------------------------
set_property PACKAGE_PIN AE12     [get_ports led]
set_property IOSTANDARD  LVCMOS33 [get_ports led]

## ----------------------------------------------------------------------------
## ILA / MARK_DEBUG signals
##
##   The following nets carry (* mark_debug = "true" *) in dab_rtl_ila.sv and
##   are inserted into the ILA core automatically by Vivado's Set Up Debug flow.
##
##     p1        — primary bridge polarity    (1-bit)
##     p2        — secondary bridge polarity  (1-bit)
##     i_L_reg   — inductor current register  (32-bit Q11.20)
##     V2_reg    — output voltage register    (32-bit Q11.20)
##     i_L_half  — Verlet half-step current   (32-bit Q11.20, kept by synthesis)
##     V2_nd     — undamped V2 node           (32-bit Q11.20, kept by synthesis)
##
##   No explicit pin constraints are needed — Vivado wires these to the ILA
##   automatically during implementation when mark_debug attributes are present.
##
##   Recommended ILA settings (Set Up Debug wizard):
##     Clock        : clk_50  (the 50 MHz divided clock)
##     Capture depth: 8192    →  8192 × 20 ns = 163.8 µs per capture
##                             ≈ 82 complete switching periods at fsw = 500 kHz
##     Trigger      : p1 rising edge  (aligned to switching cycle boundary)
## ----------------------------------------------------------------------------

## ----------------------------------------------------------------------------
## False paths
##   The reset synchroniser crosses from the async sys_rst_n button into
##   clk_50.  The two-FF synchroniser handles metastability; suppress the
##   first-stage timing violation here.
## ----------------------------------------------------------------------------
set_false_path -from [get_ports sys_rst_n] -to [get_clocks clk_50]
