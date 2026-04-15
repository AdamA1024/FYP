## ============================================================================
## Buck Verlet Digital Twin — Pin & Timing Constraints
## Board  : AXU5EVB-E (Alinx)
## Device : XCZU5EV-2SFVC784I  (Zynq UltraScale+)
## ============================================================================

## ----------------------------------------------------------------------------
## PL Clock  —  200 MHz differential oscillator
##   PL_CLK0_P  AE5  (sys_clk_p)
##   PL_CLK0_N  AF5  (sys_clk_n)
##   IBUFDS is instantiated in RTL; constrain the ports here.
## ----------------------------------------------------------------------------
set_property PACKAGE_PIN AE5   [get_ports sys_clk_p]
set_property PACKAGE_PIN AF5   [get_ports sys_clk_n]
set_property IOSTANDARD  LVDS  [get_ports sys_clk_p]
set_property IOSTANDARD  LVDS  [get_ports sys_clk_n]

## Primary clock: 200 MHz (5 ns period) on the P pin of the diff pair.
create_clock -period 5.000 -name sys_clk [get_ports sys_clk_p]

## Generated 50 MHz clock from the register divider (CLK_DIV = 4).
## Vivado cannot always trace through a toggle-FF divider automatically;
## declaring it here lets the timing engine analyse clk_50 paths correctly.
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
##   B43_L5_N  AF12  (bank 43, 3.3 V — same bank as PL_LED1)
## ----------------------------------------------------------------------------
set_property PACKAGE_PIN AF12    [get_ports sys_rst_n]
set_property IOSTANDARD  LVCMOS33 [get_ports sys_rst_n]

## ----------------------------------------------------------------------------
## LED  —  PL_LED1  (single bit)
##   B43_L5_P  AE12  (bank 43, 3.3 V)
##   On when v_out >= ~4 V (v_out[26] set).
## ----------------------------------------------------------------------------
set_property PACKAGE_PIN AE12    [get_ports led]
set_property IOSTANDARD  LVCMOS33 [get_ports led]

## ----------------------------------------------------------------------------
## ILA / MARK_DEBUG signals
##   i_out, v_out, and sk_dbg in buck_verlet_top carry MARK_DEBUG="TRUE".
##   Vivado inserts the ILA core and clocks it from clk_50 automatically
##   during implementation.  No explicit pin constraints are needed here.
##
##   Recommended ILA settings (set in IP integrator or Set Up Debug wizard):
##     Capture depth : 4096  →  4096 × 20 ns = 81.9 µs per capture
##     Trigger       : sk_dbg rising edge  (switch-on event)
## ----------------------------------------------------------------------------

## ----------------------------------------------------------------------------
## False paths
##   The reset synchroniser crosses from the async sys_rst_n button into
##   clk_50.  The two-FF synchroniser handles the metastability; tell
##   Vivado not to flag the first stage as a timing violation.
## ----------------------------------------------------------------------------
set_false_path -from [get_ports sys_rst_n] -to [get_clocks clk_50]
