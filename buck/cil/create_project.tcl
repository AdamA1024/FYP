# create_project.tcl
# Clean Vivado project for the buck_verlet CIL flow.
#
# Builds a project containing only the AXI4-Lite wrapper + plant.  No ILA,
# no standalone top, no pin constraints.  Synthesis is driven from the IP
# packaged version of buck_axi_wrapper instantiated inside a Block Design
# next to the Zynq UltraScale+ MPSoC IP.
#
# Usage from the Vivado TCL console:
#   cd  <repo>/buck/cil
#   source create_project.tcl
#
# Or batch:
#   vivado -mode batch -source create_project.tcl
#
# After the project opens:
#   1. Tools -> Create and Package New IP -> Package a specified directory
#      Point at this project's rtl/ folder.  Vivado auto-detects s_axi_*,
#      the clock, the reset, and the irq port.  Save the IP into
#      buck/cil/ip_repo so it's reusable.
#   2. File -> Project -> New (or use this same project) -> Create Block Design
#      Add Zynq UltraScale+ MPSoC, run Block Automation, enable PL0_REF_CLK
#      at 50 MHz, drop in the packaged buck_axi_wrapper IP, run Connection
#      Automation, and connect the irq output to a pl_ps_irq line.
#   3. Generate bitstream, export hardware (.xsa), launch Vitis.

set PROJECT_NAME "buck_cil"
set PROJECT_DIR  "[file dirname [info script]]/vivado_proj"
set ROOT_DIR     "[file dirname [info script]]"

# AXU5EVB-E (Alinx) — XCZU5EV-2SFVC784I.
set PART "xczu5ev-sfvc784-2-i"

file mkdir $PROJECT_DIR
create_project $PROJECT_NAME $PROJECT_DIR -part $PART -force

set_property target_language    SystemVerilog [current_project]
set_property simulator_language Mixed         [current_project]

# ── RTL sources ──────────────────────────────────────────────────────────────
add_files -norecurse [list \
    $ROOT_DIR/rtl/buck_params.sv      \
    $ROOT_DIR/rtl/ik_half.sv          \
    $ROOT_DIR/rtl/ik_new.sv           \
    $ROOT_DIR/rtl/vk_new.sv           \
    $ROOT_DIR/rtl/buck_verlet.sv      \
    $ROOT_DIR/rtl/buck_axi_wrapper.sv \
]
set_property file_type SystemVerilog [get_files -of [get_filesets sources_1] *.sv]
set_property top buck_axi_wrapper [current_fileset]

# ── Simulation sources ───────────────────────────────────────────────────────
add_files -fileset sim_1 -norecurse $ROOT_DIR/sim/tb_buck_axi_wrapper.sv
set_property file_type SystemVerilog [get_files -of [get_filesets sim_1] *.sv]
set_property top     tb_buck_axi_wrapper [get_filesets sim_1]
set_property top_lib xil_defaultlib       [get_filesets sim_1]

set_property -name {xsim.simulate.runtime} -value {-all} \
             -objects [get_filesets sim_1]

puts ""
puts "=== buck_cil project created at $PROJECT_DIR ==="
puts "Top: buck_axi_wrapper   Sim top: tb_buck_axi_wrapper"
puts "Next: Tools -> Create and Package New IP, point at rtl/"
