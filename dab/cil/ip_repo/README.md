# DAB CIL packaged IP

`DAB_PYNQ_1_0/` is a self-contained Vivado IP (VLNV `xilinx.com:user:DAB_PYNQ:1.0`)
that wraps the dab4 look-ahead core (see [../../lookahead/](../../lookahead/))
behind an AXI4-Lite slave for controller-in-loop use on the Zynq PS.

## Using it

1. Vivado -> Settings -> IP -> Repository -> add this `ip_repo/` folder.
2. The IP appears in the catalog as **DAB_PYNQ (1.0)**; drop it into a block
   design alongside the Zynq PS and connect its AXI4-Lite port.

The core HDL (`dab4_core.sv`, `dab_la_pkg.sv`, `dab_switch_gen.sv`,
`dab_top.sv`) is bundled under `DAB_PYNQ_1_0/src/` and `component.xml`
references local paths only, so the folder is portable on its own. The sibling
[../ip_repackage/](../ip_repackage/) holds the same core plus notes on how the
IP was assembled. Auto-generated `example_designs/` and the bare-metal
`drivers/` were left out; regenerate them from Vivado if needed.
