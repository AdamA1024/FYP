# Buck CIL packaged IP

`buck_Final/` is a self-contained Vivado IP (VLNV `xilinx.com:user:buck:Final`)
that wraps the buck look-ahead Verlet core (see [../../lookahead/](../../lookahead/))
behind an AXI4-Lite slave for controller-in-loop use on the Zynq PS.

## Using it

1. Vivado -> Settings -> IP -> Repository -> add this `ip_repo/` folder.
2. The IP appears in the catalog as **buck (Final)**; drop it into a block
   design alongside the Zynq PS and connect its AXI4-Lite port.

All HDL the IP needs is bundled under `buck_Final/src/` and `buck_Final/hdl/`
(`component.xml` references local paths only), so the folder is portable on its
own. Auto-generated `example_designs/` and the bare-metal `drivers/` were left
out; regenerate them from Vivado if needed.
