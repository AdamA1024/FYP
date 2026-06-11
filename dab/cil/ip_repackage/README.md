# DAB twin — clean IP source set (+ soft reset pin)

This folder is the **complete, canonical** RTL set to repackage the `dab` AXI IP.
It is the dt=20 ns / 500 kHz design that matches sim (V2 ≈ 16.8 V at V1=100 V,
R=10 Ω, 45° phase). Use it to repackage under a **fresh IP name/version** so
Vivado's IP cache cannot serve the old dt=50 ns netlist again.

## Files (and compile order)

| order | file | role |
|------|------|------|
| 1 | `dab_la_pkg.sv` | coefficient package — **dt=20 ns** (the only `dab_la_pkg`) |
| 2 | `dab4_core.sv` | `dab_look_ahead_solver` (pipelined γ-fold) |
| 3 | `dab_switch_gen.sv` | SPS bridge polarity generator |
| 4 | `dab_top.sv` | `dab_top` — switch-gen + registered p1/p2 + solver |
| 5 | `DAB_PYNQ_slave_lite_v1_0_S00_AXI.v` | AXI4-Lite slave + twin wiring + reset combine |
| 6 | `DAB_PYNQ_slave_lite_v1_0.v` | IP top wrapper |

**Do NOT add** `dab.sv`, `dab2.sv`, `dab3.sv`, `dab3_core.sv` or any second file
that defines `package dab_la_pkg` / `dab_look_ahead_solver`. The 105 V bug was a
stale dt=50 ns `dab_la_pkg` (from `dab3.sv`) being bound to the correct core.
Verify: exactly one `package dab_la_pkg` and one `dab_look_ahead_solver`.

## The reset pin (`aux_reset`)

A single new top-level port, added in 4 lines total:

- **`DAB_PYNQ_slave_lite_v1_0.v`** (IP top): new `input wire aux_reset;` +
  `.aux_reset(aux_reset)` on the slave instance.
- **`..._S00_AXI.v`** (slave): new `input wire aux_reset;` +
  `wire twin_rst_n = S_AXI_ARESETN & ~aux_reset;` feeding `dab_top.rst_n`
  (was `S_AXI_ARESETN`).

Behaviour:
- **Active-high, synchronous** to `s00_axi_aclk`. Pulse high ≥1 clock to reset.
- Resets **only the twin state** — `V2`, `i_L`, the solver pipeline, and the
  switch counter all clear to 0. The AXI config registers (`V1`, `gamma`,
  `phase` = slv_reg0..2) stay on the bus reset, so they **survive** an
  `aux_reset` pulse. This is the clean "re-arm between operating points" the
  firmware previously lacked (DAB has no duty=0 gate like the buck).
- **Unconnected ⇒ 0 ⇒ twin runs** (safe default; won't hold the design in reset).

## Repackage steps (escape the IP cache)

1. **Project Settings → IP → uncheck "Cache IP synthesis results"** (or Clear Cache).
2. Edit/recreate the IP from these 6 files. Bump the IP **version** (e.g. 2.0) or
   give it a **new name** — this changes the cache key.
3. In **Package IP → Ports and Interfaces**, confirm `aux_reset` appears as a new
   port. (Optional: associate it with no interface; leave it a plain port.)
4. **Re-Package IP** → in the block design **Report IP Status → Upgrade Selected**.
5. **Generate Bitstream** → **Export Hardware (include bitstream)** → new `.xsa`.
6. Rebuild the Vitis app against the new platform; reprogram.

## Wiring `aux_reset` in the block design

Two easy options:

- **Software-controllable (recommended):** drop an **AXI GPIO** (1-bit, output
  only), connect `gpio_io_o[0] → aux_reset`. Then from `main.c`:
  ```c
  // pulse the twin reset (XGpio or raw):
  Xil_Out32(GPIO_BASE + 0x00, 1); usleep(1); Xil_Out32(GPIO_BASE + 0x00, 0);
  ```
  Do this before programming a new operating point to start each capture from 0.
- **Tie off (if you don't need it yet):** connect `aux_reset` to a `Constant` =
  0. The pin exists for later; the twin always runs.

## Sanity (already verified in Verilator)

- Full stack lints clean with top = `dab`.
- `dab_top` (this folder) → **V2 = 16.809 V @ 30 ms** at V1=100/R=10/45°.

After reprogramming, the firmware's `gamma rb = 0x00000476` should still read
back correct, and the V2 capture should settle to **~16.8 V** (not 105 V).
