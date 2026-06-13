# FPGA-Based Real-Time Digital Twin of Power Converters

Final-year project: real-time digital twins of two DC-DC converters — a
synchronous buck and a dual active bridge (DAB) — running on a Zynq
UltraScale+ MPSoC (Alinx AXU5EVB-E). The converter state equations are
integrated with a symplectic velocity-Verlet scheme in fixed point, so the
twin advances in lock-step with real time and can sit in the loop with a
controller running on the processing system (controller-in-loop, CIL).

Two solver architectures are implemented and compared:

- **Single-cycle**: one Verlet integration step per FPGA clock
  (100 steps per switching period).
- **Look-ahead**: per-gate-state update matrices are precomputed and folded
  into coefficient packages, letting a deeper-pipelined core meet timing
  while preserving the single-cycle trajectory.

Every RTL model is validated three ways: against a Python golden model in
Verilator, on hardware via ILA capture, and end-to-end in CIL with a PI
controller on the PS (DAB hardware matches the twin with mean |err| ≈ 50 mV
at the controller operating point).

## Repository layout

```
models/                 Python reference models (buck, DAB, mass-spring Verlet study)
tools/twin_gen.py       physical params (L, C, dt, ...) -> RTL coefficient package
buck/
  single_cycle/         Q8.24 Verlet core + Verilator testbenches (open/closed loop)
    fpga/               board top + constraints for ILA validation
  cil/                  AXI-Lite wrapper, Vivado project script, Vitis PI firmware
  lookahead/            mixed-precision look-ahead core (Q6.12/Q2.16) + sims + firmware
dab/
  single_cycle/         dab_rtl + phase-shift generator + self-checking testbench
    fpga/               ILA variant, constraints, AXI-Lite slave
  lookahead/            dab4 core + switch generator + drift/ctrl/R-step testbenches
  cil/                  packaged AXI-Lite IP + Vitis PI phase-shift firmware
analysis/               plot scripts for ILA exports and sim/hardware comparisons
docs/references.md      literature underpinning the method
```

## Running the simulations

Prerequisites: Verilator ≥ 5.0, Python 3 with numpy/matplotlib, GNU make.
The board flow additionally needs Vivado/Vitis.

```sh
# Buck, single-cycle: open-loop Vin step / closed-loop PI
make -C buck/single_cycle sim
make -C buck/single_cycle ctrl

# DAB, single-cycle: self-checking run (steady-state V2 vs phasor model)
make -C dab/single_cycle run

# Buck, look-ahead: regenerate coefficients and run; step response
make -C buck/lookahead gen-run
make -C buck/lookahead step

# DAB, look-ahead: drift check, controller op-point, switch-gen, load step
make -C dab/lookahead run-1ms
make -C dab/lookahead run-ctrl
make -C dab/lookahead run-swgen
make -C dab/lookahead run-rstep
```

Coefficient packages under `*/gen/` are generated — regenerate with
`make gen` in the respective directory or directly:

```sh
python3 tools/twin_gen.py dab --L 2e-05 --Co 0.00047 --k 1.75 \
        --dt 2e-08 --steps 100 --state-fmt Q8.24 --coef-fmt Q4.28
```

## Hardware (CIL) flow

- **Buck**: `buck/cil/create_project.tcl` rebuilds the Vivado project around
  `buck_axi_wrapper`; flash the bitstream and run
  `buck/cil/vitis/buck_ctrl/src/main.c` on the PS.
- **DAB**: package `dab/cil/ip_repackage/` as an AXI-Lite IP (see its
  README), drop it into a block design, and run
  `dab/cil/vitis/dab_ctrl/src/main.c`. UART dumps are parsed and plotted by
  the scripts in `analysis/`.
