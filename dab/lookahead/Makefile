# Verilator builds for the DAB look-ahead twin (dab4 core)
#
#   make run-1ms    - 1 ms drift check at the controller operating point
#   make run-ctrl   - reproduce the Vitis PI-controller operating point
#   make run-swgen  - switch-pattern generator unit test
#   make run-rstep  - load-step (gamma) response of the bare solver core
#   make gen        - regenerate gen/dab_la_pkg.sv from physical parameters
#   make clean      - remove build artefacts

VERILATOR := verilator

# ── Twin generator: physical design params → RTL coefficient package ────────
TWINGEN = ../../tools/twin_gen.py
L       = 2e-05
CO      = 0.00047
K       = 1.75
DT      = 2e-08
STEPS   = 100
PKG     = gen/dab_la_pkg.sv

CORE = $(PKG) dab4_core.sv dab_switch_gen.sv dab_top.sv

VFLAGS := --cc --exe --build -sv \
          -Wall -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
          -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
          -Wno-EOFNEWLINE -Wno-IMPORTSTAR -Wno-DECLFILENAME \
          -CFLAGS "-std=c++17 -O2"

.PHONY: all gen run-1ms run-ctrl run-swgen run-rstep clean

all: run-1ms

gen:
	@mkdir -p gen
	python3 $(TWINGEN) dab --L $(L) --Co $(CO) --k $(K) --dt $(DT) --steps $(STEPS) \
	        --state-fmt Q8.24 --coef-fmt Q4.28 --out $(PKG)

run-1ms: $(CORE) sim/tb_dab_1ms.cpp
	$(VERILATOR) $(VFLAGS) --top-module dab_top -Mdir obj_dir_1ms $(CORE) sim/tb_dab_1ms.cpp
	obj_dir_1ms/Vdab_top

run-ctrl: $(CORE) sim/tb_dab_ctrl.cpp
	$(VERILATOR) $(VFLAGS) --top-module dab_top -Mdir obj_dir_ctrl $(CORE) sim/tb_dab_ctrl.cpp
	obj_dir_ctrl/Vdab_top

run-swgen: $(PKG) dab_switch_gen.sv sim/tb_switch_gen.cpp
	$(VERILATOR) $(VFLAGS) --top-module dab_switch_gen -Mdir obj_dir_swgen \
	    $(PKG) dab_switch_gen.sv sim/tb_switch_gen.cpp
	obj_dir_swgen/Vdab_switch_gen
	python3 sim/plot_switch_gen.py

run-rstep: $(PKG) dab4_core.sv sim/tb_dab_rstep.cpp
	$(VERILATOR) $(VFLAGS) --top-module dab_look_ahead_solver -Mdir obj_dir_rstep \
	    $(PKG) dab4_core.sv sim/tb_dab_rstep.cpp
	obj_dir_rstep/Vdab_look_ahead_solver

clean:
	rm -rf obj_dir_* *.csv
