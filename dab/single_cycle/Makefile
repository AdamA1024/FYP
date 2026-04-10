# Verilator build for dab_top + phaseGen + dab_rtl
#
#   make        - build Vdab_top
#   make run    - build, run, write results.csv, print steady-state summary
#   make clean  - remove build artefacts

VERILATOR := verilator
BUILD_DIR := obj_dir
TOP       := dab_top
TB        := tb/tb_dab_top.cpp
RTL       := dab_top.sv phaseGen.sv dab_rtl.sv

VFLAGS := --cc --exe --build -sv \
          --top-module $(TOP) \
          -Wall -Wno-WIDTHEXPAND -Wno-UNUSEDSIGNAL \
          -Mdir $(BUILD_DIR) \
          -CFLAGS "-std=c++17 -O2"

.PHONY: all run clean

all: $(BUILD_DIR)/V$(TOP)

$(BUILD_DIR)/V$(TOP): $(RTL) $(TB)
	$(VERILATOR) $(VFLAGS) $(RTL) $(TB)

run: all
	$(BUILD_DIR)/V$(TOP) > results.csv
	@echo "Wrote results.csv ($$(wc -l < results.csv) lines)"

clean:
	rm -rf $(BUILD_DIR) results.csv
