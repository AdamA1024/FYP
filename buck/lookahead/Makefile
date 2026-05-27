# Verilator build for buck_verlet_mixed_precision
TOP   = buck_verlet_mixed_precision
TB    = sim_main.cpp
SRCS  = $(TOP).sv

# ── Twin generator: physical design params → RTL coefficient package ─────────
# Override on the command line, e.g.:  make gen-run L=4.7e-6 C=2.2e-6 DT=10e-9
TWINGEN  = ../../tools/twin_gen.py
L        = 2.25e-6
C        = 1.125e-6
DT       = 10e-9
STEPS    = 100
KR_SCALE = 3
SFMT     = Q6.12
CFMT     = Q2.16
GEN_PKG  = gen/verlet_pkg.sv
GEN_SRCS = $(GEN_PKG) buck_core.sv
GEN_DIR  = obj_dir_gen
GEN_BIN  = $(GEN_DIR)/V$(TOP)

BIN       = obj_dir/V$(TOP)
STEP_TB   = sim_step_main.cpp
STEP_DIR  = obj_dir_step
STEP_BIN  = $(STEP_DIR)/V$(TOP)

VFLAGS = --cc --exe --build --trace -sv -j 0 \
         --top-module $(TOP) \
         -Wall \
         -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
         -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
         -Wno-EOFNEWLINE -Wno-IMPORTSTAR -Wno-DECLFILENAME

.PHONY: all build run step step-build step-run gen gen-build gen-run clean

all: run

# ── Generated-coefficient flow (prototyping): regenerate the package from
#    physical params, then build/run the SAME testbench against buck_core.sv ──
# Params are Make variables, so always regenerate (codegen is sub-second).
gen: FORCE
	@mkdir -p gen
	python3 $(TWINGEN) buck --L $(L) --C $(C) --dt $(DT) --steps $(STEPS) \
	        --kr-scale $(KR_SCALE) --state-fmt $(SFMT) --coef-fmt $(CFMT) --out $(GEN_PKG)

gen-build: gen
	verilator $(VFLAGS) --Mdir $(GEN_DIR) $(GEN_SRCS) $(TB)

gen-run: gen-build
	$(GEN_BIN)

FORCE:

build: $(BIN)

$(BIN): $(SRCS) $(TB)
	verilator $(VFLAGS) $(SRCS) $(TB)

run: $(BIN)
	$(BIN)

# Step-response testbench (separate obj dir so it doesn't fight the main TB)
step: step-run

step-build: $(STEP_BIN)

$(STEP_BIN): $(SRCS) $(STEP_TB)
	verilator $(VFLAGS) --Mdir $(STEP_DIR) $(SRCS) $(STEP_TB)

step-run: $(STEP_BIN)
	$(STEP_BIN)

clean:
	rm -rf obj_dir $(STEP_DIR) $(GEN_DIR) simulation_trace.vcd trace.csv \
	       step_trace.vcd step_trace.csv
