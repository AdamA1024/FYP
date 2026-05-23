# =============================================================================
# Makefile  -  Verilator build for the Buck N=3 look-ahead digital twin.
#
# Targets
#   make            verilate + compile + run the look-ahead testbench (default)
#   make build      verilate + compile only (no run)
#   make run        run the compiled testbench (builds first if stale)
#   make clean      remove build artefacts (obj_dir)
#
# The testbench (tb_buck_la.cpp) primes trajectories x1,x2 via the test ports,
# runs the pipeline, and prints committed (v_out, i_out) per cycle.
# =============================================================================

TOP   = buck_lookahead
TB    = tb_buck_la.cpp

# Source order matters: package first, then leaf modules, then the core.
SRCS  = buck_coeffs_pkg.sv fpmul.sv duty_gen.sv $(TOP).sv

BIN   = obj_dir/V$(TOP)

# Verilator treats warnings as fatal by default. The suppressions below are all
# benign-by-design for this RTL (cosmetic / intentional style); drop a -Wno-*
# flag if you want that lint check back:
#   WIDTHEXPAND/WIDTHTRUNC - duty_gen step arithmetic widens to 32b then assigns
#                            back to a 7b counter; values are provably < STEPS.
#   UNUSEDSIGNAL/UNUSEDPARAM - FRAC_S is exported by the package but only FRAC_C
#                              is consumed (by fpmul).
#   EOFNEWLINE  - source files have no trailing newline.
#   IMPORTSTAR  - intentional `import buck_coeffs_pkg::*`.
#   BLKSEQ      - intentional blocking temp (`rounded`) inside fpmul's always_ff.
VFLAGS = --cc --exe --build -sv -j 0 \
         --top-module $(TOP) \
         -Wall \
         -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
         -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
         -Wno-EOFNEWLINE -Wno-IMPORTSTAR -Wno-BLKSEQ

.PHONY: all build run clean

all: run

build: $(BIN)

# Re-verilate whenever any RTL source or the testbench changes.
$(BIN): $(SRCS) $(TB)
	verilator $(VFLAGS) $(SRCS) $(TB)

run: $(BIN)
	$(BIN)

clean:
	rm -rf obj_dir
