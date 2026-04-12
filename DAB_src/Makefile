# =============================================================================
# Makefile  –  Verilator build for DAB RTL
#
# Targets
#   all    – verilate + compile  (default)
#   run    – all + execute, writes results.csv
#   clean  – remove build artefacts
#
# RTL parameters (override on the command line, e.g. make run GAMMA=50):
#
#   Physical:  V1=400 V, N=2, L=1 mH, Co=100 µF, R=10 Ω, fsw=10 kHz, φ=π/4
#   Δt = 100 ns,  FRAC = 20 (Q11.20 fixed-point)
#
#   α = Δt/(2L) = 100e-9/(2×1e-3)       = 5.000e-5  → round(×2^20) = 52
#   β = Δt/Co   = 100e-9/100e-6          = 1.000e-3  → round(×2^20) = 1049
#   γ = Δt/(RCo)= 100e-9/(10×100e-6)    = 1.000e-4  → round(×2^20) = 105
#   N           = 2                                  → 2×2^20        = 2097152
#
#   V2_INIT: pre-charge to theoretical V2_ss = 75 V  → round(75×2^20) = 78643200
#   IL_INIT: pre-charge to i_L(t=0) = −8.125 A      → round(×2^20)   = −8519680
#     (Starting from (0,0) gives a ~+6.8 A DC offset in i_L because the ideal
#      inductor has no transformer DC-block; pre-charging eliminates this.)
# =============================================================================

VERILATOR   := verilator
RTL_DIR     := rtl
TB_DIR      := tb
BUILD_DIR   := obj_dir

TOP    := dab_rtl
TB_SRC := $(TB_DIR)/tb_dab.cpp

ALPHA   ?= 52
BETA    ?= 1049
GAMMA   ?= 105
N_RATIO ?= 2097152
V2_INIT ?= 78643200
IL_INIT ?= -8519680

VFLAGS := \
    --cc --exe --build \
    -sv \
    --top-module $(TOP) \
    -Wall \
    -Wno-WIDTHEXPAND \
    -Mdir $(BUILD_DIR) \
    -GALPHA=$(ALPHA) \
    -GBETA=$(BETA) \
    -GGAMMA=$(GAMMA) \
    -GN_RATIO=$(N_RATIO) \
    -GV2_INIT=$(V2_INIT) \
    -GIL_INIT=$(IL_INIT) \
    -CFLAGS "-std=c++17 -O2"

.PHONY: all run clean

all: $(BUILD_DIR)/V$(TOP)

$(BUILD_DIR)/V$(TOP): $(RTL_DIR)/$(TOP).sv $(TB_SRC)
	$(VERILATOR) $(VFLAGS) \
	    $(RTL_DIR)/$(TOP).sv \
	    $(TB_SRC)

run: all
	$(BUILD_DIR)/V$(TOP) > results.csv
	@echo "Wrote results.csv ($$(wc -l < results.csv) lines)"
	@awk -F',' 'NR>1{il=$$4;v2=$$5; if(v2>mx||NR==2)mx=v2; if(v2<mn||NR==2)mn=v2; \
	             if(il>ixl||NR==2)ixl=il; if(il<inl||NR==2)inl=il} \
	  END{printf "  V2:  min=%7.2f V  max=%7.2f V\n  i_L: min=%7.3f A  max=%7.3f A\n", \
	       mn,mx,inl,ixl}' results.csv

clean:
	rm -rf $(BUILD_DIR) results.csv
