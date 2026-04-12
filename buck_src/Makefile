TOP   = buck_verlet
TB    = sim_main
TB_CTRL = sim_ctrl
SRCS  = buck_params.sv $(TOP).sv ik_half.sv ik_new.sv vk_new.sv
BIN      = obj_dir/V$(TOP)
BIN_CTRL = obj_dir/V$(TOP)_ctrl

.PHONY: all sim ctrl clean plot plot-ctrl

all: sim

# --- Open-loop (Vin-step) simulation ---
sim: $(BIN)
	$(BIN)

$(BIN): $(SRCS) $(TB).cpp sim_params.h
	verilator --cc --exe --build -sv \
	    $(SRCS) $(TB).cpp \
	    --top-module $(TOP) \
	    -Wall -Wno-UNUSEDSIGNAL \
	    -o V$(TOP)

# --- Closed-loop PI controller simulation ---
ctrl: $(BIN_CTRL)
	$(BIN_CTRL)

$(BIN_CTRL): $(SRCS) $(TB_CTRL).cpp sim_params.h
	verilator --cc --exe --build -sv \
	    $(SRCS) $(TB_CTRL).cpp \
	    --top-module $(TOP) \
	    -Wall -Wno-UNUSEDSIGNAL \
	    -o V$(TOP)_ctrl

# --- Plots ---
plot: buck_sim.csv
	python3 plot_buck.py

plot-ctrl: buck_ctrl.csv
	python3 plot_ctrl.py

clean:
	rm -rf obj_dir buck_sim.csv buck_ctrl.csv
