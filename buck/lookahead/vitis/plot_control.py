#!/usr/bin/env python3
"""Plot the closed-loop PI control run captured over UART (BUCK_CTRL block).

Parses only well-formed rows between # BUCK_CTRL_BEGIN / END, so it tolerates any
UART drop. Produces control_run.png:
  top    - Vout tracking Vref, with Vin overlaid (right axis)
  bottom - PWM duty command
"""
import sys
import matplotlib.pyplot as plt

LOG = sys.argv[1] if len(sys.argv) > 1 else "putty.log.txt"

t, vin, vref, vout, duty = [], [], [], [], []
in_block = False
with open(LOG, errors="replace") as fh:
    for line in fh:
        s = line.strip()
        if s.startswith("# BUCK_CTRL_BEGIN"):
            in_block = True; continue
        if s.startswith("# BUCK_CTRL_END"):
            break
        if not in_block or s.startswith("#") or s.startswith("tick,"):
            continue
        f = s.split(",")
        if len(f) != 6:
            continue
        try:
            t.append(float(f[1]) / 1000.0)   # us -> ms
            vin.append(float(f[2])); vref.append(float(f[3]))
            vout.append(float(f[4])); duty.append(float(f[5]))
        except ValueError:
            continue

print(f"parsed {len(t)} control ticks")
if not t:
    sys.exit("no BUCK_CTRL data found in " + LOG)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True,
                               gridspec_kw={"height_ratios": [2, 1]})

ax1.plot(t, vref, "k--", lw=1.2, label="Vref")
ax1.plot(t, vout, "tab:blue", lw=1.2, label="Vout")
ax1.set_ylabel("voltage (V)")
ax1.grid(alpha=0.3)
axr = ax1.twinx()
axr.plot(t, vin, color="tab:red", lw=1.0, alpha=0.6, label="Vin")
axr.set_ylabel("Vin (V)", color="tab:red")
axr.tick_params(axis="y", labelcolor="tab:red")
l1, lab1 = ax1.get_legend_handles_labels()
l2, lab2 = axr.get_legend_handles_labels()
ax1.legend(l1 + l2, lab1 + lab2, loc="upper right", fontsize=9, ncol=3)
ax1.set_title("Closed-loop PI: Vout regulated to Vref through Vin & Vref steps")

ax2.plot(t, duty, "tab:green", lw=1.0)
ax2.set_ylabel("duty (counts)")
ax2.set_xlabel("t (ms)")
ax2.set_ylim(0, 100)
ax2.grid(alpha=0.3)

fig.tight_layout()
fig.savefig("control_run.png", dpi=130)
print("wrote control_run.png")
