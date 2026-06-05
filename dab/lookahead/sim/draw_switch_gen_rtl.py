#!/usr/bin/env python3
# Render an RTL block diagram of dab_switch_gen.sv (SPS bridge polarity gen).
# Pure matplotlib (no graphviz). Left = sequential (clocked), right =
# combinational low-latency path to the p1/p2 polarity outputs.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle

C_FF  = "#cfe2f3"    # registers
C_ALU = "#d9ead3"    # arithmetic / compare
C_MUX = "#fce5cd"    # muxes
C_RGN = "#999999"    # region outline

fig, ax = plt.subplots(figsize=(19, 11))
ax.set_xlim(0, 18.8); ax.set_ylim(1.4, 10.8)
ax.axis("off")


def box(cx, cy, w, h, label, fc=C_ALU, fs=9):
    ax.add_patch(FancyBboxPatch((cx - w/2, cy - h/2), w, h,
                 boxstyle="round,pad=0.02,rounding_size=0.06",
                 fc=fc, ec="black", lw=1.4, zorder=3))
    ax.text(cx, cy, label, ha="center", va="center", fontsize=fs, zorder=4)


def mux_lr(cx, cy, w, hh, sel_lbl=""):     # inputs left (tall), output right (point)
    pts = [(cx-w/2, cy-hh), (cx-w/2, cy+hh),
           (cx+w/2, cy+hh*0.42), (cx+w/2, cy-hh*0.42)]
    ax.add_patch(Polygon(pts, closed=True, fc=C_MUX, ec="black", lw=1.4, zorder=3))
    ax.text(cx-w/2+0.22, cy, "MUX", ha="center", va="center", fontsize=6.5,
            rotation=90, zorder=4)
    if sel_lbl:
        ax.text(cx-0.05, cy+hh*0.62, sel_lbl, ha="center", va="bottom",
                fontsize=7.5, zorder=4)
    return (cx+w/2, cy)


def mux_td(cx, cy, w, hh, sel_lbl=""):     # inputs top (wide), output bottom (point)
    pts = [(cx-w/2, cy+hh), (cx+w/2, cy+hh),
           (cx+w/2*0.42, cy-hh), (cx-w/2*0.42, cy-hh)]
    ax.add_patch(Polygon(pts, closed=True, fc=C_MUX, ec="black", lw=1.4, zorder=3))
    ax.text(cx, cy, "MUX", ha="center", va="center", fontsize=6.5, zorder=4)
    if sel_lbl:
        ax.text(cx+w/2*0.55, cy, sel_lbl, ha="left", va="center", fontsize=7.5, zorder=4)
    return (cx, cy-hh)


def wire(pts, color="black", lw=1.3, head=True, ls="-"):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs[:-1], ys[:-1], color=color, lw=lw, ls=ls,
            solid_capstyle="round", zorder=2)
    if head:
        ax.annotate("", xy=pts[-1], xytext=pts[-2],
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=lw), zorder=2)
    else:
        ax.plot(xs[-2:], ys[-2:], color=color, lw=lw, ls=ls, zorder=2)


def dot(x, y):
    ax.plot(x, y, "o", ms=5, color="black", zorder=5)


def clk_tri(cx, cy):
    ax.add_patch(Polygon([(cx-0.13, cy+0.14), (cx-0.13, cy-0.14), (cx, cy)],
                 closed=True, fc="white", ec="black", lw=1.1, zorder=4))

# ── region frames ────────────────────────────────────────────────────────────
ax.add_patch(Rectangle((1.0, 1.7), 6.4, 8.7, fill=False, ec=C_RGN, lw=1.4, ls="--"))
ax.text(1.15, 10.2, "SEQUENTIAL  (updates on posedge clk)",
        fontsize=10, color=C_RGN, style="italic")
ax.add_patch(Rectangle((7.7, 1.7), 11.0, 6.0, fill=False, ec=C_RGN, lw=1.4, ls="--"))
ax.text(7.85, 7.5, "COMBINATIONAL  (fresh p1/p2 every clock — no output register)",
        fontsize=10, color=C_RGN, style="italic")

# ── primary inputs ───────────────────────────────────────────────────────────
ax.text(0.05, 9.15, "phase_shift\n[31:0]", fontsize=8.5, va="center", weight="bold")
ax.text(0.05, 2.55, "clk",   fontsize=9, va="center", weight="bold")
ax.text(0.05, 2.15, "rst_n", fontsize=8, va="center", color="0.4")

# ── sequential blocks ────────────────────────────────────────────────────────
box(3.3, 8.9, 2.6, 0.95, "phase_latched\n[31:0]", C_FF, fs=8.5); clk_tri(2.18, 8.55)
box(3.3, 4.4, 2.6, 0.95, "cnt [31:0]", C_FF, fs=9);             clk_tri(2.18, 4.05)
box(6.1, 5.5, 0.95, 0.7, "+1", C_ALU)
box(6.1, 3.1, 1.7, 0.8, "= CNT_MAX", C_ALU, fs=8.5)
mux_td(3.3, 6.6, 1.9, 0.5, sel_lbl="")     # next-cnt mux (output bottom -> cnt.D)
ax.text(2.55, 7.25, "0",      fontsize=8.5, ha="center")
ax.text(4.05, 7.25, "cnt+1",  fontsize=8.5, ha="center")
ax.text(2.20, 6.6,  "wrap", fontsize=7.5, ha="right", color="tab:red")

# phase_shift -> phase_latched D
wire([(0.95, 9.1), (2.0, 9.1)])
# clk distribution
wire([(0.55, 2.45), (0.85, 2.45), (0.85, 8.55), (2.05, 8.55)], color="0.3", lw=1.1)
dot(0.85, 4.05); wire([(0.85, 4.05), (2.05, 4.05)], color="0.3", lw=1.1)

# counter feedback: cnt -> +1 -> mux(cnt+1 / 0) -> cnt.D ; cnt -> ==CNT_MAX -> wrap
dot(5.3, 4.4)
wire([(4.6, 4.4), (5.3, 4.4)], head=False)         # cnt out to junction J1
wire([(5.3, 4.4), (5.3, 5.5), (5.625, 5.5)])       # up to +1
wire([(5.3, 4.4), (5.3, 3.1), (5.25, 3.1)])        # down to ==CNT_MAX
wire([(6.575, 5.5), (6.575, 7.45), (4.05, 7.45), (4.05, 7.1)])   # +1 -> mux input
wire([(2.55, 7.45), (2.55, 7.1)])                  # const 0 -> mux input
wire([(3.3, 6.1), (3.3, 4.875)])                   # mux out -> cnt.D
# wrap rail (red): ==CNT_MAX -> mux sel and phase_latched EN
dot(1.35, 6.6)
wire([(6.95, 3.1), (7.2, 3.1), (7.2, 2.15), (1.35, 2.15), (1.35, 6.6), (2.7, 6.6)],
     color="tab:red")
wire([(1.35, 6.6), (1.35, 8.55), (2.0, 8.55)], color="tab:red")
ax.text(2.05, 8.72, "EN", fontsize=7, color="tab:red")
ax.text(4.0, 1.95, "wrap = (cnt == CNT_MAX)  → reset cnt to 0, latch new phase",
        fontsize=7.5, color="tab:red", ha="center")

# ── combinational region ─────────────────────────────────────────────────────
# cnt bus enters region B, fans out vertically (x=8.0); phase bus at x=8.45
dot(8.0, 4.4)
wire([(5.3, 4.4), (8.0, 4.4)], head=False)         # cnt into region B
wire([(8.0, 4.4), (8.0, 6.85)], head=False)        # cnt rail up
wire([(8.0, 4.4), (8.0, 3.2)],  head=False)        # cnt rail down
dot(8.0, 5.8); dot(8.0, 3.2); dot(8.0, 6.85)
# phase_latched out -> phase rail
wire([(4.6, 8.9), (8.45, 8.9), (8.45, 2.8)], head=False)
dot(8.45, 5.4); dot(8.45, 4.1); dot(8.45, 2.8)
ax.text(8.6, 8.75, "phase_latched", fontsize=7, color="0.3")

# arithmetic blocks
box(9.7, 5.6, 1.75, 0.9, "cnt ≥\nphase", C_ALU, fs=8.5)
box(9.7, 4.3, 1.75, 0.8, "cnt − phase", C_ALU, fs=8)
box(9.7, 3.0, 1.75, 0.8, "cnt+P\n− phase", C_ALU, fs=8)
# cnt + phase feeds
wire([(8.0, 5.8), (8.825, 5.8)])                   # cnt -> ge (upper-left)
wire([(8.45, 5.4), (8.825, 5.4)])                  # phase -> ge (lower-left)
wire([(8.0, 4.5), (8.825, 4.5)]); wire([(8.45, 4.1), (8.825, 4.1)])   # -> sub1
wire([(8.0, 3.2), (8.825, 3.2)]); wire([(8.45, 2.8), (8.825, 2.8)])   # -> sub2

# p2_pos mux (sel = ge)
o_mux2 = mux_lr(11.7, 3.65, 0.95, 0.85, sel_lbl="ge")
wire([(10.575, 4.3), (10.95, 4.3), (10.95, 4.0), (11.225, 4.0)])      # sub1 -> mux top
wire([(10.575, 3.0), (10.95, 3.0), (10.95, 3.3), (11.225, 3.3)])      # sub2 -> mux bot
wire([(10.575, 5.6), (11.65, 5.6), (11.65, 4.5)], color="tab:purple") # ge -> sel
ax.text(10.62, 5.72, "ge", fontsize=7, color="tab:purple")
ax.text(12.25, 3.95, "p2_pos", fontsize=7, color="0.3")

# p2_pos < HALF
box(13.6, 3.65, 1.75, 0.8, "p2_pos\n< HALF", C_ALU, fs=8)
wire([(o_mux2[0], 3.65), (12.725, 3.65)])

# cnt < HALF (p1 path)
box(13.6, 6.4, 1.75, 0.8, "cnt < HALF", C_ALU, fs=8.5)
wire([(8.0, 6.85), (12.725, 6.85), (12.725, 6.4)])

# output muxes: p1 / p2  (data = POS/NEG constants, sel = lt1/lt2)
ax.text(15.0, 6.95, "POS=+1", fontsize=7, ha="center")
ax.text(15.0, 5.85, "NEG=−1", fontsize=7, ha="center")
o_p1 = mux_lr(15.7, 6.4, 0.95, 0.85, sel_lbl="lt1")
wire([(15.0, 6.85), (15.225, 6.75)]); wire([(15.0, 5.95), (15.225, 6.05)])
wire([(14.475, 6.4), (14.85, 6.4), (14.85, 7.25), (15.65, 7.25), (15.65, 6.95)],
     color="tab:purple")
ax.text(14.5, 6.5, "lt1", fontsize=7, color="tab:purple")

ax.text(15.0, 4.2, "POS=+1", fontsize=7, ha="center")
ax.text(15.0, 3.1, "NEG=−1", fontsize=7, ha="center")
o_p2 = mux_lr(15.7, 3.65, 0.95, 0.85, sel_lbl="lt2")
wire([(15.0, 4.1), (15.225, 4.0)]); wire([(15.0, 3.2), (15.225, 3.3)])
wire([(14.475, 3.65), (14.85, 3.65), (14.85, 4.55), (15.65, 4.55), (15.65, 4.2)],
     color="tab:purple")
ax.text(14.5, 3.75, "lt2", fontsize=7, color="tab:purple")

# outputs
wire([o_p1, (18.25, 6.4)]); ax.text(18.3, 6.4, "p1[1:0]\n(primary)",
                                    fontsize=9, va="center", weight="bold")
wire([o_p2, (18.25, 3.65)]); ax.text(18.3, 3.65, "p2[1:0]\n(secondary)",
                                     fontsize=9, va="center", weight="bold")

# footer note
ax.text(0.05, 1.5,
        "POS = 2'sb01 (+1)   NEG = 2'sb11 (−1)   |   "
        "HALF = PWM_PERIOD/2 (50% duty)   CNT_MAX = PWM_PERIOD−1   |   "
        "P = PWM_PERIOD.  Only cnt & phase_latched are flip-flops.",
        fontsize=8, color="0.25")

ax.set_title("dab_switch_gen.sv — RTL block diagram (SPS bridge polarity generator)",
             fontsize=13, weight="bold")
fig.tight_layout()
fig.savefig("switch_gen_rtl.png", dpi=120)
print("wrote switch_gen_rtl.png")
