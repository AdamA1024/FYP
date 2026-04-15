import os
import numpy as np
import matplotlib.pyplot as plt


def plot_phase(time_us, data, mask, out_png):
    # kept for backward compatibility — delegate to more specific plotters if needed
    fig, axs = plt.subplots(2, 1, sharex=False, figsize=(10, 6))
    if np.any(mask):
        axs[0].plot(time_us[mask], data['i_L_A'][mask], color='C1')
        axs[0].set_ylabel('i_L_A (A)')
        axs[1].plot(time_us[mask], data['V2_V'][mask], color='C2')
        axs[1].set_ylabel('V2_V (V)')
        axs[1].set_xlabel('Time (us)')
        plt.tight_layout()
        plt.savefig(out_png, dpi=300)
        plt.close()
        print(f"Saved plot to {out_png}")
    else:
        print(f"No data in mask for {out_png}")


def plot_iL_snippet(time_us, data, t0_us, t1_us, out_png):
    mask = (time_us >= t0_us) & (time_us <= t1_us)
    if not np.any(mask):
        print(f"No data in {t0_us}-{t1_us}us for i_L_A")
        return
    plt.figure(figsize=(8, 3))
    plt.plot(time_us[mask], data['i_L_A'][mask], color='C1')
    plt.xlim(t0_us, t1_us)
    plt.ylabel('i_L_A (A)')
    plt.xlabel('Time (us)')
    plt.tight_layout()
    plt.savefig(out_png, dpi=300)
    plt.close()
    print(f"Saved i_L snippet to {out_png}")


def plot_V2_phase(time_us, data, phase_col, phase_val, t0_us, t1_us, out_png):
    mask = (time_us >= t0_us) & (time_us <= t1_us) & (data[phase_col] == phase_val)
    if not np.any(mask):
        print(f"No data for {phase_col}=={phase_val} in {t0_us}-{t1_us}us")
        return
    plt.figure(figsize=(10, 3))
    plt.plot(time_us[mask], data['V2_V'][mask], color='C2')
    plt.xlim(t0_us, t1_us)
    plt.ylabel('V2_V (V)')
    plt.xlabel('Time (us)')
    plt.tight_layout()
    plt.savefig(out_png, dpi=300)
    plt.close()
    print(f"Saved V2 phase plot to {out_png}")





def plot_V2_range(time_us, data, t0_us, t1_us, out_png):
    mask = (time_us >= t0_us) & (time_us <= t1_us)
    if not np.any(mask):
        print(f"No data in {t0_us}-{t1_us}us to plot V2_V")
        return
    plt.figure(figsize=(10, 3))
    plt.plot(time_us[mask], data['V2_V'][mask], color='C2')
    plt.xlim(t0_us, t1_us)
    plt.ylabel('V2_V (V)')
    plt.xlabel('Time (us)')
    plt.tight_layout()
    plt.savefig(out_png, dpi=300)
    plt.close()
    print(f"Saved V2 range plot to {out_png}")


def main():
    csv_path = os.path.join("analysis", "dab_results_vivado.csv")

    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    time_us = data['cycle'] / 10.0  # 10 MHz -> 0.1 us per cycle => cycle/10 = time in us
    # --- Graph 1: 50 us snippet for i_L_A from 35000 to 35050 us
    g1_t0 = 35000.0
    g1_t1 = 35050.0
    out_g1 = os.path.join("analysis", "graph1_iL_A_35000-35050us.png")
    # plot_iL_snippet(time_us, data, g1_t0, g1_t1, out_g1)

    # --- Graph 2: first phase of V2_V (phase1) from 0 to 40000 us
    g2_t0 = 0.0
    g2_t1 = 40000.0
    out_g2 = os.path.join("analysis", "graph2_V2_phase1_0-40000us.png")
    # phase column 'p1' equal to 1
    # plot_V2_phase(time_us, data, 'p1', 1, g2_t0, g2_t1, out_g2)



    # determine simulation end time
    t_end = float(np.max(time_us))

    # --- Graph 5: V2_V from 40000 us to end of simulation
    g5_t0 = 40000.0
    g5_t1 = t_end
    out_g5 = os.path.join("analysis", "graph5_V2_40000-endus.png")
    # plot_V2_range(time_us, data, g5_t0, g5_t1, out_g5)

    # --- Graph 6: V2_V in last 1000 us of simulation
    g6_t1 = t_end
    g6_t0 = max(0.0, t_end - 1000.0)
    out_g6 = os.path.join("analysis", "graph6_V2_last1000us.png")
    # plot_V2_range(time_us, data, g6_t0, g6_t1, out_g6)

    # --- Graph 7: V2_V for the last 1000 us up to 40000 us (39000-40000 us)
    g7_t1 = 40000.0
    g7_t0 = max(0.0, g7_t1 - 1000.0)
    out_g7 = os.path.join("analysis", "graph7_V2_39000-40000us.png")
    # plot_V2_range(time_us, data, g7_t0, g7_t1, out_g7)



    plot_V2_range(time_us, data, 0, t_end, os.path.join("analysis", "graph_V2_full_range.png")
                  )
    #plot last 1000 us of V2_V
    plot_V2_range(time_us, data, max(0.0, t_end - 1000.0), t_end, os.path.join("analysis", "graph_V2_last1000us.png")
                  )

if __name__ == '__main__':
    main()
