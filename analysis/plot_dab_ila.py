#!/usr/bin/env python3
"""Plot DAB ILA data from iladata_dab1.csv

Creates two figures:
 1) `V2_reg` over time (downsampled by 4 => 20 ns per sample)
 2) A snippet (default 100 us) of `i_L_reg` and, on the same image, `p1` and `p2`

Usage: python analysis/plot_dab_ila.py [csv_path] [snippet_us]
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def hex_to_signed_int(hstr, bits=32):
    v = int(hstr, 16)
    if v & (1 << (bits - 1)):
        v = v - (1 << bits)
    return v


def q_to_float(signed_int, frac_bits=20):
    return signed_int / float(1 << frac_bits)


def load_and_process(csv_path):
    # CSV has a second line describing radixes, skip it
    df = pd.read_csv(csv_path, header=0, skiprows=[1], dtype=str)

    # relevant columns
    col_v2 = 'u_dab/V2_reg[31:0]'
    col_il = 'u_dab/i_L_reg[31:0]'
    col_p1 = 'u_dab/p1'
    col_p2 = 'u_dab/p2'

    # Downsample by 4 (samples repeat every 4 rows -> 20 ns per downsampled point)
    df_ds = df.iloc[::4].reset_index(drop=True)

    # convert hex columns to signed ints then to float using Q11.20 (20 fractional bits)
    v2_vals = df_ds[col_v2].apply(lambda h: q_to_float(hex_to_signed_int(h)))
    il_vals = df_ds[col_il].apply(lambda h: q_to_float(hex_to_signed_int(h)))

    # p1/p2 appear as small hex ints (0/1) — take as unsigned ints
    p1_vals = df_ds[col_p1].apply(lambda h: int(h, 16)).astype(float)
    p2_vals = df_ds[col_p2].apply(lambda h: int(h, 16)).astype(float)

    # time axis in seconds: downsampled rate = 20 ns per sample
    t = np.arange(len(df_ds)) * 20e-9

    return t, v2_vals.to_numpy(), il_vals.to_numpy(), p1_vals.to_numpy(), p2_vals.to_numpy()


def plot_v2(t, v2, out_path=None):
    plt.figure(figsize=(10, 4))
    plt.plot(t * 1e6, v2, linewidth=1)
    plt.xlabel('Time (us)')
    plt.ylabel('V2_reg (Q11.20 -> V)')
    plt.title('V2_reg vs Time')
    plt.grid(True)
    plt.tight_layout()
    if out_path:
        plt.savefig(out_path)
    else:
        plt.show()


def plot_il_and_p(t, il, p1, p2, snippet_us=20.0, out_path=None):
    # snippet_us in microseconds -> convert to seconds
    snippet_s = snippet_us * 1e-6
    max_idx = int(np.searchsorted(t, snippet_s))
    if max_idx <= 0:
        max_idx = len(t)

    tt = t[:max_idx] * 1e6  # use microseconds for x-axis
    il_snip = il[:max_idx]
    p1_snip = p1[:max_idx]
    p2_snip = p2[:max_idx]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    
    # Top subplot: i_L_reg
    ax1.plot(tt, il_snip, label='i_L_reg (A)', color='tab:blue', linewidth=1.5)
    ax1.set_ylabel('i_L_reg (Q11.20 -> A)')
    ax1.grid(True)
    ax1.legend(loc='upper right')
    
    # Bottom subplot: p1 and p2
    ax2.step(tt, p1_snip, where='post', label='p1', color='tab:orange', alpha=0.9, linewidth=1.5)
    ax2.step(tt, p2_snip, where='post', label='p2', color='tab:green', alpha=0.9, linewidth=1.5)
    ax2.set_xlabel('Time (us)')
    ax2.set_ylabel('p1 / p2 (digital)')
    ax2.set_ylim(-0.2, 1.2)
    ax2.grid(True)
    ax2.legend(loc='upper right')

    fig.suptitle(f'i_L_reg + p1/p2 snippet ({snippet_us} us)')
    fig.tight_layout()
    if out_path:
        fig.savefig(out_path)
    else:
        plt.show()


def main():
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name('iladata_dab1.csv')
    snippet_us = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    if not csv_path.exists():
        print(f'CSV file not found: {csv_path}', file=sys.stderr)
        sys.exit(1)

    t, v2, il, p1, p2 = load_and_process(csv_path)

    out_dir = Path.cwd()
    plot_v2(t, v2, out_path=out_dir / 'V2_reg.png')
    plot_il_and_p(t, il, p1, p2, snippet_us=snippet_us, out_path=out_dir / f'iL_p1p2_{int(snippet_us)}us.png')

    print('Saved plots: V2_reg.png, iL_p1p2_{}us.png'.format(int(snippet_us)))


if __name__ == '__main__':
    main()
