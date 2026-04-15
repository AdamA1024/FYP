import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load data, skipping the radix row
df = pd.read_csv('iladata.csv', header=0, skiprows=[1])

# Each sample is held for 8 samples = 40 ns  →  sample period = 5 ns
T_SAMPLE_NS = 5.0
time_ns = df['Sample in Buffer'] * T_SAMPLE_NS
time_us = time_ns / 1e3

# Q8.24 conversion: interpret hex as signed 32-bit, divide by 2^24
FRAC_BITS = 24

def q8_24(hex_series):
    raw = hex_series.apply(lambda x: int(x, 16)).astype(np.int32)
    return raw.astype(np.float64) / (2 ** FRAC_BITS)

v_out = q8_24(df['v_out[31:0]'])
i_out = q8_24(df['i_out[31:0]'])
sk_dbg = df['sk_dbg']

fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

axes[0].plot(time_us, v_out, color='tab:blue', linewidth=0.8)
axes[0].set_ylabel('V_out (V)')
axes[0].set_title('Buck Converter ILA Capture')
axes[0].grid(True, alpha=0.4)

axes[1].plot(time_us, i_out, color='tab:orange', linewidth=0.8)
axes[1].set_ylabel('I_out (A)')
axes[1].grid(True, alpha=0.4)

axes[2].step(time_us, sk_dbg, color='tab:green', linewidth=0.8, where='post')
axes[2].set_ylabel('sk_dbg')
axes[2].set_xlabel('Time (µs)')
axes[2].grid(True, alpha=0.4)

plt.tight_layout()
plt.savefig('ila_plot.png', dpi=150)
plt.show()
print("Saved ila_plot.png")
