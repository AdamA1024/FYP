import numpy as np
import matplotlib.pyplot as plt

# Example buck parameters
Vin = 12.0
L = 10e-6
C = 100e-6
R = 5.0

fsw = 100e3
Tsw = 1/fsw
duty = 0.5

dt = 50e-9
t_end = 5e-3

kL = dt / L
kC = dt / C
kR = dt / (R * C)

def pwm_s(t):
    tau = t % Tsw
    return 1.0 if tau < duty * Tsw else 0.0

# --- Step-by-step numerical iterations (first 10 steps) ---
i = 0.0
v = 0.0
steps_to_show = 10
rows = []
for k in range(steps_to_show):
    tk = k * dt
    s = pwm_s(tk)
    i_half = i + (kL/2.0) * (s*Vin - v)
    v_next = v + kC * i_half - kR * v
    i_next = i_half + (kL/2.0) * (s*Vin - v_next)
    rows.append((k, tk, s, i, v, i_half, v_next, i_next))
    i, v = i_next, v_next

print("Verlet-style updates for a buck converter (first 10 steps)")
print(f"Parameters: Vin={Vin} V, L={L} H, C={C} F, R={R} ohm, fsw={fsw} Hz, duty={duty}, dt={dt} s")
print()
header = f"{'k':>2}  {'t (ns)':>8}  {'s':>1}  {'i_k (A)':>10}  {'v_k (V)':>10}  {'i_{k+1/2}':>12}  {'v_{k+1}':>12}  {'i_{k+1}':>10}"
print(header)
print("-"*len(header))
for (k, tk, s, i_k, v_k, i_half, v_next, i_next) in rows:
    print(f"{k:2d}  {tk*1e9:8.1f}  {int(s):1d}  {i_k:10.6f}  {v_k:10.6f}  {i_half:12.6f}  {v_next:12.6f}  {i_next:10.6f}")

# --- Full simulation using the same discrete updates ---
t = np.arange(0, t_end, dt)
n = len(t)
iL = np.zeros(n)
vC = np.zeros(n)

i = 0.0
v = 0.0
for k in range(n):
    tk = t[k]
    s = pwm_s(tk)
    i_half = i + (kL/2.0) * (s*Vin - v)
    v_next = v + kC * i_half - kR * v
    i_next = i_half + (kL/2.0) * (s*Vin - v_next)
    iL[k] = i_next
    vC[k] = v_next
    i, v = i_next, v_next

# Plot output voltage
plt.figure()
plt.plot(t*1e3, vC)
plt.xlabel("Time (ms)")
plt.ylabel("Output voltage vout = vC (V)")
plt.title("Buck output voltage using discrete Verlet-style updates")
plt.tight_layout()
plt.show()

# Zoomed-in plot to see ripple clearly (last 0.2 ms)
zoom_start = t_end - 0.2e-3
mask = t >= zoom_start
plt.figure()
plt.plot(t[mask]*1e3, vC[mask])
plt.xlabel("Time (ms)")
plt.ylabel("Output voltage vout = vC (V)")
plt.title("Zoom: output ripple (last 0.2 ms)")
plt.tight_layout()
plt.show()
