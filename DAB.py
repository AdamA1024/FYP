import numpy as np
import matplotlib.pyplot as plt

V1 = 48.0
k = 1.75
L = 20e-6
Co = 470e-6
Rload = 10.0

fsw = 100e3
Tsw = 1 / fsw
phi_deg = 90.0
phi = phi_deg * np.pi/180.0
tshift = (phi / (2*np.pi)) * Tsw

dt = 50e-9
t_end = 3e-3
t = np.arange(0, t_end, dt)
n = len(t)

alpha = dt / (2*L)
beta = dt / Co
gamma = dt / (Rload * Co)

def p_square(time, shift=0.0):
    tau = (time - shift) % Tsw
    return 1.0 if tau < (Tsw/2) else -1.0

i = 0.0
V2 = 0.0

iL = np.zeros(n)
V2_series = np.zeros(n)
v1_series = np.zeros(n)
v2p_series = np.zeros(n)
vL_series = np.zeros(n)
p1_series = np.zeros(n)
p2_series = np.zeros(n)

for idx, tk in enumerate(t):
    p1 = p_square(tk, 0.0)
    p2 = p_square(tk, tshift)

    vL_k = p1*V1 - k*p2*V2
    i_half = i + alpha * vL_k
    V2_next = V2 + beta*(k*p2*i_half) - gamma*V2
    vL_k2 = p1*V1 - k*p2*V2_next
    i_next = i_half + alpha * vL_k2

    p1_series[idx] = p1
    p2_series[idx] = p2
    v1_series[idx] = p1*V1
    v2p_series[idx] = k*p2*V2
    vL_series[idx] = vL_k
    iL[idx] = i_next
    V2_series[idx] = V2_next

    i, V2 = i_next, V2_next

t0 = t_end - 5*Tsw
t1 = t0 + Tsw
mask = (t >= t0) & (t < t1)

t_win = t[mask]
t_rel = (t_win - t0) * 1e6

i_win = iL[mask]
vL_win = vL_series[mask]
v1_win = v1_series[mask]
v2p_win = v2p_series[mask]
p1_win = p1_series[mask]
p2_win = p2_series[mask]
V2_end = V2_series[np.where(mask)[0][-1]]

print(f"Zoom window: one switching period starting at t0={t0*1e3:.3f} ms")
print(f"fsw={fsw/1e3:.1f} kHz, Tsw={Tsw*1e6:.2f} µs, dt={dt*1e9:.1f} ns")
print(f"Approx V2 during this window: ~{V2_end:.2f} V (so kV2 ~ {k*V2_end:.2f} V)")

plt.figure()
plt.plot(t_rel, i_win)
plt.xlabel("Time within one switching period (µs)")
plt.ylabel("Inductor current iL (A)")
plt.title("DAB: iL over one switching period (piecewise linear)")
plt.tight_layout()
plt.show()

plt.figure()
plt.plot(t_rel, v1_win, label="v1 = p1*V1")
plt.plot(t_rel, v2p_win, label="v2' = k*p2*V2 (referred)")
plt.plot(t_rel, vL_win, label="vL = v1 - v2'")
plt.xlabel("Time within one switching period (µs)")
plt.ylabel("Voltage (V)")
plt.title("DAB: v1, v2', and vL over the same period")
plt.legend()
plt.tight_layout()
plt.show()

plt.figure()
plt.plot(t_rel, p1_win, label="p1")
plt.plot(t_rel, p2_win, label="p2")
plt.xlabel("Time within one switching period (µs)")
plt.ylabel("Polarity (+1 / -1)")
plt.title("DAB: bridge polarities p1 and p2 (phase shifted)")
plt.legend()
plt.tight_layout()
plt.show()
