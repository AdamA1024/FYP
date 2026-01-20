import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Mass-spring parameters
# -----------------------------
m = 1.0        # mass (kg)
k = 10.0       # spring constant (N/m)

# Simulation parameters
dt = 0.01
t_end = 10.0
t = np.arange(0, t_end, dt)

# Initial conditions
x = 1.0        # initial displacement (m)
v = 0.0        # initial velocity (m/s)

# Storage
x_hist = []
v_hist = []

# -----------------------------
# Velocity Verlet integration
# -----------------------------
for _ in t:
    x_hist.append(x)
    v_hist.append(v)

    # acceleration at current position
    a = -(k / m) * x

    # position update
    x_new = x + v * dt + 0.5 * a * dt**2

    # acceleration at new position
    a_new = -(k / m) * x_new

    # velocity update
    v_new = v + 0.5 * (a + a_new) * dt

    x, v = x_new, v_new

x_hist = np.array(x_hist)
v_hist = np.array(v_hist)

# -----------------------------
# Plots
# -----------------------------
plt.figure()
plt.plot(t, x_hist)
plt.xlabel("Time (s)")
plt.ylabel("Position x (m)")
plt.title("Mass-spring oscillator: position vs time (velocity Verlet)")
plt.tight_layout()
plt.show()

plt.figure()
plt.plot(t, v_hist)
plt.xlabel("Time (s)")
plt.ylabel("Velocity v (m/s)")
plt.title("Mass-spring oscillator: velocity vs time (velocity Verlet)")
plt.tight_layout()
plt.show()
