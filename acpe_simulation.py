import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

st.title("ACPE Sudden Acceleration & Braking Simulation")

st.markdown("""
This simulation models a scenario where the driver of an Ego vehicle accidentally slams the accelerator, causing a sudden acceleration. The ACPE system detects the unintended acceleration and, after a configurable delay, sends a braking command to the chassis. The deceleration is then applied at a fixed rate. You can adjust the following parameters in the sidebar:

- **Distance to obstacle** (0–3 m)
- **ACPE brake command delay** (0–5000 ms)
- **Braking deceleration** (0 to -20 m/s²)
- **Sudden acceleration** (0–10 m/s²)

The time-velocity graph below visualizes the scenario, and the final collision speed (if any) is shown.
""")

# Sliders for parameters in the sidebar
st.sidebar.header("Adjustable Parameters")
distance_to_obstacle = st.sidebar.slider("Distance to obstacle (m)", 0.0, 3.0, 2.0, 0.01)
delay_ms = st.sidebar.slider("ACPE brake command delay (ms)", 0, 5000, 1000, 10)
brake_decel = st.sidebar.slider("Braking deceleration (m/s², negative)", -20.0, 0.0, -8.0, 0.1)
accel = st.sidebar.slider("Sudden acceleration (m/s²)", 0.0, 10.0, 5.0, 0.1)

delay_s = delay_ms / 1000.0

# Simulation parameters
dt = 0.01  # time step (s)
max_time = 10  # max simulation time (s)
time = [0]
velocity = [0]
position = [0]

brake_applied = False
collision = False

while time[-1] < max_time and not collision:
    t = time[-1] + dt
    if t < delay_s:
        a = accel
    else:
        a = brake_decel if velocity[-1] > 0 else 0
    v = max(0, velocity[-1] + a * dt)  # velocity can't go below 0
    s = position[-1] + v * dt
    time.append(t)
    velocity.append(v)
    position.append(s)
    if s >= distance_to_obstacle:
        collision = True
        break

# Plotting
time = np.array(time)
velocity = np.array(velocity)
position = np.array(position)

fig, ax1 = plt.subplots(figsize=(8, 4))
ax1.plot(time, velocity, label="Velocity (m/s)", color="b")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Velocity (m/s)", color="b")
ax1.tick_params(axis='y', labelcolor='b')
ax1.grid(True)

# Mark collision point and annotate collision speed
if collision:
    collision_time = time[position >= distance_to_obstacle][0]
    collision_speed = velocity[position >= distance_to_obstacle][0]
    ax1.axvline(collision_time, color='r', linestyle='--', label='Collision')
    ax1.plot(collision_time, collision_speed, 'ro')
    ax1.annotate(f"Collision\nSpeed: {collision_speed:.2f} m/s",
                 xy=(collision_time, collision_speed),
                 xytext=(collision_time+0.2, collision_speed+0.5),
                 arrowprops=dict(facecolor='red', shrink=0.05),
                 fontsize=11, color='red', bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
    st.warning(f"Collision occurred at {collision_time:.2f} s with speed {collision_speed:.2f} m/s!")
else:
    st.success("No collision within simulation time.")

# Show Chassis Braking Start (should align with velocity turning point)
turning_idx = np.argmax(time >= delay_s)
turning_time = time[turning_idx]
turning_velocity = velocity[turning_idx]
ax1.axvline(turning_time, color='g', linestyle=':', label='Chassis Physical Braking Start')
ax1.annotate("Chassis Braking Start",
             xy=(turning_time, turning_velocity),
             xytext=(turning_time, turning_velocity + 0.5),
             arrowprops=dict(facecolor='green', shrink=0.05),
             fontsize=10, color='green', bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))

ax1.legend()
st.pyplot(fig)

# Show parameter summary
st.markdown(f"**Parameters:**  ")
st.markdown(f"- Distance to obstacle: {distance_to_obstacle} m")
st.markdown(f"- ACPE brake delay: {delay_ms} ms")
st.markdown(f"- Braking deceleration: {brake_decel} m/s²")
st.markdown(f"- Sudden acceleration: {accel} m/s²") 