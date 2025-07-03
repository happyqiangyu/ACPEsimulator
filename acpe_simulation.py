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
delay_ms = st.sidebar.slider("ACPE brake command delay (ms)", 0, 5000, 600, 10)
brake_decel = st.sidebar.slider("Braking deceleration (m/s², negative)", -20.0, 0.0, -8.0, 0.1)
accel = st.sidebar.slider("Sudden acceleration (m/s²)", 0.0, 10.0, 3.0, 0.1)

delay_s = delay_ms / 1000.0

def simulate_with_acpe(distance_to_obstacle, delay_s, brake_decel, accel, dt=0.01, max_time=10):
    time = [0]
    velocity = [0]
    position = [0]
    collision = False
    while time[-1] < max_time and not collision:
        t = time[-1] + dt
        if t < delay_s:
            a = accel
        else:
            a = brake_decel if velocity[-1] > 0 else 0
        v = max(0, velocity[-1] + a * dt)
        s = position[-1] + v * dt
        time.append(t)
        velocity.append(v)
        position.append(s)
        if s >= distance_to_obstacle:
            collision = True
            break
    return np.array(time), np.array(velocity), np.array(position), collision

def simulate_no_acpe(distance_to_obstacle, accel, dt=0.01, max_time=10):
    time = [0]
    velocity = [0]
    position = [0]
    collision = False
    while time[-1] < max_time and not collision:
        t = time[-1] + dt
        a = accel
        v = max(0, velocity[-1] + a * dt)
        s = position[-1] + v * dt
        time.append(t)
        velocity.append(v)
        position.append(s)
        if s >= distance_to_obstacle:
            collision = True
            break
    return np.array(time), np.array(velocity), np.array(position), collision

# Run both simulations
time, velocity, position, collision = simulate_with_acpe(distance_to_obstacle, delay_s, brake_decel, accel)
time_noacpe, velocity_noacpe, position_noacpe, collision_noacpe = simulate_no_acpe(distance_to_obstacle, accel)

# Convert all velocities to KPH for display and plotting
velocity_kph = velocity * 3.6
velocity_noacpe_kph = velocity_noacpe * 3.6

# Prepare plot
fig, ax1 = plt.subplots(figsize=(8, 4))
ax1.plot(time, velocity_kph, label="Velocity with ACPE (KPH)", color="b")
ax1.plot(time_noacpe, velocity_noacpe_kph, label="Velocity without ACPE (KPH)", color="orange", linestyle="--")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Velocity (KPH)", color="b")
ax1.tick_params(axis='y', labelcolor='b')
ax1.grid(True)

# Always show 8 KPH horizontal line
ax1.axhline(8, color='purple', linestyle=':', label='8 KPH Reference')

# Mark collision points and annotate collision speeds
if collision:
    collision_time = time[position >= distance_to_obstacle][0]
    collision_speed_kph = velocity_kph[position >= distance_to_obstacle][0]
    ax1.axvline(collision_time, color='r', linestyle='--', label='Collision (ACPE)')
    ax1.plot(collision_time, collision_speed_kph, 'ro')
    ax1.annotate(f"ACPE\n{collision_speed_kph:.2f} KPH",
                 xy=(collision_time, collision_speed_kph),
                 xytext=(collision_time, collision_speed_kph + 2),
                 arrowprops=dict(facecolor='red', shrink=0.05),
                 fontsize=10, color='red', bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
else:
    collision_speed_kph = 0
    st.success("No collision within simulation time (ACPE enabled).")

if collision_noacpe:
    collision_time_noacpe = time_noacpe[position_noacpe >= distance_to_obstacle][0]
    collision_speed_noacpe_kph = velocity_noacpe_kph[position_noacpe >= distance_to_obstacle][0]
    ax1.axvline(collision_time_noacpe, color='brown', linestyle='--', label='Collision (No ACPE)')
    ax1.plot(collision_time_noacpe, collision_speed_noacpe_kph, 'o', color='brown')
    ax1.annotate(f"No ACPE\n{collision_speed_noacpe_kph:.2f} KPH",
                 xy=(collision_time_noacpe, collision_speed_noacpe_kph),
                 xytext=(collision_time_noacpe, collision_speed_noacpe_kph + 2),
                 arrowprops=dict(facecolor='brown', shrink=0.05),
                 fontsize=10, color='brown', bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
else:
    collision_speed_noacpe_kph = 0
    st.success("No collision within simulation time (No ACPE).")

# Show Chassis Braking Start (should align with velocity turning point) only if ACPE avoids collision
if not collision:
    turning_idx = np.argmax(time >= delay_s)
    turning_time = time[turning_idx]
    turning_velocity_kph = velocity_kph[turning_idx]
    ax1.axvline(turning_time, color='g', linestyle=':', label='Chassis Physical Braking Start')
    ax1.annotate("Chassis Braking Start",
                 xy=(turning_time, turning_velocity_kph),
                 xytext=(turning_time, turning_velocity_kph + 2),
                 arrowprops=dict(facecolor='green', shrink=0.05),
                 fontsize=10, color='green', bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))

ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5))
st.pyplot(fig)

# Show collision speed comparison and reduction percentage
if collision and collision_noacpe:
    reduction = (collision_speed_noacpe_kph - collision_speed_kph) / collision_speed_noacpe_kph * 100 if collision_speed_noacpe_kph > 0 else 0
    st.markdown(f"**Collision speed with ACPE:** {collision_speed_kph:.2f} KPH")
    st.markdown(f"**Collision speed without ACPE:** {collision_speed_noacpe_kph:.2f} KPH")
    st.markdown(f"**Reduction:** {reduction:.1f}%")
elif collision_noacpe:
    st.markdown(f"**Collision speed without ACPE:** {collision_speed_noacpe_kph:.2f} KPH")
    st.markdown(f"**No collision with ACPE!**")
elif collision:
    st.markdown(f"**Collision speed with ACPE:** {collision_speed_kph:.2f} KPH")
    st.markdown(f"**No collision without ACPE!**")
else:
    st.markdown(f"**No collision in either scenario!**")

# Show parameter summary
st.markdown(f"**Parameters:**  ")
st.markdown(f"- Distance to obstacle: {distance_to_obstacle} m")
st.markdown(f"- ACPE brake delay: {delay_ms} ms")
st.markdown(f"- Braking deceleration: {brake_decel} m/s²")
st.markdown(f"- Sudden acceleration: {accel} m/s²") 