import numpy as np
import matplotlib.pyplot as plt


def calculate_displacement(P_start, P_end, theta, L):
    D_linear = np.array(P_end) - np.array(P_start)

    D_FL_linear = D_linear[1] + D_linear[0]
    D_FR_linear = D_linear[1] - D_linear[0]
    D_RL_linear = D_linear[1] - D_linear[0]
    D_RR_linear = D_linear[1] + D_linear[0]

    D_angular = theta * L

    D_FL = D_FL_linear + D_angular
    D_FR = D_FR_linear - D_angular
    D_RL = D_RL_linear + D_angular
    D_RR = D_RR_linear - D_angular

    return D_FL, D_FR, D_RL, D_RR


def calculate_wheel_velocities(D, omega, L, V_max):
    V_FL = D[1] + D[0] + omega * L
    V_FR = D[1] - D[0] - omega * L
    V_RL = D[1] - D[0] + omega * L
    V_RR = D[1] + D[0] - omega * L

    V_abs_max = max(abs(V_FL), abs(V_FR), abs(V_RL), abs(V_RR))
    scaling_factor = V_max / V_abs_max

    V_FL *= scaling_factor
    V_FR *= scaling_factor
    V_RL *= scaling_factor
    V_RR *= scaling_factor

    return V_FL, V_FR, V_RL, V_RR


def estimate_time(D, V):
    t_FL = D[0] / V[0]
    t_FR = D[1] / V[1]
    t_RL = D[2] / V[2]
    t_RR = D[3] / V[3]

    return max(t_FL, t_FR, t_RL, t_RR)


# Define parameters
P_start = [0, 0]
P_end = [5, 0]
theta = 0
L = 1.0  # Assuming distance from robot center to a wheel is 1 unit
V_max = 1.0  # Maximum allowed velocity

# Calculate Displacements
D = calculate_displacement(P_start, P_end, theta, L)

# Calculate and Normalize Wheel Velocities
V = calculate_wheel_velocities([P_end[0] - P_start[0], P_end[1] - P_start[1]], theta, L, V_max)

# Estimate Time to reach destination
t_max = estimate_time(D, V)

# Plot Results
fig, ax = plt.subplots(3, 1, figsize=(10, 15))

# Displacement Plot
ax[0].bar(['FL', 'FR', 'RL', 'RR'], D)
ax[0].set_title('Wheel Displacements')
ax[0].set_ylabel('Distance')

# Velocity Plot
ax[1].bar(['FL', 'FR', 'RL', 'RR'], V)
ax[1].set_title('Wheel Velocities')
ax[1].set_ylabel('Velocity')

# Time Plot
ax[2].bar(['FL', 'FR', 'RL', 'RR'], [t_max] * 4, color='gray')
ax[2].set_title('Estimated Time to Reach Destination')
ax[2].set_ylabel('Time')

plt.tight_layout()
plt.show()
