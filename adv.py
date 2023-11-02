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


def calculate_total_time(target_distance, max_vel, max_accel):
    time_to_max_vel, _, time_at_max_vel = calculate_motion_segments(target_distance, max_vel, max_accel)
    total_time = 2 * time_to_max_vel + time_at_max_vel
    return total_time


def calculate_motion_segments(target_distance, max_vel, max_accel):
    """
    Calculates the time to accelerate to max velocity, the distance traveled during acceleration, and the time spent
    Supports both positive and negative distances.
    :param target_distance:
    :param max_vel:
    :param max_accel:
    :return: time_to_max_vel, accel_distance, time_at_max_vel
    """
    time_to_max_vel = abs(max_vel) / abs(max_accel)
    accel_distance = 0.5 * abs(max_accel) * time_to_max_vel ** 2

    if abs(target_distance) <= 2 * accel_distance:
        time_at_max_vel = 0
    else:
        time_at_max_vel = (abs(target_distance) - 2 * accel_distance) / abs(max_vel)

    return time_to_max_vel, accel_distance, time_at_max_vel


def plot_combined(D, V, max_time, max_accel):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    ax1.set_title('Wheel Trajectories')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position & Velocity')
    ax1.axhline(0, color='black', linewidth=0.8)  # x-axis


    for i, wheel in enumerate(['FL', 'FR', 'RL', 'RR']):
        adjusted_velocity = V[i]  # Retain the original sign of the velocity
        time_to_max_vel, accel_distance, time_at_max_vel = calculate_motion_segments(D[i], adjusted_velocity, max_accel)
        t = np.linspace(0, max_time, 1000)

        # Velocity calculation (Use the original sign of adjusted_velocity)
        vel = np.where(t <= time_to_max_vel,
                       max_accel * t * np.sign(D[i]),
                       np.where(t <= time_to_max_vel + time_at_max_vel,
                                adjusted_velocity,
                                adjusted_velocity - max_accel * (t - time_to_max_vel - time_at_max_vel) * np.sign(D[i])))

        # Position calculation (similarly, use the original sign of adjusted_velocity)
        pos = np.where(
            t <= time_to_max_vel,
            0.5 * max_accel * t ** 2 * np.sign(D[i]),  # Acceleration phase
            np.where(
                t <= time_to_max_vel + time_at_max_vel,
                accel_distance * np.sign(D[i]) + adjusted_velocity * (t - time_to_max_vel),
                accel_distance * np.sign(D[i]) + adjusted_velocity * time_at_max_vel + adjusted_velocity * (t - time_to_max_vel - time_at_max_vel) - 0.5 * max_accel * (t - time_to_max_vel - time_at_max_vel) ** 2 * np.sign(D[i])
                # Deceleration phase
            )
        )

        ax1.plot(t, pos, label=f'{wheel} Position')
        ax1.plot(t, vel, '--', label=f'{wheel} Velocity')

    ax1.legend()

    # Bar graph for displacements
    ax2.set_title('Wheel Displacements')
    ax2.bar(['FL', 'FR', 'RL', 'RR'], D, color='grey', alpha=0.7)
    ax2.set_ylabel('Displacement')

    plt.tight_layout()
    plt.show()

def calculate_trajectories(D, V, max_time, max_accel):
    trajectories = {}
    for i, wheel in enumerate(['FL', 'FR', 'RL', 'RR']):
        adjusted_velocity = V[i]
        time_to_max_vel, accel_distance, time_at_max_vel = calculate_motion_segments(D[i], adjusted_velocity, max_accel)
        t = np.linspace(0, max_time, 1000)

        # Velocity calculation
        vel = np.where(t <= time_to_max_vel,
                       max_accel * t * np.sign(D[i]),
                       np.where(t <= time_to_max_vel + time_at_max_vel,
                                adjusted_velocity,
                                adjusted_velocity - max_accel * (t - time_to_max_vel - time_at_max_vel) * np.sign(D[i])))

        # Position calculation
        pos = np.where(
            t <= time_to_max_vel,
            0.5 * max_accel * t ** 2 * np.sign(D[i]),
            np.where(
                t <= time_to_max_vel + time_at_max_vel,
                accel_distance * np.sign(D[i]) + adjusted_velocity * (t - time_to_max_vel),
                accel_distance * np.sign(D[i]) + adjusted_velocity * time_at_max_vel + adjusted_velocity * (t - time_to_max_vel - time_at_max_vel) - 0.5 * max_accel * (t - time_to_max_vel - time_at_max_vel) ** 2 * np.sign(D[i])
            )
        )

        trajectories[wheel] = {'time': t, 'position': pos, 'velocity': vel}

    return trajectories


def plot_trajectories(D, trajectories):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    ax1.set_title('Wheel Trajectories')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position & Velocity')
    ax1.axhline(0, color='black', linewidth=0.8)

    for wheel in ['FL', 'FR', 'RL', 'RR']:
        t = trajectories[wheel]['time']
        pos = trajectories[wheel]['position']
        vel = trajectories[wheel]['velocity']
        ax1.plot(t, pos, label=f'{wheel} Position')
        ax1.plot(t, vel, '--', label=f'{wheel} Velocity')

    ax1.legend()

    ax2.set_title('Wheel Displacements')
    ax2.bar(['FL', 'FR', 'RL', 'RR'], D, color='grey', alpha=0.7)
    ax2.set_ylabel('Displacement')

    plt.tight_layout()
    plt.show()

def calculate_max_time(D, V, max_accel):
    max_times = []

    for i in range(len(D)):
        time_to_max_vel, accel_distance, time_at_max_vel = calculate_motion_segments(D[i], V[i], max_accel)

        # Time to decelerate to a stop from max velocity
        time_to_stop = abs(V[i]) / max_accel

        # Total time for the wheel
        total_time = time_to_max_vel + time_at_max_vel + time_to_stop

        max_times.append(total_time)

    # Maximum time across all wheels
    return max(max_times)


print(calculate_motion_segments(-8, 1, 1))
print(calculate_total_time(-8, 1, 1))

# Define parameters
P_start = [0, 0]
P_end = [0, 8]
theta_degrees = 0
theta = theta_degrees * np.pi / 180
L = 1.0
V_max = 1.0
max_accel = 1.0

# Calculate Displacements
D = calculate_displacement(P_start, P_end, theta, L)

# Calculate and Normalize Wheel Velocities
V = calculate_wheel_velocities([P_end[0] - P_start[0], P_end[1] - P_start[1]], theta, L, V_max)


max_time = calculate_max_time(D, V, max_accel)
trajectories = calculate_trajectories(D, V, max_time, max_accel)
plot_trajectories(D, trajectories)



