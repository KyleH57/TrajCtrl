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


print(calculate_motion_segments(-8, 1, 1))
print(calculate_total_time(-8, 1, 1))

# Define parameters
P_start = [0, 0]
P_end = [8, 5]
theta_degrees = 69
theta = theta_degrees * np.pi / 180
L = 1.0
V_max = 1.0
max_accel = 1.0

# Calculate Displacements
D = calculate_displacement(P_start, P_end, theta, L)

# Calculate and Normalize Wheel Velocities
V = calculate_wheel_velocities([P_end[0] - P_start[0], P_end[1] - P_start[1]], theta, L, V_max)

times = [calculate_total_time(d, V[i], max_accel) for i, d in enumerate(D)]
print("Individual Wheel Times:", times)
max_time = max(times)

# Find the maximum time required for any wheel to complete its movement
max_time = max([calculate_total_time(d, V[i], max_accel) for i, d in enumerate(D)])

# Plot combined trajectories of all wheels and the bar graph of displacements
plot_combined(D, V, max_time, max_accel)

# After calculating D
print("Displacements:", D)

# After calculating V
print("Wheel Velocities:", V)

# After calculating max_time
print("Maximum Time Required:", max_time)



#
# # Revised adjusted velocity and acceleration distance calculation
# for i, wheel in enumerate(['FL', 'FR', 'BL', 'BR']):
#     adjusted_velocity = V[i]  # Using the actual maximum velocity for each wheel
#     time_to_max_vel, accel_distance, time_at_max_vel = calculate_motion_segments(D[i], adjusted_velocity,
#                                                                                  max_accel)
#
#     # Debugging for each wheel
#     # Generating time array for plotting
#     t = np.linspace(0, max_time, 1000)
#
#     # Velocity calculation
#     vel = np.where(t <= time_to_max_vel,
#                    max_accel * t * np.sign(D[i]),
#                    np.where(t <= time_to_max_vel + time_at_max_vel,
#                             adjusted_velocity * np.sign(D[i]),
#                             adjusted_velocity * np.sign(D[i]) - max_accel * (
#                                         t - time_to_max_vel - time_at_max_vel) * np.sign(D[i])))
#
#     # Position calculation
#     pos = np.where(
#         t <= time_to_max_vel,
#         0.5 * max_accel * t ** 2 * np.sign(D[i]),  # Acceleration phase
#         np.where(
#             t <= time_to_max_vel + time_at_max_vel,
#             accel_distance * np.sign(D[i]) + adjusted_velocity * (t - time_to_max_vel) * np.sign(D[i]),
#             # Constant velocity phase
#             accel_distance * np.sign(D[i]) + adjusted_velocity * time_at_max_vel * np.sign(D[i]) + adjusted_velocity * (
#                         t - time_to_max_vel - time_at_max_vel) * np.sign(D[i]) - 0.5 * max_accel * (
#                         t - time_to_max_vel - time_at_max_vel) ** 2 * np.sign(D[i])  # Deceleration phase
#         )
#     )
#
#     # Rerun with revised position calculation
#     t_accel_phase_end = np.where(t > time_to_max_vel)[0][0]
#     pos_at_accel_phase_end = pos[t_accel_phase_end]
#
#     print(f"Wheel: {wheel}")
#     print(f"Calculated Accel Distance: {accel_distance}")
#     print(f"Position at End of Accel Phase on Plot: {pos_at_accel_phase_end}")
#
#     # Plotting for the wheel
#     plt.figure(figsize=(8, 4))
#     plt.plot(t, vel, label='Velocity')
#     plt.plot(t, pos, label='Position')
#     plt.axvline(time_to_max_vel, color='red', linestyle='--', label='End of Accel Phase')  # End of acceleration phase
#     plt.title(f'{wheel} Wheel Trajectory')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Position & Velocity')
#     plt.axhline(0, color='black', linewidth=0.8)  # x-axis
#
#     plt.grid(True)  # Adding grid
#
#     plt.legend()
#     plt.show()
