import numpy as np
import matplotlib.pyplot as plt


def calculate_motion_segments(target_distance, max_vel, max_accel):
    """
    Calculates the time to accelerate to max velocity, the distance traveled during acceleration, and the time spent
    :param target_distance:
    :param max_vel:
    :param max_accel:
    :return: time_to_max_vel, accel_distance, time_at_max_vel
    """
    time_to_max_vel = max_vel / max_accel
    accel_distance = 0.5 * max_accel * time_to_max_vel ** 2

    if target_distance <= 2 * accel_distance:
        time_at_max_vel = 0
    else:
        time_at_max_vel = (target_distance - 2 * accel_distance) / max_vel

    return time_to_max_vel, accel_distance, time_at_max_vel


def mecanum_velocities(current_pos, current_angle, target_pos, end_angle, max_vel, debug=False):
    # Constants
    l = 1.0  # Distance from center to wheel, adjust as necessary

    # Positional error
    error_x = target_pos[0] - current_pos[0]
    error_y = target_pos[1] - current_pos[1]

    # Angular error
    error_angle = end_angle - current_angle

    # Calculate the desired robot velocities
    Vx = error_x
    Vy = error_y
    omega = error_angle

    # Clamping velocities
    Vx = max(-max_vel, min(max_vel, Vx))
    Vy = max(-max_vel, min(max_vel, Vy))
    omega = max(-max_vel, min(max_vel, omega))

    # Mecanum kinematics
    V_FR = Vy - Vx - omega * l
    V_FL = Vy + Vx + omega * l
    V_RR = Vy + Vx - omega * l
    V_RL = Vy - Vx + omega * l

    if debug:
        time = np.linspace(0, 1, 100)  # Sample time points
        plt.plot(time, V_FR*np.ones_like(time), label="V_FR")
        plt.plot(time, V_FL*np.ones_like(time), label="V_FL")
        plt.plot(time, V_RR*np.ones_like(time), label="V_RR")
        plt.plot(time, V_RL*np.ones_like(time), label="V_RL")
        plt.legend()
        plt.xlabel("Time")
        plt.ylabel("Velocity")
        plt.title("Motor Trajectories")
        plt.ylim(-max_vel * 1.2, max_vel * 1.2)
        plt.grid(True)
        plt.show()

    return V_FR, V_FL, V_RR, V_RL


def trapezoid_value(start, rise_duration, horizontal_duration, horizontal_y, t):
    """
    Calculates the value of a trapezoid function at a given time
    :param start: x coordinate of the start of the trapezoid. Y coordinate is assumed to be 0
    :param rise_duration: Acceleration time. Deceleration time is assumed to be the same
    :param horizontal_duration: The time spent at max velocity
    :param horizontal_y: The max velocity
    :param t: Time to evaluate the function at
    :return: Velocity at time t
    """
    # Check if t is outside the trapezoid
    if t <= start or t >= start + 2*rise_duration + horizontal_duration:
        return 0

    # Calculate value based on which segment t falls into
    if t <= start + rise_duration:
        # On the rising edge
        slope = horizontal_y / rise_duration
        return slope * (t - start)
    elif t <= start + rise_duration + horizontal_duration:
        # On the horizontal segment
        return horizontal_y
    else:
        # On the falling edge
        slope = -horizontal_y / rise_duration
        return horizontal_y + slope * (t - start - rise_duration - horizontal_duration)


def plot_trapezoid(start, rise_duration, horizontal_duration, horizontal_y):
    # Generate a range of t values
    t_values = np.linspace(0, 10, 400)

    # Evaluate the trapezoid function for each t value
    y_values = [trapezoid_value(start, rise_duration, horizontal_duration, horizontal_y, t) for t in t_values]

    # Plot the results
    plt.plot(t_values, y_values, label="Trapezoid Function")
    plt.axvline(x=start, color='r', linestyle='--', label="Start")
    plt.axvline(x=start + rise_duration, color='b', linestyle='--', label="Start of Horizontal Segment")
    plt.axvline(x=start + rise_duration + horizontal_duration, color='g', linestyle='--', label="End of Horizontal Segment")
    plt.title("Trapezoid Function Plot")
    plt.xlabel("Time (t)")
    plt.ylabel("Value")
    plt.legend()
    plt.grid(True)
    plt.show()


# Test
current_position = (0, 0)
current_velocity = (0, 0, 0)
current_angle = 0
target_position = (14, 10)
end_angle = 0
max_velocity = 2.0
max_acceleration = 1.0
#
# V_FR, V_FL, V_RR, V_RL = mecanum_velocities(current_position, current_angle, target_position,
#                                             end_angle, max_velocity, debug=True)
# print(V_FR, V_FL, V_RR, V_RL)


motion_segments = calculate_motion_segments(10, max_velocity, 1)
print(motion_segments)  # (time_to_max_vel, accel_distance, time_at_max_vel)
trapezoid_value(0, motion_segments[0], motion_segments[2], max_velocity, 0)
plot_trapezoid(0, motion_segments[0], motion_segments[2], max_velocity)
