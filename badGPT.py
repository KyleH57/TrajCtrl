import numpy as np
import matplotlib.pyplot as plt


class WheelTrajectories:
    def __init__(self, L, V_max, max_accel):
        self.L = L
        self.V_max = V_max
        self.max_accel = max_accel
        self.start_pos = None
        self.start_angle = None
        self.end_pos = None
        self.end_angle = None
        self.D = None
        self.V = None
        self.max_time = None
        self.trajectory_data = None

    def set_start(self, x, y, angle_degrees):
        self.start_pos = (x, y)
        self.start_angle = np.radians(angle_degrees)  # Convert degrees to radians

    def set_end(self, x, y, angle_degrees):
        self.end_pos = (x, y)
        self.end_angle = np.radians(angle_degrees)  # Convert degrees to radians
        self._calculate_displacement()
        self.max_time = self._calculate_max_time()
        self._calculate_velocity()
        self.trajectory_data = self._generate_trajectories()

    def _calculate_displacement(self):
        P_start = np.array(self.start_pos)
        P_end = np.array(self.end_pos)
        theta = self.end_angle - self.start_angle

        # Calculate linear displacement components
        D_linear = P_end - P_start
        D_FL_linear = D_linear[1] + D_linear[0]
        D_FR_linear = D_linear[1] - D_linear[0]
        D_RL_linear = D_linear[1] - D_linear[0]
        D_RR_linear = D_linear[1] + D_linear[0]

        # Calculate angular displacement components
        D_angular = theta * self.L

        # Combine linear and angular displacements for each wheel
        D_FL = D_FL_linear + D_angular
        D_FR = D_FR_linear - D_angular
        D_RL = D_RL_linear + D_angular
        D_RR = D_RR_linear - D_angular

        self.D = [D_FL, D_FR, D_RL, D_RR]

    def _calculate_velocity(self):
        theta = self.end_angle - self.start_angle
        omega = theta / self.max_time  # Angular velocity

        # Calculate velocities
        V_FL = self.D[0] / self.max_time + omega * self.L
        V_FR = self.D[1] / self.max_time - omega * self.L
        V_RL = self.D[2] / self.max_time + omega * self.L
        V_RR = self.D[3] / self.max_time - omega * self.L

        # Normalize velocities
        V_abs_max = max(abs(V_FL), abs(V_FR), abs(V_RL), abs(V_RR))
        scaling_factor = self.V_max / V_abs_max
        self.V = [V_FL * scaling_factor, V_FR * scaling_factor, V_RL * scaling_factor, V_RR * scaling_factor]

    def _calculate_max_time(self):
        times = [self._calculate_total_time(self.D[i]) for i in range(4)]
        return max(times)

    def _calculate_total_time(self, target_distance):
        time_to_max_vel = abs(target_distance / self.V_max) / self.max_accel
        accel_distance = 0.5 * self.max_accel * time_to_max_vel ** 2

        if abs(target_distance) <= 2 * accel_distance:
            time_at_max_vel = 0
        else:
            time_at_max_vel = (abs(target_distance) - 2 * accel_distance) / self.V_max

        total_time = 2 * time_to_max_vel + time_at_max_vel
        return total_time

    def _generate_trajectories(self):
        trajectory_data = {}
        for i, wheel in enumerate(['FL', 'FR', 'RL', 'RR']):
            time_to_max_vel = abs(self.V[i]) / self.max_accel
            accel_distance = 0.5 * self.max_accel * time_to_max_vel ** 2
            time_at_max_vel = self.max_time - 2 * time_to_max_vel
            t = np.linspace(0, self.max_time, 1000)

            # Velocity calculation
            vel = np.where(t <= time_to_max_vel,
                           self.max_accel * t * np.sign(self.D[i]),
                           np.where(t <= time_to_max_vel + time_at_max_vel,
                                    self.V[i],
                                    self.V[i] - self.max_accel * (t - time_to_max_vel - time_at_max_vel) * np.sign(self.D[i])))

            # Position calculation
            pos = np.where(
                t <= time_to_max_vel,
                0.5 * self.max_accel * t ** 2 * np.sign(self.D[i]),  # Acceleration phase
                np.where(
                    t <= time_to_max_vel + time_at_max_vel,
                    accel_distance + self.V[i] * (t - time_to_max_vel) * np.sign(self.D[i]),  # Constant velocity phase
                    self.D[i] - 0.5 * self.max_accel * (self.max_time - t) ** 2 * np.sign(self.D[i])  # Deceleration phase
                )
            )

            trajectory_data[wheel] = {'velocity': vel, 'position': pos}
        return trajectory_data

    def get_wheel_vel(self, wheel, time):
        if wheel not in self.trajectory_data:
            return None
        vel = np.interp(time, np.linspace(0, self.max_time, 1000), self.trajectory_data[wheel]['velocity'])
        return vel

    def plot_trajectories(self):
        t = np.linspace(0, self.max_time, 1000)

        plt.figure(figsize=(12, 6))

        plt.subplot(1, 2, 1)
        plt.title('Velocity Trajectories')
        for wheel in self.trajectory_data:
            plt.plot(t, self.trajectory_data[wheel]['velocity'], label=wheel)
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity')
        plt.legend()

        plt.subplot(1, 2, 2)
        plt.title('Position Trajectories')
        for wheel in self.trajectory_data:
            plt.plot(t, self.trajectory_data[wheel]['position'], label=wheel)
        plt.xlabel('Time (s)')
        plt.ylabel('Position')
        plt.legend()

        plt.tight_layout()
        plt.show()


# Example usage
wheel_traj = WheelTrajectories(L=1.0, V_max=1.0, max_accel=1.0)

# Set start and end positions and angles
wheel_traj.set_start(x=0, y=0, angle_degrees=0)
wheel_traj.set_end(x=0, y=8, angle_degrees=90)

# Plot trajectories
wheel_traj.plot_trajectories()

# Get the velocity of a specific wheel at a specific time
