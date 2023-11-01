import math


def field_to_robot_coords(x_current, y_current, theta, x_target, y_target):
    """
    Converts field coordinates to robot-centric coordinates.

    Parameters:
        x_current (float): Current X position of the robot in field coordinates.
        y_current (float): Current Y position of the robot in field coordinates.
        theta (float): Current orientation of the robot in radians.
        x_target (float): Target X position in field coordinates.
        y_target (float): Target Y position in field coordinates.

    Returns:
        (float, float): A tuple containing the target position in robot-centric coordinates.
    """
    # Calculate the relative position
    delta_x = x_target - x_current
    delta_y = y_target - y_current

    # Apply rotation transformation
    x_robot = delta_x * math.cos(theta) + delta_y * math.sin(theta)
    y_robot = -delta_x * math.sin(theta) + delta_y * math.cos(theta)

    return x_robot, y_robot


# Example usage:
x_current = 5.0  # Current X position of the robot
y_current = 5.0  # Current Y position of the robot
theta = math.pi / 2 # Current orientation of the robot in radians
x_target = 5.0  # Target X position in field coordinates
y_target = 10.0  # Target Y position in field coordinates

# Convert field coordinates to robot-centric coordinates
x_robot, y_robot = field_to_robot_coords(x_current, y_current, theta, x_target, y_target)
print(f"Robot-centric coordinates: X = {x_robot}, Y = {y_robot}")
