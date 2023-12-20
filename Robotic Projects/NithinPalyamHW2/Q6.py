import numpy as np


def FK(DH):  # Compute the forward kinematic matrix based on Denavit-Hartenberg (DH) parameters
    """
    Parameters:
    - DH (list of tuples): List of DH parameter tuples. Each tuple contains the following DH parameters:
      - a (float): Link length (distance along the common normal, measured in meters).
      - alpha (float): Link twist (angle about the previous z-axis, measured in radians).
      - d (float): Link offset (distance along the previous x-axis, measured in meters).
      - theta (float): Joint angle (angle about the current z-axis, measured in radians).
    """
    A = np.eye(4)  # Initialize the transformation matrix as an identity matrix

    for dh_params in DH:
        a, alpha, d, theta = dh_params

        # Create the transformation matrix for this DH link
        link_transform = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                                   [np.sin(theta), np.cos(
                                       theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                                   [0, np.sin(alpha), np.cos(alpha), d],
                                   [0, 0, 0, 1]])

        # Multiply the current transformation matrix by the link's transformation matrix
        A = np.dot(A, link_transform)

    # Returns:- A (numpy.ndarray): The 4x4 forward kinematic matrix representing the transformation from the base frame
    return A
    # to the end-effector frame based on the DH parameters


# DH parameters are represented as tuples: (a, alpha, d, theta)
# Define the robot's DH parameters as a list of tuples:
DH_parameters = [
    (0.1, np.pi/2, 0.2, np.pi/4),
    (0.2, 0, 0, np.pi/3),
    (0.15, -np.pi/2, 0, np.pi/6),
    (0.05, 0, 0.1, -np.pi/8)
]

# Compute the forward kinematic matrix
forward_kinematics_matrix = FK(DH_parameters)

print("Forward Kinematic Matrix:")
print(forward_kinematics_matrix)
