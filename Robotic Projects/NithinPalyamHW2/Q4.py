import numpy as np


# Parameters: theta (float): Angle of rotation in radians

def RotX(theta):  # Compute the rotation matrix around the X-axis
    R = np.array([[1, 0, 0],
                  [0, np.cos(theta), -np.sin(theta)],
                  [0, np.sin(theta), np.cos(theta)]])
    # Returns:R : Rotation matrix about the X-axis
    return R


def RotY(theta):  # Compute the rotation matrix around the Y-axis
    R = np.array([[np.cos(theta), 0, np.sin(theta)],
                  [0, 1, 0],
                  [-np.sin(theta), 0, np.cos(theta)]])
    # Returns:R : Rotation matrix  about the Y-axis
    return R


def RotZ(theta):  # Compute the rotation matrix around the Z-axis

    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta), np.cos(theta), 0],
                  [0, 0, 1]])
    # Returns:R  Rotation matrix  about the Z-axis
    return R


# Example usage:
theta_x = float(
    input("Enter the rotation angle around the X-axis (in radians): "))
R_x = RotX(theta_x)

theta_y = float(
    input("Enter the rotation angle around the Y-axis (in radians): "))
R_y = RotY(theta_y)

theta_z = float(
    input("Enter the rotation angle around the Z-axis (in radians): "))
R_z = RotZ(theta_z)

print("Rotation matrix around X-axis:")
print(R_x)
print("\nRotation matrix around Y-axis:")
print(R_y)
print("\nRotation matrix around Z-axis:")
print(R_z)
