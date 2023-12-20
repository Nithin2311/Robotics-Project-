import numpy as np


def Euler(theta1, theta2, theta3):  # where theta1 = z, theta2 = y, theta3 = Z

    R1 = np.array([[1, 0, 0],
                   [0, np.cos(theta1), -np.sin(theta1)],
                   [0, np.sin(theta1), np.cos(theta1)]])

    R2 = np.array([[np.cos(theta2), 0, np.sin(theta2)],
                   [0, 1, 0],
                   [-np.sin(theta2), 0, np.cos(theta2)]])

    R3 = np.array([[np.cos(theta3), -np.sin(theta3), 0],
                   [np.sin(theta3), np.cos(theta3), 0],
                   [0, 0, 1]])

    # Combine the individual rotations
    R = np.dot(R3, np.dot(R2, R1))

    return R


def RPY(Rtheta1, Rtheta2, Rtheta3):  # where Rtheta1 = roll, Rtheta2 = pitch, Rtheta3 = Yaw

    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(Rtheta1), -np.sin(Rtheta1)],
                       [0, np.sin(Rtheta1), np.cos(Rtheta1)]])

    R_pitch = np.array([[np.cos(Rtheta2), 0, np.sin(Rtheta2)],
                        [0, 1, 0],
                        [-np.sin(Rtheta2), 0, np.cos(Rtheta2)]])

    R_yaw = np.array([[np.cos(Rtheta3), -np.sin(Rtheta3), 0],
                      [np.sin(Rtheta3), np.cos(Rtheta3), 0],
                      [0, 0, 1]])

    # Combine the individual rotations
    R = np.dot(R_yaw, np.dot(R_pitch, R_roll))

    return R


theta1 = float(input("Enter the first Euler angle (in radians): "))
theta2 = float(input("Enter the second Euler angle (in radians): "))
theta3 = float(input("Enter the third Euler angle (in radians): "))
R_euler = Euler(theta1, theta2, theta3)

Rtheta1 = float(input("Enter the roll angle (in radians): "))
Rtheta2 = float(input("Enter the pitch angle (in radians): "))
Rtheta3 = float(input("Enter the yaw angle (in radians): "))
R_rpy = RPY(Rtheta1, Rtheta2, Rtheta3)

print("Rotation matrix using Euler angles:")
print(R_euler)
print("\nRotation matrix using Roll-Pitch-Yaw angles:")
print(R_rpy)
