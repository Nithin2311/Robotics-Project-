from zmqRemoteApi import RemoteAPIClient
import numpy as np
import sys
import math
import time
import sim.py
sys.path.append('PythonAPI')
sys.path.append('zmqRemoteApi')


print('Program started')
# connect to the coppelia scene
clientID = RemoteAPIClient()
# clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Failed connecting to remote API server')
    sys.exit('Could not connect to Coppelia')

sim = clientID.getObject('sim')

# get the handles of arm joints
arm_handle = sim.getObject('/UR5')
armjoint1_handle = sim.getObject('/UR5/joint')
armjoint2_handle = sim.getObject('/UR5/joint/joint')
armjoint3_handle = sim.getObject('/UR5/joint/joint/joint')
armjoint4_handle = sim.getObject('/UR5/joint/joint/joint/joint')
armjoint5_handle = sim.getObject('/UR5/joint/joint/joint/joint/joint')
armjoint6_handle = sim.getObject('/UR5/joint/joint/joint/joint/joint/joint')
# get the handles of end effector
endeffector_handle = sim.getObject(
    '/UR5/joint/joint/joint/joint/joint/joint/suctionPad')

# set the arm to position control
sim.simxSetObjectIntParameter(
    clientID, armjoint1_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint1_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint2_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint2_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint3_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint3_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint4_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint4_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint5_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint5_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint6_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(
    clientID, armjoint6_handle, 2001, 1, sim.simx_opmode_oneshot)


def move_arm(armpose):
    armpose_convert = []
    for i in range(6):
        armpose_convert.append(round(armpose[i]/180 * math.pi, 3))
    sim.simxPauseCommunication(clientID, True)
    sim.simxSetJointTargetPosition(
        clientID, armjoint1_handle, armpose_convert[0], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(
        clientID, armjoint2_handle, armpose_convert[1], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(
        clientID, armjoint3_handle, armpose_convert[2], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(
        clientID, armjoint4_handle, armpose_convert[3], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(
        clientID, armjoint5_handle, armpose_convert[4], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(
        clientID, armjoint6_handle, armpose_convert[5], sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(clientID, False)
    time.sleep(3)

# convert to cosine angle


def convert_cosine(angles):
    cosine = math.cos(angles)
    return cosine

# Convert to sine angle


def convert_sine(angles):
    sine = math.sin(angles)
    return sine


def FK(DH):
    Array = np.array(DH)
    ai = Array[:, 0]  # storing all ai values in one array
    alpha_i = Array[:, 1]  # storing all alpha values in one array
    di = Array[:, 2]  # storing all di values in one array
    Theta_i = Array[:, 3]  # storing all Theta values in one array

    Cos_Theta = convert_cosine(math.radians(Theta_i[0]))
    Sin_Theta = convert_sine(math.radians(Theta_i[0]))
    Cos_Alpha = convert_cosine(math.radians(alpha_i[0]))
    Sin_Alpha = convert_sine(math.radians(alpha_i[0]))

    # saving the matrix calculations for first link
    Final = [[Cos_Theta, -Cos_Alpha*Sin_Theta, Sin_Alpha*Sin_Theta, ai[0]*Cos_Theta],
             [Sin_Theta, Cos_Alpha*Cos_Theta, -
                 Sin_Alpha*Cos_Theta, ai[0]*Sin_Theta],
             [0, Sin_Alpha, Cos_Alpha, di[0]],
             [0, 0, 0, 1]]

    # computing the calculations for each link and returning final matrix
    for i in range(6):
        Cos_Theta = convert_cosine(math.radians(Theta_i[i+1]))
        Sin_Theta = convert_sine(math.radians(Theta_i[i+1]))
        Cos_Alpha = convert_cosine(math.radians(alpha_i[i+1]))
        Sin_Alpha = convert_sine(math.radians(alpha_i[i+1]))

        Matrix = [[Cos_Theta, -Cos_Alpha*Sin_Theta, Sin_Alpha*Sin_Theta, ai[i+1]*Cos_Theta],
                  [Sin_Theta, Cos_Alpha*Cos_Theta, -
                      Sin_Alpha*Cos_Theta, ai[i+1]*Sin_Theta],
                  [0, Sin_Alpha, Cos_Alpha, di[i+1]],
                  [0, 0, 0, 1]]
        Final_matrix = np.matmul(Final, Matrix)
    return Final_matrix


joint_angles = np.zeros((10, 6))
for i in range(10):
    joint_angles[i, :] = np.random.uniform(-60, 60, size=(6))
joint_angles = joint_angles.astype(int)

# DH parameters are represented as tuples: (a, alpha, d, theta)
# Define the robot's DH parameters as a list of tuples:
for i in range(10):
    DH = [(90,     0,    89.2,    joint_angles[i, 0]),
          (0,    425,       0,    joint_angles[i, 1]),
          (0,    392,       0,    joint_angles[i, 2]),
          (-90,    0,   109.3,    joint_angles[i, 3]),
          (90,     0,   94.75,    joint_angles[i, 4]),
          (0,      0,    82.5,    joint_angles[i, 4]),
          (180,    0,    27.5,    joint_angles[i, 5])
          ]
DH_array = np.array(DH)

# Compute the forward kinematic matrix
forward_kinematics_matrix = FK(DH_array)


for i in range(10):
    print("\n arm is moving to the angle set: ", i+1)
    move_arm([joint_angles[i, 0],
             joint_angles[i, 1],
             joint_angles[i, 2],
             joint_angles[i, 3],
             joint_angles[i, 4],
             joint_angles[i, 5],])

    position = sim.simxGetObjectPosition(
        clientID, endeffector_handle, -1, sim.simx_opmode_blocking)[1]
    orientation = sim.simxGetObjectOrientation(
        clientID, endeffector_handle, -1, sim.simx_opmode_blocking)[1]
    for i in range(3):
        orientation[i] = round(orientation[i] / math.pi * 180, 2)
    print('The position of the end-effector is ' + str(position))
    print('The orientation of the end-effector is ' + str(orientation))

sim.startSimulation()

print('\nend')
