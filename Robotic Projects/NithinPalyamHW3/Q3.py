import sys
import numpy as np
sys.path.append('PythonAPI')
sys.path.append('zmqRemoteApi')
import math
import time
from zmqRemoteApi import RemoteAPIClient

print ('Program started')
# connect to the coppelia scene
clientID = RemoteAPIClient()
#clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID != -1:
     print ('Connected to remote API server')
else:
     print ('Failed connecting to remote API server')
     sys.exit('Could not connect to Coppelia')

sim= clientID.getObject('sim')

# get the handles of arm joints
arm_handle = sim.getObject('/UR5')
armjoint_handles = np.array()
armjoint_handles[0] = sim.getObject('/UR5/joint')
armjoint_handles[1] = sim.getObject('/UR5/joint/joint')
armjoint_handles[2] = sim.getObject('/UR5/joint/joint/joint')
armjoint_handles[3] = sim.getObject('/UR5/joint/joint/joint/joint')
armjoint_handles[4] = sim.getObject('/UR5/joint/joint/joint/joint/joint')
armjoint_handles[5] = sim.getObject('/UR5/joint/joint/joint/joint/joint/joint')
# get the handles of end effector
endeffector_handle = sim.getObject('/UR5/joint/joint/joint/joint/joint/joint/suctionPad')

# set the arm to position control
for i in range(6):
    sim.simxSetObjectIntParameter(clientID, armjoint_handles[i], 2000, 1, sim.simx_opmode_oneshot)
for i in range(6):
    sim.simxSetObjectIntParameter(clientID, armjoint_handles[i], 2001, 1, sim.simx_opmode_oneshot)


collision_handle_list = []
for i in range(40):
    err_code, collision_handle = sim.simxGetCollisionHandle(clientID, "Collision" + str(i), sim.simx_opmode_blocking)
    sim.simxReadCollision(clientID, collision_handle, sim.simx_opmode_streaming)
    collision_handle_list.append(collision_handle)

def move_arm(armpose):
    armpose_convert = []
    for i in range(len(armpose)):
        armpose_convert.append(round(armpose[i]/180 * math.pi,3))
    sim.simxPauseCommunication(clientID,True)
    for i in range(len(armpose)):
        sim.simxSetJointTargetPosition(clientID, armjoint_handles[i], armpose_convert[i], sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(clientID,False)
    time.sleep(3)

def check_collision(robotCollection, arm_handle):
    sim.addItemToCollection(robotCollection, sim.handle_tree, arm_handle, 0)
    result = sim.checkCollision(robotCollection, sim.handle_all)[0]
    if result == 0:
       return 'No collision detected'
    else:
       print('Collision detected!')
       return 1

joint_angles = np.zeros((10,6))
for i in range(10):
    joint_angles[i,:] = np.random.uniform(-60,60,size=(6))
joint_angles = joint_angles.astype(int)

F = []
#checking collisions for each of those 100 poses
for i in range(100):
    print("Checking pose ", i+1)
    move_arm([joint_angles[i,0],
             joint_angles[i,1],
             joint_angles[i,2],
             joint_angles[i,3],
             joint_angles[i,4],
             joint_angles[i,5],])
    coll = check_collision()
    if coll == 0:
        F.append(joint_angles[i,:])

# close the communication between collision handles
for i in range(40):
    sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_discontinue)

#print(F)
print(len(F), " out of 100 poses are in free space.")

print ('Program ended')