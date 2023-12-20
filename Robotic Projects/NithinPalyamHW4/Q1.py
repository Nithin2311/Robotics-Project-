import numpy as np
import sys
import math
import time
import sim.py
from collections import defaultdict, deque
from zmqRemoteApi import RemoteAPIClient
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


class Paths():
    def __init__(self, p0, p1):
        p0_array = np.asarray(p0)
        p1_array = np.asarray(p1)
        self.p = p0_array
        self.dirn = np.linalg.norm(p1_array - p0_array, axis=0, keepdims=True)

    def path(self, t):
        return self.p + t * self.dirn


def in_path(paths, center, r):
    a = np.linalg.norm(paths.dirn)
    b = 2 * np.linalg.norm(paths.dirn) * np.linalg.norm(paths.p - center)
    c = np.linalg.norm(paths.p - center) ** 2 - r ** 2

    disc = b * b - 4 * a * c
    sqrt_disc = np.sqrt(disc)

    t1 = (-b + sqrt_disc) / (2 * a)
    t2 = (-b - sqrt_disc) / (2 * a)

    if disc < 0 or ((t1 < 0 and t2 < 0) or (t1 > paths.dist and t2 > paths.dist)):
        return False

    return True


def nearest(g, target_vertex, oscs, r):
    nearest_vertex = None
    nearest_index = None

    min_dist = np.inf

    for idx, v in enumerate(g.vertices):
        paths = Paths(v, target_vertex)
        for obs in oscs:
            if in_path(paths, obs, r):
                break

        dist = np.linalg.norm(np.array(v) - np.array(target_vertex))
        if dist < min_dist:
            min_dist = dist
            nearest_index = idx
            nearest_vertex = v

    return {'vertex': nearest_vertex, 'index': nearest_index}


def calculate_and_return_new_vertex(rand_vex, near_vex, step_size):
    dirn = np.array(rand_vex) - np.array(near_vex)
    length = np.linalg.norm(dirn)
    min_length = min(step_size, length)
    dirn = (dirn / length) * min_length
    new_vex = np.array([near_vex[0] + dirn[0], near_vex[1] + dirn[1]])
    return new_vex


class Graph:
    def __init__(self, start_pose, goal_pose):
        self.start_pose = start_pose
        self.goal_pose = goal_pose

        self.vertices = [start_pose]
        self.edges = []

        self.vertex_to_index = {start_pose: 0}
        self.neighbors = {0: []}
        self.distances = {0: 0.}

        self.start_to_goal_x = goal_pose[0] - start_pose[0]
        self.start_to_goal_y = goal_pose[1] - start_pose[1]

        self.success = False

    def add_vertex(self, pos):
        idx = self.vertex_to_index.setdefault(pos, len(self.vertices))
        if idx == len(self.vertices):
            self.vertices.append(pos)
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        if idx1 in self.neighbors and idx2 in self.neighbors and idx1 != idx2:
            self.edges.append((idx1, idx2))
            self.neighbors[idx1].append((idx2, cost))
            self.neighbors[idx2].append((idx1, cost))

    def random_position(self):
        rx = np.random.random()
        ry = np.random.random()

        if self.sx != 0 and self.sy != 0:
            pos_x = self.start_pose[0] - (self.sx / 2.) + rx * self.sx * 2
            pos_y = self.start_pose[1] - (self.sy / 2.) + ry * self.sy * 2
        else:
            pos_x = self.start_pose[0]
            pos_y = self.start_pose[1]
        return pos_x, pos_y


def calculate_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))


def is_within_obstacle(rand_vex, oscs, r, g):
    for obs in oscs:
        dist = calculate_distance(obs, rand_vex)
        if dist < r:
            return True
    return False


def RRT(start_pose, goal_pose, oscs, n_iter, r, step_size):
    g = Graph(start_pose, goal_pose)
    for _ in range(n_iter):
        rand_vex = g.random_position()
        if is_within_obstacle(rand_vex, oscs, r, g):
            continue

        near_vex, near_index = nearest(g, rand_vex, oscs, r)
        if near_vex is None:
            continue

        new_vex = calculate_and_return_new_vertex(
            rand_vex, near_vex, step_size)

        new_index = g.add_vertex(new_vex)
        dist = calculate_distance(new_vex, near_vex)
        g.add_edge(new_index, near_index, dist)

        dist = calculate_distance(new_vex, g.goal_pose)
        if dist < 2 * r:
            end_idx = g.add_vertex(g.goal_pose)
            g.add_edge(new_index, end_idx, dist)
            g.success = True
            print('success')
            break
    return g


def dijkstra(graph):
    start_index = graph.vertex_to_index[graph.start_pose]
    goal_index = graph.vertex_to_index[graph.goal_pose]
    nodes = list(graph.neighbors.keys())

    dist = defaultdict(lambda: math.inf)
    prev = defaultdict(lambda: None)
    dist[start_index] = 0

    while nodes:
        current_node = min(nodes, key=lambda node: dist[node])
        nodes.remove(current_node)

        if dist[current_node] == math.inf:
            break

        for neighbor, cost in graph.neighbors[current_node]:
            new_cost = dist[current_node] + cost
            if new_cost < dist[neighbor]:
                dist[neighbor] = new_cost
                prev[neighbor] = current_node

    path = deque()
    current_node = goal_index
    while prev[current_node] is not None:
        path.appendleft(graph.vertices[current_node])
        current_node = prev[current_node]

    path.appendleft(graph.vertices[current_node])
    return list(path)


if __name__ == '__main__':
    start_pose = (0., 0.)
    goal_pose = (5., 5.)
    oscs = [(1., 1.), (2., 2.)]
    n_iter = 200
    r = 0.5
    step_size = 0.7
    g = RRT(start_pose, goal_pose, oscs, n_iter, r, step_size)
    if g.success:
        path = dijkstra(g)
        path.plot(g, oscs, r, path)
        print(path)

print('\nProgram ended')
