# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
from scipy.spatial import distance as rel_dist
import numpy as np


disk_robot = True #(change this to False for the advanced extension)
obstacles = None # the obstacles
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap,
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class

def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius

    obstacles = scene_obstacles # setting the global obstacle variable

    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)
    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
    robot_radius = max(robot_width, robot_height)/2.
    sensitivity = 0.3 # 0 means no movement, 1 means max distance is init_dist - To add noise in the sample space

    # create regularly spaced neurons
    x = np.linspace(x_limit[0], x_limit[1], 40)
    y = np.linspace(y_limit[0], y_limit[1], 40)
    coords = np.stack(np.meshgrid(x, y),-1).reshape(-1,2)   # Generating a meshgrid of x and y samples

    # compute spacing
    init_dist = np.min((x[1]-x[0], y[1]-y[0]))
    min_dist = init_dist * (1 - sensitivity)

    # perturb points
    perturbation = (init_dist - min_dist)/2
    noise = np.random.uniform(low=-perturbation,high=perturbation,size=(len(coords), 2)) # Creating a linspace array of noise based on the perturbation
    coords += noise                                                                      # Adding noise to the points in ssample space
    samples_x = coords[:,0]
    samples_y = coords[:,1]
    # the roadmap
    graph = Roadmap()

    # This loop will add all the vertices in the obstacle free configuration space
    for i in range(len(samples_x)):
        c = 0
        for j in range(len(obstacles)):
            if ((obstacles[j].x_min-robot_radius)<=samples_x[i]<=(obstacles[j].x_max+robot_radius)) and ((obstacles[j].y_min-robot_radius)<=samples_y[i]<=(obstacles[j].y_max+robot_radius)):
                c = 1
        if c == 0:
            samples=[samples_x[i],samples_y[i]]
            graph.addVertex(samples)

    # This loop is used to add the edges between the vertices that are within a certain distance from the vertex
    for vertice in graph.vertices:
        for poss_neighbor in graph.vertices:
            if vertice!=poss_neighbor:
                distance=math.sqrt((vertice.q[0]-poss_neighbor.q[0])**2 + (vertice.q[1]-poss_neighbor.q[1])**2)
                if distance < 10:
                    if interpolate(vertice,poss_neighbor,5,obstacles,distance,robot_radius) == True:   # Calling the incremental sampling function to avoid the obstacles
                        if poss_neighbor.connectedComponentNr == -1 and poss_neighbor.getEdge(vertice.id)==None: # Connecting configurations if they are not previously connected
                            graph.addEdge(vertice,poss_neighbor,distance)
                            poss_neighbor.connectedComponentNr = vertice.id
                        else: #If they are already connected, checking parents
                            x=path_find(vertice,graph.vertices)
                            y=path_find(poss_neighbor,graph.vertices)
                            z=len(list(set(x) & set(y))) # Checking for common parents between two configurations using set intersection
                            if z == 0 and poss_neighbor.getEdge(vertice.id)==None:
                                graph.addEdge(vertice,poss_neighbor,distance)
                                poss_neighbor.connectedComponentNr = vertice.id

    # uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")
    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None

def find_path(q_start, q_goal, graph):
    global sce

    path  = []
    heuristic = [] # Store the heuristic from all the nodes to the goal node

    qs = graph.addVertex((q_start[0], q_start[1]))   # Add start vertex
    qg = graph.addVertex((q_goal[0], q_goal[1]))     # Add goal vertex

    for h in graph.vertices:
        heuristic.append(math.sqrt((q_goal[0] - h.q[0])**2 + (q_goal[1] - h.q[1])**2))
    sce.getObstacles()  # Call the getObstacles method from scene.py
    for node in graph.vertices:
        if math.sqrt((q_start[0] - node.q[0])**2 + (q_start[1] - node.q[1])**2) <= 6: # Distance between start and all nodes
            if interpolate(qs,node,5,obstacles,math.sqrt((q_start[0] - node.q[0])**2 + (q_start[1] - node.q[1])**2),2.0):
                graph.addEdge(node,qs, math.sqrt((q_start[0] - node.q[0])**2 + (q_start[1] - node.q[1])**2)) # Add edge between start vertex and its nearby vertices
        if math.sqrt((q_goal[0] - node.q[0])**2 + (q_goal[1] - node.q[1])**2) <= 6: # Distance between goal and all nodes
            if interpolate(qg,node,5,obstacles,math.sqrt((q_goal[0] - node.q[0])**2 + (q_goal[1] - node.q[1])**2),2.0):
                graph.addEdge(node,qg,  math.sqrt((q_goal[0] - node.q[0])**2 + (q_goal[1] - node.q[1])**2)) # Add edge between goal vertex and its nearby vertices

    parent =  np.empty(graph.getNrVertices(), dtype = (tuple,2))
    q_start_2d = (q_start[0], q_start[1])
    q_goal_2d = (q_goal[0], q_goal[1])
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()

    start_heuristic = [i for i in range(len(heuristic)) if heuristic[i] == math.sqrt((q_start[0] - q_goal[0])**2 + (q_start[1] - q_goal[1])**2)] # Find the index where heuristic value is equal to start and goal node distance
    g = 0
    heuristic_start = heuristic[start_heuristic[0]]
    f = g+heuristic_start
    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)
    open_set.put(q_start_2d, Value(f=f,g=g))
    get_my_path_to = q_goal_2d
    # Implementing A*
    while len(open_set)!=0:
        current_node,current_node_add=open_set.pop()
        current_node_2d = (current_node[0],current_node[1])
        closed_set.add(current_node)
        if current_node[0]==q_goal[0] and current_node[1]==q_goal[1]:
            print("Goal reached")
            # Tracing path back from goal to start
            while get_my_path_to[0] != q_start_2d[0] and get_my_path_to[1] != q_start_2d[1] :
                for vertice in graph.vertices:
                    if vertice.q[0] == get_my_path_to[0] and vertice.q[1] == get_my_path_to[1]:
                        back_track = parent[vertice.id]
                        path.append(back_track)
                        get_my_path_to = back_track
                        break
            break
        for vertice in graph.vertices:
            if vertice.q[0] == current_node_2d[0] and vertice.q[1] == current_node_2d[1]:
                for edge in vertice.edges:
                    child_node = (graph.vertices[edge.id].q[0], graph.vertices[edge.id].q[1])
                    if child_node in closed_set:
                        continue
                    else:
                        g_child = current_node_add.g + edge.dist
                        if child_node not in open_set or open_set.get(child_node).g > g_child:
                            f_child = g_child + heuristic[edge.id]
                            open_set.put(child_node, Value(f_child,g_child))
                            parent[edge.id] = (current_node[0],current_node[1])

    path.reverse()  # This is being used to reverse the path so that the dots go from blue to green as we progress towards the goal node
    return path

# ----------------------------------------
# below are some functions that you may want to populate/modify and use above
# ----------------------------------------

def nearest_neighbors(graph, q, max_dist=10.0):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances
    """

    return None, None


def k_nearest_neighbors(graph, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q.
        You may also want to return the corresponding distances
    """

    return None

def distance (q1, q2):
    """
        Returns the distance between two configurations.
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration
    """

    return None

def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.
    """


    return False


def interpolate (q1, q2, stepsize, obstacles, dist, radius):
    """
        Returns an interpolated local path between two given configurations.
        It can be used to determine whether an edge between vertices is collision-free.
    """
    p=int(dist/stepsize)
    x_spacing=(q2.q[0]-q1.q[0])/(p+1)
    y_spacing=(q2.q[1]-q1.q[1])/(p+1)
    co=0

    for i in range(1,(p+1)):
        x_coord=q1.q[0] + i*x_spacing
        y_coord=q1.q[1] + i*y_spacing
        for j in range(len(obstacles)):
            if (obstacles[j].x_min-radius<=x_coord<=obstacles[j].x_max+radius) and (obstacles[j].y_min-radius<=y_coord<=obstacles[j].y_max+radius):
                co = 1
    if co == 0:
        return True

def path_find (p1, collection):
    """ To trace back and find the parents of the particular configuration. In order to achieve this, first the previous parents are stored
    with the help of connectedComponentNr and then the vertices of every edge from the set of parents are computed and returned as a list"""
    c1=p1
    road1=[]
    counter=0
    while(c1.connectedComponentNr!=-1):
        parent=c1.connectedComponentNr
        road1.append(parent)
        for i in collection:
            if parent == i.id:
                counter=i
        c1=counter
    parent=c1.id
    road1.append(parent)
    road2 = []
    parent_edges = []
    for i in road1:
        for j in collection:
            if j.id == i:
                counter = j
        parent_edges = counter.getEdges()
        for k in parent_edges:
            x=k.src_id
            road2.append(x)
            y=k.dest_id
            road2.append(y)
    return road2

def interpolate_nodes (q1, q2, stepsize):
    """
        Returns an interpolated local path between two given configurations.
        It can be used to determine whether an edge between vertices is collision-free.
    """
    x_steps = (q2[0] - q1[0])/(stepsize+1)
    y_steps = (q2[1] - q1[1])/(stepsize+1)

    return [[q1[0] + i*x_steps,q1[1] + i*y_steps]
            for i in range(1,stepsize+1)]

if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk
    import scene

    win = tk.Tk()
    sce = Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
