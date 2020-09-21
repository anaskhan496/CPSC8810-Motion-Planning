#!/usr/bin/env python
import rospy
from math import sqrt
from geometry_msgs.msg import PoseArray,Twist
from nav_msgs.msg import Odometry
import tf
import math
import os
import sys
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as img
from PIL import Image

occupancy = np.empty([20,20])
height = 0
width = 0
plt.imshow(img.imread(r'/home/sid/AuE893Spring20_Siddhant/catkin_ws/src/Project/point_map_10.png'))


tb_x_trans = 0.0
tb_y_trans = 0.0
tb_z_trans = 0.0
tb_x_or = 0.0
tb_y_or = 0.0
tb_z_or = 0.0
tb_w_or = 0.0
yaw_tb= 0.0
x_offset = 393
y_offset = 529
scale = 0.01
path = []
path_x_coordinate = []
path_y_coordinate = []


def pose_callback(odom1_msg):
    global tb_x_trans,tb_y_trans,tb_z_trans,tb_x_or,tb_y_or,tb_z_or,tb_w_or,yaw_tb
    tb_x_trans = odom1_msg.pose.pose.position.x
    tb_y_trans = odom1_msg.pose.pose.position.y
    tb_z_trans = odom1_msg.pose.pose.position.z
    tb_x_or = odom1_msg.pose.pose.orientation.x
    tb_y_or = odom1_msg.pose.pose.orientation.y
    tb_z_or = odom1_msg.pose.pose.orientation.z
    tb_w_or = odom1_msg.pose.pose.orientation.w
    quaternion = (
    tb_x_or,
    tb_y_or,
    tb_z_or,
    tb_w_or)

    #convert the quaternion to roll-pitch-yaw
    rpy_tb = tf.transformations.euler_from_quaternion(quaternion)
    roll_tb = rpy_tb[0]
    pitch_tb = rpy_tb[1]
    yaw_tb = rpy_tb[2]

    return tb_x_trans,tb_y_trans,yaw_tb


def occupancy_map_generator():
    global occupancy,height,width
    filename = 'img_inflated_10.csv'
    fp = open(filename,'r')
    lines = fp.readlines()
    fp.close()
    height = len(lines)
    for i in lines:
        x = (i.count(','))
    width = x + 1          #to determine the number of width of image
    occupancy = np.zeros((height,width), dtype= int)
    row = 0
    for line in lines:
        occupancy[row] = line.split(',')
        row = row + 1

def global_planner():
    global x_offset,y_offset,scale,path_x_coordinate,path_y_coordinate
    global tb_x_trans,tb_y_trans,yaw_tb
    rospy.loginfo("global planner")
    rospy.Subscriber('/odom',Odometry,pose_callback)


    Kp_dist = 0.35
    Kp_ang = 0.15
    velocity_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size = 10)
    vel_msg = Twist()
    xgazebo = []
    ygazebo = []
    for j in range(len(path_x_coordinate)):
        xgazebo.append((y_offset - path_y_coordinate[j])*scale)
        ygazebo.append((x_offset - path_x_coordinate[j])*scale)

    rate = rospy.Rate(10)
    for i in range(len(xgazebo)-1):
        distance = (sqrt((tb_x_trans - xgazebo[-i-2])**2 + (tb_y_trans-ygazebo[-i-2])**2))
        angular_error = (math.atan2(ygazebo[-i-2]-tb_y_trans,xgazebo[-i-2]-tb_x_trans))-yaw_tb


        while abs(angular_error) > 0.1:
            vel_msg.linear.x = 0
            vel_msg.angular.z = Kp_ang * angular_error
            velocity_publisher.publish(vel_msg)
            rate.sleep()
            angular_error = (math.atan2(ygazebo[-i-2]-tb_y_trans,xgazebo[-i-2]-tb_x_trans))-yaw_tb
            rospy.loginfo("distance is %f angular error is %f ", distance, angular_error)
        while abs(distance) > 0.1:
            distance = (sqrt((tb_x_trans - xgazebo[-i-2])**2 + (tb_y_trans-ygazebo[-i-2])**2))
            angular_error = (math.atan2(ygazebo[-i-2]-tb_y_trans,xgazebo[-i-2]-tb_x_trans))-yaw_tb
            vel_msg.linear.x = Kp_dist * distance
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            rate.sleep()
            rospy.loginfo("distance is %f angular error is %f ", distance, angular_error)
            # rospy.loginfo("current x %f goal x %f current y %f goal y %f",tb_x_trans,xgazebo[-i-2],tb_y_trans,ygazebo[-i-2])

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
    rospy.loginfo("Reached Goal")

def collision_check(x,y):       #pass the x and y in image coordinate frame
    collision_value = occupancy[y][x]
    if collision_value == 1:
        collision = True
    else:
        collision = False
    return collision

def edge_collision_checker(x1,y1,x2,y2):    #pass x and y in image coordinate frame
    collision = False
    y_delta = y2 - y1
    x_delta = x2 - x1
    theta = (math.atan2(y_delta, x_delta))
    m = (math.tan(theta))
    c = y1 - m * x1

    #iterating for different xs
    x_range = abs(x2 - x1)
    x = x1
    for i in range(int(x_range)):
        y = m*x + c
        collision = (collision_check(int(x),int(y)))
        if collision == True:
            break
        x = x+1
    if collision==True:
        return collision
    #iterating for different ys
    y_range = abs(y2 - y1)
    x = x1
    y = y1
    for i in range(int(y_range)):
        x = (y - c)/m
        collision = (collision_check(int(x),int(y)))
        if collision == True:
            break
        y = y+1
    return collision



class RRTStar():


    class Node():
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0

    def __init__(self, start, goal, rand_area,
                 expand_dis=100.0,
                 path_resolution=60,
                 max_iter=5000,
                 connect_circle_dist=150.0
                 ):


        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand_x = rand_area[0]
        self.max_rand_x = rand_area[1]
        self.min_rand_y = rand_area[2]
        self.max_rand_y = rand_area[3]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_iter = max_iter
        self.node_list = []
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])

    def planning(self, animation=False, search_until_max_iter=True):


        self.node_list = [self.start]
        for i in range(self.max_iter):
            rospy.loginfo("Iter: %d and number of nodes: %d", i,len(self.node_list))
            rand = self.Node(random.randint(self.min_rand_x,self.max_rand_x),random.randint(self.min_rand_y,self.max_rand_y))

            near_index = self.neighbor_index(self.node_list, rand)
            neighbor_node = self.node_list[near_index]
            new_node = self.steer(neighbor_node, rand, self.expand_dis)

            parent_node = new_node.parent
            if edge_collision_checker(new_node.x, new_node.y, parent_node.x, parent_node.y) == False:
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                if new_node:
                    self.node_list.append(new_node)
                    plt.scatter(new_node.x,new_node.y,s=2,c='r')
                    self.rewire(new_node, near_inds)

            if (not search_until_max_iter) and new_node:
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.rrtstar_path(last_index)

        rospy.loginfo("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.rrtstar_path(last_index)
        return None

    def steer(self, source_node, destination_node, delta):
        dist, theta = self.dist_and_angle(source_node,destination_node)
        if delta > dist:
            delta = dist
        next_node = self.Node(source_node.x, source_node.y)
        next_node.path_x = [next_node.x]
        next_node.path_y = [next_node.y]
        n_expand = int(math.floor(delta / self.path_resolution))
        for _ in range(n_expand):
            next_node.x += self.path_resolution * math.cos(theta)
            next_node.y += self.path_resolution * math.sin(theta)
            next_node.path_x.append(next_node.x)
            next_node.path_y.append(next_node.y)
        next_node.parent = source_node
        return next_node

    def dist_and_angle(self,source,destination):
        delta_x = destination.x - source.x
        delta_y = destination.y - source.y
        distance = math.hypot(delta_x,delta_y)
        theta = math.atan2(delta_y,delta_x)
        return distance,theta

    def dist_from_goal(self,x,y):
        delta_x = x - self.end.x
        delta_y = y - self.end.y
        goal_dist = math.hypot(delta_x,delta_y)
        return goal_dist

    def new_cost(self,source,destination):
        dist, _ = self.dist_and_angle(source,destination)
        return source.cost + dist

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node,self.expand_dis)
            if t_node and edge_collision_checker(t_node.x, t_node.y, near_node.x, near_node.y) == False:
                c = self.new_cost(near_node, new_node)
                costs.append(c)
            else:
                costs.append(float("inf"))
        min_cost = min(costs)
        if min_cost == float("inf"):
            return None
        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node,self.expand_dis)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost
        return new_node

    def neighbor_index(self,node_list, rand_node):
        dlist = [(node.x - rand_node.x) ** 2 + (node.y - rand_node.y)
                 ** 2 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    def search_best_goal_node(self):
        dist_to_goal_list = [self.dist_from_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_dis]
        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node,self.expand_dis)
            if edge_collision_checker(t_node.x, t_node.y, self.goal_node.x, self.goal_node.y) == False:
                safe_goal_inds.append(goal_ind)
        if not safe_goal_inds:
            return None
        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i
        return None

    def find_near_nodes(self,new_node):
        nnode = len(self.node_list)+1
        r = self.connect_circle_dist*math.sqrt((math.log(nnode)/nnode))
        r = min(self.expand_dis, r)
        dist_list = [math.sqrt((node.x - new_node.x)**2 + (new_node.y - new_node.y)**2) for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r]
        return near_inds

    def rewire(self,new_node,nearest_nodes):
        for i in nearest_nodes:
            improved_cost = False
            nearby_node = self.node_list[i]
            new_edge_node = self.steer(new_node,nearby_node,self.expand_dis)
            if not new_edge_node:
                continue
            new_edge_node.cost = self.new_cost(new_node,new_edge_node)
            collision = edge_collision_checker(new_node.x, new_node.y, new_edge_node.x, new_edge_node.y)

            if nearby_node.cost > new_edge_node.cost:
                improved_cost = True

            if collision == False and improved_cost:
                self.node_list[i] = new_edge_node
                self.modify_child_cost(new_node)

    def modify_child_cost(self,parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.new_cost(parent_node,node)
                self.modify_child_cost(node)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.dist_and_angle(from_node, to_node)
        return from_node.cost + d


    def rrtstar_path(self,last_index):
        global path_x_coordinate,path_y_coordinate
        path = [[self.end.x,self.end.y]]
        node = self.node_list[last_index]
        pathcost = 0
        path_x_coordinate = [self.end.x]
        path_y_coordinate = [self.end.y]
        while node.parent is not None:

            path.append([node.x,node.y])
            path_x_coordinate.append(node.x)
            path_y_coordinate.append(node.y)
            pathcost = pathcost+node.cost
            node = node.parent
        path.append([self.start.x,self.start.y])
        path_x_coordinate.append(self.start.x)
        path_y_coordinate.append(self.start.y)
        rospy.loginfo(pathcost)
        rospy.loginfo(path_x_coordinate)
        rospy.loginfo(path_y_coordinate)
        plt.plot(path_x_coordinate,path_y_coordinate)
        plt.show(block = False)
        path.append([self.start.x,self.start.y])
        return path,path_x_coordinate,path_y_coordinate

def main():

    global height, width
    occupancy_map_generator()

    # Set Initial parameters
    rrt_star = RRTStar(start=[191,588],
                       goal=[464, 350],
                       rand_area=[0,width-1,0, height-1],
                       )
    path = rrt_star.planning()

    if path is None:
        rospy.loginfo("Cannot find path")
    else:
        rospy.loginfo("found path!!")
        rospy.Subscriber('/odom',Odometry,pose_callback)
        global_planner()

if __name__ == '__main__':
    rospy.init_node('planner')
    while not rospy.is_shutdown():
        main()
