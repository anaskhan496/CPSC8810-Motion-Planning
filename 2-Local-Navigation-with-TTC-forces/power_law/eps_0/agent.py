# simulator.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
import math
from math import sqrt
from scipy.spatial import distance as rel_dist
import random

class Agent(object):
    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=5, goalRadiusSq=1, maxF = 10):
        """
            Takes an input line from the csv file,
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq = goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = 10 # the sensing radius
        self.timehor = 5 # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = 10 # the maximum force that can be applied to the agent

    def computeForces(self, neighbors=[]):
        """
            Your code to compute the forces acting on the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """

        #Initializing function variables
        eps = 0
        tau_not = 3
        k = 1
        m = 2
        ttc_min = float('inf')
        Force = (self.gvel - self.vel)/self.ksi                                 #Goal force
        for i in range(len(neighbors)):
            if self.id != neighbors[i].id and not neighbors[i].atGoal:          #ignoring the same agent as its own neighbor
                sense_dist = sqrt((neighbors[i].pos - self.pos).dot(neighbors[i].pos - self.pos)) - (self.radius + neighbors[i].radius) #computing distance to determine the agent's neighbors
                if sense_dist < self.dhor:
                    rad = self.radius + neighbors[i].radius                     #computing the total radius
                    rel_pos = self.pos - neighbors[i].pos
                    c = rel_pos.dot(rel_pos) - rad*rad
                    v = self.vel - neighbors[i].vel                             #computing relative velocity
                    a = v.dot(v) - eps**2
                    b = rel_pos.dot(v) - eps*rad
                    discr = (rel_pos.dot(v))**2 - (v.dot(v))*(rel_pos.dot(rel_pos)- rad*rad) #computing discriminant
                    if b>0:
                        ttc = float('inf')
                    if discr <= 0:
                        ttc = np.inf
                    if discr > 0:
                        tau = c / (-b + np.sqrt(discr))
                        if tau < 0:
                            ttc = np.inf
                        else:
                            ttc = tau
                    if ttc!=float('inf'):
                        if ttc<ttc_min:
                            ttc_min = ttc
                            Fa = (k*math.exp(-(ttc_min/tau_not))/(ttc_min**(m+1)))*(m + ttc_min/tau_not)*((rel_pos+v*ttc_min)/sqrt(discr)) #Power-Law based
                            Force = Force + Fa
                            if not self.atGoal:
                                self.F = Force
                                self.F = np.clip(self.F, -self.maxF, self.maxF)
            else:
                self.F = Force
                self.F = np.clip(self.F, -self.maxF, self.maxF)

    def update(self, dt):
        """
            Code to update the velocity and position of the agents.
            as well as determine the new goal velocity
        """
        if not self.atGoal:
            self.vel += self.F*dt    #update the velocity
            self.vel = np.clip(self.vel, -self.maxspeed, self.maxspeed)
            self.pos+= self.vel*dt   #update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed
