# agent.py
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
import matplotlib.pyplot as plt

class Agent(object):
    position=[]
    instance = []
    max=[]
    theta=[]
    #ttc = []
    rad = []
    gv = []
    vmag = []
    min = 999999999999999999
    indx = 0
    new_vel = 0
    samp_vels_final = np.zeros(2)
    neighbours=np.zeros(shape=(3,2))
    def __init__(self, csvParameters, dhor = 5, goalRadiusSq=1):
        """
            Takes an input line from the csv file,
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent
        #print(np.shape(self.pos))
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel)))*self.prefspeed
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent
        """Agent.instance.append(self.id) #Appending id of Agents
        Agent.position.append(self.pos) #Appending position of agents
        Agent.rad.append(self.radius) #Appending radius of agents
        Agent.gv.append(self.gvel) #Appending goal velocity of Agents
        Agent.vmag.append(self.vel)""" #Appending velocity of Agents

    def ngbrs():
        for i in range(len(Agent.instance)):
            c=0
            for j in range(len(Agent.instance)):
                if(i!=j):
                    dist=np.sqrt((np.array(Agent.position)[i,0]-np.array(Agent.position)[j,0])**2+(np.array(Agent.position)[i,1]-np.array(Agent.position)[j,1])**2)
                    Agent.neighbours[i,c] =dist
                    #print(dist)
                    #print(Agent.neighbours)
                    c += 1
        return Agent.neighbours

    def computeNewVelocity(self,neighbors=[]):
        """
            Your code to compute the new velocity of the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.
        """
        Agent.ngbrs()
        #print(Agent.neighbours)
        Agent.max = np.random.uniform(0,self.maxspeed**2,1000)
        Agent.theta=np.random.uniform(0,2*np.pi,1000)
        samp_vels_x = np.sqrt(Agent.max)*np.cos(Agent.theta)
        samp_vels_y = np.sqrt(Agent.max)*np.sin(Agent.theta)
        fitting_function = np.zeros(len(samp_vels_x))


                    else:
                        self.vnew[:] = self.gvel[:]
                        #print("org",self.vnew)

    def update(self, dt):
        """
            Code to update the velocity and position of the agent
            as well as determine the new goal velocity
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed
