# agent.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#Code submitted by Mohammad Ans Imam Khan (C17566828) and Ashit Mohanty (C13582787)
import numpy as np
import math
from scipy.spatial import distance as rel_dist
from math import sqrt
import matplotlib.pyplot as mat_plot

class Agent(object):
    def __init__(self, csvParameters, dhor = 10, goalRadiusSq=1):
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
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent

    def computeNewVelocity(self, neighbors=[]):
        """
            Your code to compute the new velocity of the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.
        """
        near_goal = rel_dist.euclidean(self.goal,self.pos)
        if near_goal > 2:
            #Initializing function variables
            alpha = 1250
            beta = 1250
            gamma = 2500
            N_samples = 350     #declaring sample size
            samp_vels = []
            theta = []
            samp_vels = []
            nb_pos = []
            nb_size = []
            nb_id = []
            nb_vel = []

            # Sampling N velocities in the admissible velocity space
            candidate_vel = np.zeros((N_samples,2))
            samp_vels = np.random.uniform(low = 0, high = (self.maxspeed)**2, size = N_samples)     #creating ramdomised velocities for candidate
            theta = np.random.uniform(low = 0, high = 2*(np.pi), size = N_samples)                 #creating randomised theta values for candidate velocities
            samp_vels_x = np.sqrt(samp_vels)*np.cos(theta)                                         #x-coordinate for sampled candidate velocity
            samp_vels_y = np.sqrt(samp_vels)*np.sin(theta)                                         #y-coordinate for sampled candidate velocity
            candidate_vel[:,0] = samp_vels_x
            candidate_vel[:,1] = samp_vels_y
            candidate_vel_mag=np.sqrt(candidate_vel[:,0]**2 + candidate_vel[:,1]**2)
            #mat_plot.scatter(candidate_vel[:,0], candidate_vel[:,1])
            #mat_plot.show()

            for neighbor in neighbors:
                if self.id != neighbor.id:                                                        #ignoring the same agent as its own neighbor
                    sense_dist = rel_dist.euclidean(neighbor.pos, self.pos) -1.5                  #computing distance to determine the agent's neighbors
                    if sense_dist < self.dhor:                                                   #determining neighbors
                        nb_id.append(neighbor.id)
                        nb_pos.append(neighbor.pos)
                        nb_size.append(neighbor.radius)
                        nb_vel.append(neighbor.vel)

            fitting_function = np.zeros(len(candidate_vel))
            if len(nb_id)!=0:
                for i in range(len(candidate_vel)):
                    ttc = np.zeros(len(nb_id))
                    for j in range(len(nb_id)):
                        rad = self.radius + nb_size[j]                                              #computing the total radius
                        rel_pos = self.pos - nb_pos[j]
                        c = rel_pos.dot(rel_pos) - rad*rad
                        v = candidate_vel[i] - nb_vel[j]                                            #computing relative velocity
                        a = v.dot(v)
                        b = rel_pos.dot(v)
                        if c<0:
                            ttc[j]=0
                        discr = b*b - a*c                                                           #computing discriminant
                        if b>0:
                            ttc[j] = float('inf')
                        if discr <= 0:
                            ttc[j] = np.inf
                        if discr > 0:
                            tau = c / (-b + np.sqrt(discr))
                            if tau < 0:
                                ttc[j] = np.inf
                            else:
                                ttc[j] = tau                                                        #storing the value of time to collision
                    ttc_min = np.amin(ttc)
                    fitting_function[i] =  alpha * rel_dist.euclidean(candidate_vel[i], self.gvel) + beta * rel_dist.euclidean(candidate_vel[i], self.vel) + gamma/ttc_min
                min_idx_loc = np.argmin(fitting_function)
                min_cost_vel = candidate_vel[min_idx_loc]

                if not self.atGoal:
                    self.vnew[:] = min_cost_vel[:]
            else:
                self.vnew[:] = self.gvel[:]

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
