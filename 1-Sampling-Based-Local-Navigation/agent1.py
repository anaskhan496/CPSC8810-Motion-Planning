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
    ttc = []
    rad = []
    gv = []
    vmag = []
    min = 999999999999999999
    indx = 0
    new_vel = 0
    samp_vels_final = np.zeros(2)
    def __init__(self, csvParameters, dhor = 10, goalRadiusSq=1):
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
        Agent.instance.append(self.id) #Appending id of Agents
        Agent.position.append(self.pos) #Appending position of agents
        Agent.rad.append(self.radius) #Appending radius of agents
        Agent.gv.append(self.gvel) #Appending goal velocity of Agents
        Agent.vmag.append(self.vel) #Appending velocity of Agents


    def computeNewVelocity(self, neighbors=[]):
        """
            Your code to compute the new velocity of the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.
        """
        dist02=np.sqrt(((np.array(Agent.position)[0,0])-(np.array(Agent.position)[2,0]))**2+((np.array(Agent.position)[0,1])-(np.array(Agent.position)[2,1]))**2)-np.array(Agent.rad)[0]-np.array(Agent.rad)[1]
        dist01=np.sqrt(((np.array(Agent.position)[1,0])-(np.array(Agent.position)[2,0]))**2+((np.array(Agent.position)[1,1])-(np.array(Agent.position)[2,1]))**2)-np.array(Agent.rad)[1]-np.array(Agent.rad)[2]
        if (dist01 <= self.dhor or dist02<= self.dhor):
            neighbors=(np.array([dist02,dist01]))

        #Agent.max=(np.arange(0,self.maxspeed,0.2)).tolist()
            Agent.max = np.random.uniform(0,self.maxspeed**2,1000)
            Agent.theta=np.random.uniform(0,2*np.pi,1000)
            #for x in range(len(Agent.max)):
            #    for y in range(len(Agent.theta)):
            #        samp_vels_x.append(math.sqrt(Agent.max[x])*math.cos(math.radians(Agent.theta[y])))
            #        samp_vels_y.append(math.sqrt(Agent.max[x])*math.sin(math.radians(Agent.theta[y])))


            samp_vels = (np.array([np.sqrt(Agent.max)*np.cos(Agent.theta),np.sqrt(Agent.max)*np.sin(Agent.theta)]))
            samp_vels  = np.transpose(samp_vels)
            samp_vels_mag = np.sqrt(samp_vels[:,0]**2 + samp_vels[:,1]**2)
            #vel_mag = np.sqrt(np.array(Agent.vmag)[:,0]**2 + np.array(Agent.vmag)[:,1]**2)

            #samp_vels_y = np.sqrt(Agent.max)*np.sin(Agent.theta)
            #print(np.shape(samp_vels))
            #plt.scatter(samp_vels[:,0], samp_vels[:,1])
            #plt.show()
            alpha = 1
            beta = 1
            gamma = 10
            discr =[]
            discr_fill=[]

            for i in range(len(Agent.instance)-1):
                R = Agent.rad[2] + Agent.rad[i]
                W = (np.array(Agent.position)[2,:] - np.array(Agent.position)[i,:])
                Wmag = (np.sqrt(W[0]**2+W[1]**2))
                #c = np.dot(Wmag,Wmag) - R*R
                c = np.dot(W,W) - R*R

                if c<0:
                    print("Kat gaya")
                for j in range(len(samp_vels_mag)):
                    Vgx = (np.array(samp_vels)[j,0] - np.array(Agent.gv)[i,0])**2 #vcandx - vogalx
                    Vgy = (np.array(samp_vels)[j,1] - np.array(Agent.gv)[i,1])**2 #vcandy - vgoaly
                    Vax = (np.array(samp_vels)[j,0] - np.array(Agent.vmag)[2,0])**2 #vcandx - vagent_actualx
                    Vay = (np.array(samp_vels)[j,1] - np.array(Agent.vmag)[2,1])**2 #vcandy - vagent_actualy

                    Vx = samp_vels[j,0] - np.array(Agent.vmag)[i,0]
                    Vy = samp_vels[j,1] -np.array(Agent.vmag)[i,1]
                    V1 = np.array([Vx,Vy])
                    V = np.sqrt(Vx**2+Vy**2)
                    a  = np.dot(V1,V1)
                    b = np.dot(W,V1)
                #if b.all()>0:
                    #print("Agents are diverging")
                #discr_fill[discr_fill<=0] = float("inf")
                    if (b*b-a*c) > 0:
                        discr = (b*b-a*c)
                        tau1 = c/(-b+np.sqrt(discr))
                        tau2 = c/(-b-np.sqrt(discr))
                        if tau1 and tau2 > 0:
                            if tau1<tau2:
                                #Agent.ttc.append(tau1)
                                fitting_function = alpha*np.sqrt(Vgx + Vgy) + beta*np.sqrt(Vax + Vay) + np.true_divide(gamma,tau1)
                                if fitting_function.all() < Agent.min :
                                    Agent.min = fitting_function
                                    Agent.indx = j
                            else:
                                #Agent.ttc.append(tau2)
                                fitting_function = alpha*np.sqrt(Vgx + Vgy) + beta*np.sqrt(Vax + Vay) + np.true_divide(gamma,tau2)
                                if fitting_function.all() < Agent.min :
                                    Agent.min = fitting_function
                                    Agent.indx = j
                        if self.id is 2:
                            Agent.samp_vels_final = np.array([samp_vels[Agent.indx,0],samp_vels[Agent.indx,1]])
                        else:
                            Agent.samp_vels_final = np.multiply(np.array(samp_vels)[Agent.indx,:],-1)
                    #print(samp_vels_final)
                    #print(Agent.indx)
                    Agent.min = 999999999999999999


            """for k in range(len(samp_vels)):
                Vgx = (np.array(samp_vels)[k,0] - np.array(Agent.gv)[i,0])**2 #vcandx - vogalx
                Vgy = (np.array(samp_vels)[k,1] - np.array(Agent.gv)[i,1])**2 #vcandy - vgoaly
                Vax = (np.array(samp_vels)[k,0] - np.array(Agent.vmag)[-1,0])**2 #vcandx - vagent_actualx
                Vay = (np.array(samp_vels)[k,1] - np.array(Agent.vmag)[-1,1])**2 #vcandy - vagent_actualy

                #np.seterr(divide='ignore', invalid='ignore')
                #if Agent.ttc > 0:
                fitting_function = alpha*np.sqrt(Vgx + Vgy) + beta*np.sqrt(Vax + Vay) + np.true_divide(gamma,Agent.ttc)
                #fitting_function[~np.isfinite(fitting_function)] = 0
                #idx = np.where(fitting_function==float(0))
                #if fitting_function.all()==0:
                #    fitting_function=np.delete(fitting_function,idx)
                #print(np.min(fitting_function))"""


        #print(np.array(Agent.position)[0,1]
        #print(range(len(Agent.instance)))
        #print("distAgent_Up",dist01)
        #print("distAgent_Down",dist02)

            if not self.atGoal:
                self.vnew[:] = Agent.samp_vels_final[:]   # here I just set the new velocity to be the goal velocity
                #print("vnew",np.shape(self.vnew))
                #print("vnew",np.shape(Agent.samp_vels_final))

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
