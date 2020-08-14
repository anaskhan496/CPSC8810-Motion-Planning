Subject: CPSC8810 Motion Planning
Date: 02/09/2020
Submission done by: Mohammad Anas Imam Khan (C17566828) and Ashit Mohanty (C13582787)
Programming Assignment 2 - Local Navigation Forces

Basic Requirements:
1.a) Force: This variable computes the goal force as well as gets added to the avoidance force when a collision between the agent and its neighbours is imminent.

1.b) i: This will loop through all the agents 
Sense_dist: This calculates the distance between the agent and its neighbours 

1.c) rad: Computes the combined radii of the neighbor and the agent
rel_pos: Computes the relative position between the neighbor and the agent
v = Computes the relative velocities between the neighbor and the agent
discr = computes discriminants
ttc = Computes the time to collision between the neighbor and the agent
ttc_min = Checks if the previous value of ttc is less than current ttc. If evaluated to false then this is the minimum time-to-collision between the agent and its corresponding neighbor.
Fa = Computes avoidance force
n = Unit vector of avoidance force

2. np.clip function is used to cap the x and y components of force and velocity respectively.

3. eps - Velocity uncertainty constant

Extra credits:

1) m: magnitude
tau_not: Exponential cut-off point
k: scaling factor

2) nu = Radius of disk from which uncertainty perturbations are sampled
eta = Sampled randomly from a disk centered at 0 having radius nu
samp_vels: creating ramdomised velocities for candidate
theta: creating randomised theta values for candidate velocities
samp_vels_x: x-coordinate for sampled candidate velocity
samp_vels_y: y-coordinate for sampled candidate velocity 

3) The relative velocity calculated is then modified to account for adversarial uncertainty velocities