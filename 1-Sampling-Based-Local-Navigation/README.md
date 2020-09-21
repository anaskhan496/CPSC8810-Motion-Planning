CPSC 8810 - Motion Planning
Assignment 1: Sampling-Based Local Navigation
Submitted by: Ashit Mohanty, Mohammad Anas Imam Khan
Date of submission: 01-26-2020 

Programming variables:

Global objects:
1) np : object of numpy class
2) mat_plot : object of the matplotlib.pyplot class

Local variables for computeNewVelocity function
1) near_goal : variable to compute distance between agent position and goal position
2) alpha : constant used in cost function weighing relative velocity between candidate and goal velocity
3) beta : constant used in cost function weighing relative velocity between candidate and agent's current velocity
4) gamma : constant used in cost function weighing time to collision
5) N_samples : sample size to declare random velocities in velocity space
6) samp_vels[] : variable to store sampled velocities
7) theta[] : variable to store sampled angle values
8) nb_pos[] : to store position of neighbor agents
9) nb_size[] : to store agent size i.e. radius
10) nb_id : to store id's of the agents
11) nb_vel : storing the velocity of agents
12) candidate_vel : sampled admissible velocities
13) samp_vels_x : storing the x-coordinate of the sampled velocity
14) samp_vels_y : storing the y-coordinate of the sampled velocity
15) fitting function : Evaluating the fitness of each of the N candidate velocities
16) ttc : Calculates the time to collision
17) min_cost_vel : The candidate velocity that has the minimum cost