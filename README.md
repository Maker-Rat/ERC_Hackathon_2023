# ERC_Hackathon_2023
This is our teams submission for the ERC Hackathon 2023

Team Members :
  1) Parth Shah
  2) Ajinkya Deshpande
  3) Sharvil Potdar
  4) Nayan Gogari
  5) Ritwik Sharma


# Automation (Perception, PLanning and Control)
1) **Planning** : 
   We've implemented RRT and A* path planning algorithms to solve the presented problem. For A* grid generation, we have implemented a hardcoded regular grid as well as a PRM (ProbabIlistic Random Mapping) based approach. Since the given maze
   is relatively rectangular the harcoded grid approach tends to provide the best results. The density of the grid can be altered by modifying a single value in the code.

3) **Control** : 
   After an unsuccessful attempt at implementing PID for the problem, we switched to LQR (Linear Quadratic Regulator). This made more sense given the multi-variable control nature of the problem. LQR utilizes a state space model of the
   system where the state matrix consists of the x coordinate, y coordinate and the yaw angle while the control input matrix consists of linear velocity and angular velocity. LQR operates by solving the quadratic Ricatti loss equation for
   stable poles to find the gain matrix K. The controller input is then calculated as (scaled reference) - (K * X), where X is the current state vector.


   As can be observed in the provided video, the robot tries to rigorously follow the path planned and hence can be slow at certain points while trying to calibrate. This can be further improved by fine tuning the Q and R matrices. One more
   interesting approach to tune these matrices could be a learning based approach using RL or deep learning.
    
# Electronics
# Mechanical
