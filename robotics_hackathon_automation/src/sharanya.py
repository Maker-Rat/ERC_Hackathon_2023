#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.79, Y + 0.66).
#you need to name this node "controller"

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import geometry_msgs
from math import sqrt, atan, atan2, tan, pi
import json
from tf.transformations import euler_from_quaternion
import ast
import numpy as np
import time

np.set_printoptions(precision=3,suppress=True)

BURGER_MAX_LIN_VEL = 0.2
BURGER_MAX_ANG_VEL = 1.5

path = []
cur_x = None
cur_y = None
cur_z = None
prev_x = None
prev_z = None

esum_x = 0
esum_z = 0

Kp_x = 0.05
Ki_x = 0.000000000001

Kp_z = 0.1
Ki_z = 0.000000000000

cur_yaw = None

prev_yaw = 0

prev_time = 0
cur_time = 0

def dist(x1, y1, x2, y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def store_path(data):
   global path
   if data.data:
    path = ast.literal_eval(data.data)


def get_pos(msg):
    global cur_x, cur_z, cur_y, cur_yaw
    cur_x = msg.pose.pose.position.x
    cur_y = msg.pose.pose.position.y
    cur_z = msg.pose.pose.position.z

    rot_q = msg.pose.pose.orientation
    (roll, pitch, cur_yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    #print(cur_yaw)

def getB(yaw, deltat):
    B = np.array([  [np.cos(yaw)*deltat, 0],
                                    [np.sin(yaw)*deltat, 0],
                                    [0, deltat]])
    return B


def lqr(actual_state_x, desired_state_xf, Q, R, A, B, dt):
    x_error = actual_state_x - desired_state_xf

    N = 50
    P = [None] * (N + 1)
     
    Qf = Q
    P[N] = Qf

    for i in range(N, 0, -1):
        P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(
            R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)      

    K = [None] * N
    u = [None] * N
 
    for i in range(N):
        K[i] = -np.linalg.pinv(R + B.T @ P[i+1] @ B) @ B.T @ P[i+1] @ A
        u[i] = K[i] @ x_error
 
    u_star = u[N-1]
 
    return u_star


def get_gradient(x1, x2, y1, y2):
    grad = 0.0
    dx = x2 - x1
    dy = y2 - y1

    if (dx != 0):

        grad = atan(dy / dx)

        if (dy == 0 and dx < 0):
            grad = pi
        elif (dy > 0 and dx < 0):
            grad = pi + grad
        elif (dy < 0 and dx < 0):
            grad = -pi + grad
        elif (dy < 0 and dx > 0):
            grad = grad

    else:
        if (dy > 0):
            grad = pi / 2
        elif (dy < 0):
            grad = -pi / 2
        else:
            grad = 0.0

    return grad


rospy.init_node('tb3_controller', anonymous=True)
rospy.Subscriber("planned_path", String, store_path)
rospy.Subscriber('/odom', Odometry, get_pos)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


Q = np.array([  [100,  0,   0],
                [  0,100,   0],
                [  0,  0, 5]])

R = np.array([  [0.01,  0],
                [  0,0.05]])

A = np.array([  [1.0,  0,   0],
                [  0,1.0,   0],
                [  0,  0, 1.0]])

prev_time = time.time()
count = 1
while not rospy.is_shutdown():
    if len(path) > 0:
        target = path[count]
        print("Target : ",target)
        slope = get_gradient(path[count-1][0], target[0], path[count-1][1], target[1])
        desired_x = np.array([target[0], target[1], slope])
        d_to_g = dist(target[0], target[1], cur_x, cur_y)
        while d_to_g > 0.05:
            twist = Twist()

            state = np.array([cur_x, cur_y, cur_yaw])

            cur_time = time.time()
            del_t = cur_time - prev_time

            B = getB(cur_yaw, del_t)
            u = lqr(state, desired_x, Q, R, A, B, del_t)

            prev_time = cur_time

            x_vel = u[0]
            z_ang = u[1]

            if x_vel > BURGER_MAX_LIN_VEL:
                x_vel = BURGER_MAX_LIN_VEL
            if x_vel < -1*BURGER_MAX_LIN_VEL:
                x_vel = -1*BURGER_MAX_LIN_VEL

            if z_ang > BURGER_MAX_ANG_VEL:
                z_ang = BURGER_MAX_ANG_VEL
            if z_ang < -1*BURGER_MAX_ANG_VEL:
                z_ang = -1*BURGER_MAX_ANG_VEL

            #print(x_vel, z_ang)

            twist.linear.x = x_vel
            twist.angular.z = z_ang

            pub.publish(twist)
            d_to_g = dist(target[0], target[1], cur_x, cur_y)

        count+=1
        esum_x = 0
        esum_z = 0

