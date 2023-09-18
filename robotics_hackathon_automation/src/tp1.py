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

BURGER_MAX_LIN_VEL = 0.2
BURGER_MAX_ANG_VEL = 2.5

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

def PID_z(error_z):
    global esum_z
    esum_z += error_z
    z_ang = Kp_z*(error_z) + Ki_z*(esum_z)

    if z_ang > BURGER_MAX_ANG_VEL:
        z_ang = BURGER_MAX_ANG_VEL
    if z_ang < -1*BURGER_MAX_ANG_VEL:
        z_ang = -1*BURGER_MAX_ANG_VEL

    return z_ang


def PID_x(error_x):
    global esum_x
    esum_x += error_x
    x_vel = Kp_x*(error_x) + Ki_x*(esum_x)

    if x_vel > BURGER_MAX_LIN_VEL:
        x_vel = BURGER_MAX_LIN_VEL
    if x_vel < -1*BURGER_MAX_LIN_VEL:
        x_vel = -1*BURGER_MAX_LIN_VEL

    return x_vel


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


def PID(error_x, error_z):
    global esum_x, esum_z
    # slope = (target[1] - cur_y)/(target[0] - cur_x)
    
    esum_z += error_z
    z_ang = Kp_z*(error_z) + Ki_z*(esum_z)

    esum_x += error_x
    x_vel = Kp_x*(error_x) + Ki_x*(esum_x)

    if x_vel > BURGER_MAX_LIN_VEL:
        x_vel = BURGER_MAX_LIN_VEL
    if x_vel < -1*BURGER_MAX_LIN_VEL:
        x_vel = -1*BURGER_MAX_LIN_VEL

    if z_ang > BURGER_MAX_ANG_VEL:
        z_ang = BURGER_MAX_ANG_VEL
    if z_ang < -1*BURGER_MAX_ANG_VEL:
        z_ang = -1*BURGER_MAX_ANG_VEL

    return x_vel, z_ang


rospy.init_node('tb3_controller', anonymous=True)
rospy.Subscriber("planned_path", String, store_path)
rospy.Subscriber('/odom', Odometry, get_pos)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


count = 0
while not rospy.is_shutdown():
    if len(path) > 0:
        target = path[count]
        print("Target : ",target)
        d_to_g = dist(target[0], target[1], cur_x, cur_y)
        while d_to_g > 0.08:
            twist = Twist()
            #slope = atan2(target[1] - cur_y, target[0] - cur_x)
            slope = get_gradient(target[0], cur_x, target[1], cur_y)
            #print("Slope : ", slope, "Yaw : ", cur_yaw)
            #print(d_to_g)
            #print("X : ", cur_x, "Y: ", cur_y)
            #print(target[1] - cur_y, target[0] - cur_x)

            # if slope > pi/4 or slope < -pi/4:
            #     if target[1] < 0 and cur_y < target[1]:
            #         slope = -2*pi + slope
            #     elif target[1] > 0 and cur_y > target[1]:
            #         slope = 2*pi + slope

            # if prev_yaw > (pi-0.1) and cur_yaw <= 0:
            #     cur_yaw = 2*pi + cur_yaw
            # if prev_yaw < (-pi + 0.1) and cur_yaw > 0:
            #     cur_yaw = -2*pi + cur_yaw

            prev_yaw = cur_yaw

            error_z = -slope
            error_x = d_to_g

            print(error_z, cur_yaw, error_x)

            if abs(error_z) > 0.1:
                esum_z += error_z
                z_ang = Kp_z*(error_z) + Ki_z*(esum_z)

                if z_ang > BURGER_MAX_ANG_VEL:
                    z_ang = BURGER_MAX_ANG_VEL
                if z_ang < -1*BURGER_MAX_ANG_VEL:
                    z_ang = -1*BURGER_MAX_ANG_VEL

                twist.linear.x = 0.0
                twist.angular.z = z_ang
                

            else:
                #print("Yess")
                esum_x += error_x
                x_vel = Kp_x*(error_x) + Ki_x*(esum_x)

                if x_vel > BURGER_MAX_LIN_VEL:
                    x_vel = BURGER_MAX_LIN_VEL
                if x_vel < -1*BURGER_MAX_LIN_VEL:
                    x_vel = -1*BURGER_MAX_LIN_VEL

                twist.linear.x = x_vel
                twist.angular.z = 0

            #x_vel, z_ang = PID(error_x, error_z)
            pub.publish(twist)
            d_to_g = dist(target[0], target[1], cur_x, cur_y)

        count+=1
        esum_x = 0
        esum_z = 0

