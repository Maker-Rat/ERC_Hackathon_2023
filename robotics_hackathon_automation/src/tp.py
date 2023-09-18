#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.79, Y + 0.66).
#you need to name this node "controller"

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import geometry_msgs
from math import sqrt, atan
import json
import ast

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

path = []
cur_x = None
cur_y = None
cur_z = None
prev_x = None
prev_z = None

esum_x = 0
esum_z = 0

Kp_x = 0.1
Ki_x = 0.00000

Kp_z = 0.1
Ki_z = 0.000000000000000

cur_yaw = None

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

    orient_x = msg.pose.pose.orientation.x
    orient_y = msg.pose.pose.orientation.y
    orient_z = msg.pose.pose.orientation.z
    orient_w = msg.pose.pose.orientation.w
    quaternion = (orient_x,
                  orient_y,
                  orient_z,
                  orient_w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    cur_yaw = euler[2]

    #print(cur_yaw)


def PID(target):
    global esum_x, esum_z

    slope = atan((target[1] - cur_y)/(target[0] - cur_x))
    # print("Slope : ", slope, "Yaw : ", cur_yaw)
    error_z = slope - cur_yaw
    esum_z += error_z
    z_ang = Kp_z*(error_z) + Ki_z*(esum_z)

    error_x = dist(target[0], target[1], cur_x, cur_y)
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
        while dist(target[0], target[1], cur_x, cur_y) > 0.05:
            x_vel, z_ang = PID(target)
            twist = Twist()
            twist.linear.x = x_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = z_ang
            pub.publish(twist)

        count+=1
        esum_x = 0
        esum_z = 0

