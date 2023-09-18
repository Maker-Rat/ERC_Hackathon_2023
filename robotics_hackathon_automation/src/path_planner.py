#!/usr/bin/env python3
#Following is the import line for importing the obstacle detection methods i.e. isValidPoint()
#you need to name this node as "path_planner"
import obstacle_detection as obsdet

from random import random, uniform 
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon

from math import atan, cos, sin

import rospy
from std_msgs.msg import String

targets = [(-3.61,-2.2), (-2.28,1.86), (0.57,0.33), (1.58,-2.26), (5.18,-2.19)]

def dist(x1, y1, x2, y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def min_dist(p, p_lst):
    min_distance = 100
    for pnt in  p_lst:
        d = dist(p[0], p[1], pnt[0], pnt[1])
        if d < min_distance:
            min_distance = d
            nearest = pnt

    return min_distance, nearest


start_point = (-5.06,-3.12)


class RRT():
    def __init__(self):
        self.parent_dict = {}
        self.point_list = []
        self.point_list.append(start_point)

    def plan_path_RRT(self):
        count = 0
        while count < 5:
            found = False
            while not found:
                p = (random(-6, 6), random(-3, 3))
                dist, nearest = min_dist(p, self.point_list)
                if (dist <= 0.2) and (not obsdet.isPointInObstacles(p[0], p[1])) and (obsdet.isValidPoint(nearest[0], nearest[1], p[0], p[1])):
                    self.parent_dict


class A_star():
    def __init__(self):
        self.parent = {}
        self.adjacency_dict = {start_point:[]}
        self.point_list = [start_point]
        self.closed_list = []
        self.open_list = []
        self.g = {}
        self.count = 0

    def h(self, p, target):
        return dist(p[0], p[1], target[0], target[1])
    
    def nearest_n(self, n, p):
        d = {}
        for pnt in self.point_list:
            d[dist(pnt[0], pnt[1], p[0], p[1])] = pnt

        distances = list(d.keys())
        distances.sort()
        lst = []
        c = 0
        for dst in distances:
            if obsdet.isValidPoint(p[0], p[1], d[dst][0], d[dst][1]) and dst > 0:
                lst.append(d[dst])
                c+=1
            if c == n:
                break

        return lst

    def PRM(self):
        for i in range(500):
            x = uniform(-6, 6)
            y = uniform(-3.3, 3.3)
            while obsdet.isPointInObstacles(round(x, 2), round(y, 2)):
                x = uniform(-6, 6)
                y = uniform(-3.3, 3.3)

            neighbors = self.nearest_n(4, (x, y))
            p = (round(x, 2), round(y, 2))
            self.point_list.append(p)
            self.adjacency_dict[p] = neighbors
            for neighbor in neighbors:
                self.adjacency_dict[neighbor].append(p)

        for pnt in targets:
            x = pnt[0]
            y = pnt[1]
            neighbors = self.nearest_n(4, (x, y))
            p = (x, y)
            self.point_list.append(p)
            self.adjacency_dict[p] = neighbors
            for neighbor in neighbors:
                self.adjacency_dict[neighbor].append(p)

    def brute(self):
        num = 0
        grid_spacing = 0.25
        for x in np.round(np.arange(-6, 6, grid_spacing), 3):
            for y in np.round(np.arange(-3.5, 3.5, grid_spacing), 3):
                if not obsdet.isPointInObstacles(x, y):
                    pnt = (round(x, 2), round(y, 2))
                    self.point_list.append(pnt)
                    self.adjacency_dict[pnt] = []

                    neighbors = [(x, y+grid_spacing), (x, y-grid_spacing), (x+grid_spacing, y), (x-grid_spacing, y), (x+grid_spacing, y+grid_spacing), (x+grid_spacing, y-grid_spacing), (x+grid_spacing, y+grid_spacing), (x-grid_spacing, y-grid_spacing)]

                    for p in neighbors:
                        p = (round(p[0], 2), round(p[1], 2))
                        if obsdet.isValidPoint(pnt[0], pnt[1], p[0], p[1]) and not obsdet.isPointInObstacles(p[0], p[1]) and abs(p[0]) <= 6 and abs(p[1]) <=3.5:
                            self.adjacency_dict[pnt].append(p)
                            num+=1

        for pnt in targets:
            x = pnt[0]
            y = pnt[1]
            neighbors = self.nearest_n(4, (x, y))
            p = (x, y)
            self.point_list.append(p)
            self.adjacency_dict[p] = neighbors
            for neighbor in neighbors:
                self.adjacency_dict[neighbor].append(p)

        x = start_point[0]
        y = start_point[1]
        neighbors = self.nearest_n(4, (x, y))
        p = (x, y)
        self.point_list.append(p)
        self.adjacency_dict[p] = neighbors
        for neighbor in neighbors:
            self.adjacency_dict[neighbor].append(start_point)
        
        return

    def plan_path(self):
        path = []
        for target in targets:
            temp_path = []
            if self.count == 0:
                start = start_point
                self.closed_list = []
                self.open_list = [start]
                self.parent = {}
                self.g = {}
                self.g[start] = 0
                self.parent[start] = start

            else:
                start = targets[self.count-1]
                self.closed_list = []
                self.open_list = [start]
                self.closed_list = []
                self.open_list = [start]
                self.parent = {}
                self.g = {}
                self.g[start] = 0
                self.parent[start] = start

            while (len(self.open_list) > 0):
                n = None

                for p in self.open_list:
                    if n == None or self.g[p] + self.h(p, target) < self.g[n] + self.h(n, target):
                        n = p
                        n = (round(n[0], 2), round(n[1], 2))

                if n == None:
                    print('Path does not exist!')
                    return None
                
                if n == target:
                    while self.parent[n] != n:
                        temp_path.append(n)
                        n = self.parent[n]

                    temp_path.append(start)
                    temp_path.reverse()
                    path.extend(temp_path)
                    self.count += 1
                    break

                for m in self.adjacency_dict[n]:
                    weight = dist(m[0], m[1], n[0], n[1])
                    m = (round(m[0], 2), round(m[1], 2))
                    if m not in self.open_list and m not in self.closed_list:
                        self.open_list.append(m)
                        self.parent[m] = n
                        self.g[m] = round(self.g[n] + weight, 2)

                    else:
                        if self.g[m] > self.g[n] + weight:
                            self.g[m] = self.g[n] + weight
                            self.parent[m] = n

                            if m in self.closed_list:
                                self.closed_list.remove(m)
                                self.open_list.append(m)

                # remove n from the open_list, and add it to closed_list
                # because all of his neighbors were inspected
                self.open_list.remove(n)
                self.closed_list.append(n)

        return path

            
path_planner = A_star()
path_planner.brute()

path = path_planner.plan_path()
print("Path_Planned")

path_orig = list(path)

for i in range(len(path)):
    path[i] = (round(path[i][0]+1.79, 2), round(path[i][1]+0.66, 2))
    if path_orig[i] in targets:
        if path[i][0] == path[i-1][0]:
            theta = np.pi/2
        else:
            slope = (path[i][1] - path[i-1][1])/(path[i][0] - path[i-1][0])
            theta = abs(atan(slope))

        if slope >= 0:
            if path[i][1] > path[i-1][1]:
                path[i] = (path[i][0] - (0.4*cos(theta)), path[i][1] - (0.4*sin(theta)))
            else:
                path[i] = (path[i][0] + (0.4*cos(theta)), path[i][1] + (0.4*sin(theta)))
        else:
            if path[i][1] > path[i-1][1]:
                path[i] = (path[i][0] + (0.4*cos(theta)), path[i][1] - (0.4*sin(theta)))
            else:
                path[i] = (path[i][0] - (0.4*cos(theta)), path[i][1] + (0.4*sin(theta)))


xlist = []
ylist = []
for xcoord, ycoord in path_orig[::-1]:
    xlist.append(xcoord)
    ylist.append(ycoord)

class Wall:
    def __init__(self, centerX, centerY, length, width):
        self.centerX = centerX
        self.centerY = centerY
        self.width = width + 0.2
        self.length = length+0.2
        Ax = (self.centerX+(self.length/2))
        Ay = (self.centerY+(self.width/2))
        Bx = (self.centerX+(self.length/2))
        By = (self.centerY-(self.width/2))
        Cx = (self.centerX-(self.length/2))
        Cy = (self.centerY-(self.width/2))
        Dx = (self.centerX-(self.length/2))
        Dy = (self.centerY+(self.width/2))
        self.polygon = Polygon(
            [(Ax, Ay), (Bx, By), (Cx, Cy), (Dx, Dy), (Ax, Ay)])
        self.corners = [(Ax, Ay), (Bx, By), (Cx, Cy), (Dx, Dy)]


maze = [Wall(-5.191, 0.9886, 1, 0.15), Wall(-5.639, -0.8309, 0.15, 3.769200), Wall(-5.672, 1.785, 0.15, 1.597130), Wall(-4.957, 2.543, 1.597130, 0.15), Wall(-4.277, 2.007956, 0.15, 1.169920), Wall(-0.0037, 2.51, 8.729630, 0.15), Wall(-1.588, 1.8136, 0.15, 1.25), Wall(-1.588, 0.0886, 0.15, 2.5), Wall(-2.138, 1.26, 1.25, 0.15), Wall(-2.668, 0.7136, 0.15, 1.25), Wall(-3.488, 0.16, 1.75, 0.15), Wall(2.405, 0.656, 0.75, 0.15), Wall(2.705, 0.956, 0.15, 0.75), Wall(3.2522, 1.2566, 1.25, 0.15), Wall(3.80526, 0.2066, 0.15, 2.25), Wall(3.3802, -0.844, 1, 0.15), Wall(2.955, -0.5433, 0.15, 0.75), Wall(2.7802, -0.2433, 0.5, 0.15), Wall(2.605, -0.5433, 0.15, 0.75), Wall(4.301, 2.189, 0.15, 0.810003), Wall(4.975, 2.5196, 1.50, 0.15), Wall(5.711,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              1.998, 0.15, 1.192330), Wall(5.306, 1.463, 0.919672, 0.15), Wall(5.698, 0.301, 0.15, 2.276490), Wall(5.185, -0.885, 1.119670, 0.15), Wall(4.7, -1.296, 0.15, 0.982963), Wall(5.67, -1.7033, 0.15, 1.75), Wall(5.154, -2.521, 1.185380, 0.15), Wall(0.673, -2.534, 7.883080, 0.15), Wall(1.906, -1.93, 0.15, 1.206910), Wall(0.877, -1.7, 0.15, 1.719980), Wall(0.2502, -0.917, 1.50, 0.15), Wall(-0.433, -1.389, 0.15, 1.072), Wall(-0.4292, -0.4799, 0.15, 0.927565), Wall(0.9177, 0.2156, 0.15, 2.416050), Wall(0.23527, 1.3486, 1.5, 0.15), Wall(-0.439, 1.048, 0.15, 0.75), Wall(-3.2627, -1.72, 0.15, 1.75), Wall(-3.883, -0.9203, 1.414750, 0.15), Wall(-3.9377, -2.52, 1.5, 0.15), Wall(-4.615, -2.157, 0.15, 0.870384), Wall(2.105, 1.58, 0.15, 2.15893)]
# To try checking out how the maze looks, uncomment the following
display_maze = [Wall(-5.191, 0.9886, 1, 0.15), Wall(-5.639, -0.8309, 0.15, 3.769200), Wall(-5.672, 1.785, 0.15, 1.597130), Wall(-4.957, 2.543, 1.597130, 0.15), Wall(-4.277, 2.007956, 0.15, 1.169920), Wall(-0.0037, 2.51, 8.729630, 0.15), Wall(-1.588, 1.8136, 0.15, 1.25), Wall(-1.588, 0.0886, 0.15, 2.5), Wall(-2.138, 1.26, 1.25, 0.15), Wall(-2.668, 0.7136, 0.15, 1.25), Wall(-3.488, 0.16, 1.75, 0.15), Wall(2.405, 0.656, 0.75, 0.15), Wall(2.705, 0.956, 0.15, 0.75), Wall(3.2522, 1.2566, 1.25, 0.15), Wall(3.80526, 0.2066, 0.15, 2.25), Wall(3.3802, -0.844, 1, 0.15), Wall(2.955, -0.5433, 0.15, 0.75), Wall(2.7802, -0.2433, 0.5, 0.15), Wall(2.605, -0.5433, 0.15, 0.75), Wall(4.301, 2.189, 0.15, 0.810003), Wall(4.975, 2.5196, 1.50, 0.15), Wall(5.711, 1.998, 0.15, 1.192330), Wall(5.306, 1.463, 0.919672, 0.15), Wall(5.698, 0.301, 0.15, 2.276490), Wall(5.185, -0.885, 1.119670, 0.15), Wall(4.7, -1.296, 0.15, 0.982963), Wall(5.67, -1.7033, 0.15, 1.75), Wall(5.154, -2.521, 1.185380, 0.15), Wall(0.673, -2.534, 7.883080, 0.15), Wall(1.906, -1.93, 0.15, 1.206910), Wall(0.877, -1.7, 0.15, 1.719980), Wall(0.2502, -0.917, 1.50, 0.15), Wall(-0.433, -1.389, 0.15, 1.072), Wall(-0.4292, -0.4799, 0.15, 0.927565), Wall(0.9177, 0.2156, 0.15, 2.416050), Wall(0.23527, 1.3486, 1.5, 0.15), Wall(-0.439, 1.048, 0.15, 0.75), Wall(-3.2627, -1.72, 0.15, 1.75), Wall(-3.883, -0.9203, 1.414750, 0.15), Wall(-3.9377, -2.52, 1.5, 0.15), Wall(-4.615, -2.157, 0.15, 0.870384), Wall(2.105, 1.58, 0.15, 2.15893)]
for wall in display_maze:
    x, y = wall.polygon.exterior.xy
    plt.plot(x, y)

plt.plot(-5.06,-3.12,'ro')#start
plt.plot(5.18,-2.19,'ro')#5
plt.plot(-2.28,1.86,'ro')#2
plt.plot(0.57,0.33,'ro') #3
plt.plot(1.58,-2.26,'ro')#4
plt.plot(-3.61,-2.2,'ro')#1

plt.plot(xlist, ylist)

plt.show()

path_str = str(path)

pub = rospy.Publisher('planned_path', String, queue_size=10, latch=True)
rospy.init_node('path_planner', anonymous=True)
rate = rospy.Rate(10)
#rospy.loginfo(path_str)
pub.publish(path_str)
rospy.spin()