#!/usr/bin/env python3
#You need to name this node "color_detector"

#!/usr/bin/env python3

import rospy
import cv2
from math import sqrt
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from tf.transformations import euler_from_quaternion

cur_x = None
cur_y = None
cur_yaw = None

def dist(x1, y1, x2, y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def get_pos(msg):
    global cur_x, cur_y, cur_yaw
    cur_x = msg.pose.pose.position.x
    cur_y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, cur_yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

class ColorDetectionNode:
    def __init__(self):
        rospy.init_node('colour_detector')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/odom', Odometry, get_pos)
        self.color_pub = rospy.Publisher('/task_status', String, queue_size=1)
        self.coordinates_list = [(-3.61,-2.2), (-2.28,1.86), (0.57,0.33), (1.58,-2.26), (5.18,-2.19)]
        self.current_coordinate_index = 0

        self.light_blue = np.array([5,150,0])
        self.dark_blue = np.array([200,255,255])

        self.light_red = np.array([0,50,50])
        self.dark_red = np.array([10,255,255])

    def classify_color(self, img):
        mask_blue = cv2.inRange(img, self.light_blue, self.dark_blue)
        mask_red = cv2.inRange(img, self.light_red, self.dark_red)
        ratio_blue = cv2.countNonZero(mask_blue)/(img.size)
        ratio_red = cv2.countNonZero(mask_red)/(img.size)
        
        percent_blue = (ratio_blue * 100)
        percent_red = (ratio_red * 100)

        # cv2.imshow("cam", mask_red)
        # cv2.resizeWindow("cam", 640, 480)

        # cv2.waitKey(1)

        if percent_blue > 6:
            return "Iron extraction ongoing"
        elif percent_red > 6:
            return "Zinc extraction ongoing"

    def image_callback(self, data):
        try:
            if self.current_coordinate_index < len(self.coordinates_list):
                target = self.coordinates_list[self.current_coordinate_index]
                cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
                hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                cv2.imshow("cam1", cv_image)
                cv2.resizeWindow("cam1", 640, 480)

                cv2.waitKey(1)

                dominant_color = self.classify_color(hsv_img)

                color_msg = String()
                color_msg.data = dominant_color
                self.color_pub.publish(color_msg)

                if dominant_color and dist(cur_x, cur_y, target[0]+1.79, target[1]+0.66) < 0.5:
                    print(dominant_color)
                    self.current_coordinate_index += 1

                if self.current_coordinate_index == len(self.coordinates_list):
                    completion_msg = String()
                    completion_msg.data = "Task Completed"
                    print("Task Completed")
                    self.color_pub.publish(completion_msg)

        except Exception as e:
            rospy.logerr(str(e))

try:
    color_detection_node = ColorDetectionNode()
    rospy.spin()

except rospy.ROSInterruptException:
    pass
