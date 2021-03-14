#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import sys
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from enum import Enum, IntEnum
from sensor_msgs.msg import LaserScan
from PySide2.QtGui import QTransform
from PySide2.QtCore import QPointF
from math import *

"""
Run the camera view
rosrun image_view image_view image:=/image_rectangle
roslaunch integrated_robotics_project turtlebot3_search_and_rescue.launch
rosrun find_object_2d find_object_2d image:=/camera/rgb/image_raw
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=frontier_exploration
roslaunch explore_lite explore.launch
rosrun image_view image_view image:=/image_marked
"""


class Distance_estimation():
    def __init__(self):
        rospy.init_node("distance_estimation", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/image_marked', Image, queue_size=1)
        self.arm_command_pub = rospy.Publisher('/rightControlerTrigger', Float64, queue_size=1)

        self.image_message = Image()
        self.image = np.zeros((384, 384, 1), np.uint8)
        self.count = 0

        self.current_time = rospy.get_rostime()
        self.passed_time = rospy.get_rostime()

        # variable updated in topic
        self.actual_image = Image()
        self.actual_odom = Odometry()
        self.robot_yaw = 0
        # image identification
        self.image_person = 11
        self.image_toxic = 12
        self.image_warning = 13
        #self.image_injured = 14
        self.image_fire = 15
        self.image_no_smoke = 16
        self.image_radioactive = 17
        self.image_triangle = 18
        self.image_dead = 19
        self.injured = 20

        self.object_distance = 0
        self.object_angle = 0
        self.listMeanMarker = []

        # people state
        self.color_detected = 0
        self.person_state = 0

        self.mean_counter = 0
        self.previous_ref_marker = 0
    
    def image_callback(self, ros_image):
        self.actual_image = ros_image
        self.find_visual_distance()

    def find_visual_distance(self):
        focal_length_mm = 3.04
        sensor_width_mm = 3.68
        #image_width_px = 1920
        image_width_px = 640
        focal_length_px = (focal_length_mm / sensor_width_mm) * image_width_px
        object_size = 0.135
        #camera_angle_conv = 0.0334
        camera_angle_conv = 0.0971
        camera_min_angle = -31.1
        camera_max_angle = 31.1

        cv_image = self.bridge.imgmsg_to_cv2(self.actual_image, "bgr8")

        state, xg, yg, width, height = self.detect_color_state(cv_image)
        if state == 1:

            distance = (focal_length_px * object_size) / height
            self.object_distance = distance
            angle_deg = int(camera_min_angle + camera_angle_conv * xg)
            self.object_angle = angle_deg
            # Draw the rectangle around the object
            font = cv2.FONT_HERSHEY_SIMPLEX
            # square position parameters
            xtopLeft = xg
            ytopLeft = yg

            xbotRight = xg + width
            ybotRight = yg + height

            cv2.rectangle(cv_image, (int(xtopLeft), int(ytopLeft)), (int(xbotRight), int(ybotRight)), (0, 255, 0), 2)
            #cv2.putText(cv_image, "("+str(round(distance, 2)) + "m," + str(round(angle_deg, 2)) + "deg" + ")", (int(xtopLeft), int(ytopLeft)), font, 1, (255, 0, 255), 2)
            cv2.putText(cv_image, "("+str(round(distance, 2)) + "m,", (int(xtopLeft), int(ytopLeft)), font, 1, (255, 0, 255), 2)
            image_message = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(image_message)
            if distance < 0.45:
                self.arm_command_pub.publish(1)
            else:
                self.arm_command_pub.publish(0)

        else:
            self.image_pub.publish(self.actual_image)
        
        

    """
    this function takes the last image capture and look for green or red
    if red detected = injured, green = alive
    """
    def detect_color_state(self, cv_image):
        width = 0
        height = 0
        xg = 0 
        yg = 0
        # We use cv bridge to convert the image in opencv format
        cv_people_image = self.bridge.imgmsg_to_cv2(self.actual_image, "bgr8")
        hsv = cv2.cvtColor(cv_people_image, cv2.COLOR_BGR2HSV)
        #redLower = (0, 0, 30)
        #redUpper = (80, 80, 255)
        #redmask = cv2.inRange(hsv, redLower, redUpper)
        # define a mask using the lower and upper bound of the green color
        """
        greenLower = (40, 40,40)
        greenUpper = (70, 255,255)
        """
        greenLower = (30, 80, 100)
        greenUpper = (60, 255, 255)
        greenmask = cv2.inRange(hsv, greenLower, greenUpper)
        
        _, contours, _ = cv2.findContours(greenmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            green_area = max(contours, key=cv2.contourArea)
            (xg, yg, width, height) = cv2.boundingRect(green_area)
            print("widght and height : ("+ str(width) +", "+str(height))

        # Check a minimal size of mask to avoid false detections
        if(width > 10 and height > 10):
            # color has been detected
            state = 1
        else:
            # no color detected
            state = 0
        return state, xg, yg, width, height



def main(args):
    Distance_estimation()
    try:
        rospy.spin()
    except rospy.KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
