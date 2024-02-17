#!/usr/bin/env python2.7
import numpy as np
from utils import aruco_display
import time
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge

class marker_detect_class():

    def __init__(self):

        # Setting up the subsriber node
        rospy.init_node("marker_detector", anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

        #Initializing the bridge object
        self.bridge = CvBridge()

        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

        rospy.spin()


    def callback(self, rosimg):
        
        # Converting Image message to cv2 image
        cv_img = self.bridge.imgmsg_to_cv2(rosimg,'bgr8')
        
        # Detecting the ArUco Markers
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_img, self.arucoDict, parameters=self.arucoParams)
        detected_markers, ids = aruco_display(corners, ids, rejected, cv_img)

        # Displays the image
        # cv2.imshow("Image", detected_markers)
        # cv2.waitKey(1)
        print(ids)


if __name__ == '__main__':
    obj = marker_detect_class()