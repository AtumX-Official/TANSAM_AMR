#!/usr/bin/env python2.7
import numpy as np

from utils import aruco_display

import time
import sys
import math

import cv2
from cv_bridge import CvBridge

import rospy
import tf_conversions
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped

class pose_estimator_class():

    def __init__(self):
        
        # Initializing the node and setting up the Publishers and Subscibers
        rospy.init_node("marker_pose", anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.br = rospy.Publisher("/marker_tf",TransformStamped,queue_size=10)

        # Setting up the message headers
        self.marker_tf = TransformStamped()
        self.marker_tf.header.frame_id = "camera_color_frame"
        
        # Initializing the bridge object
        self.bridge = CvBridge()

        # Getting the marker dictionary and parameters for detecting the markers
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

        # The intrinsic matrix of the camera
        self.matrix_coefficients = np.array([[904.62939453,    0.0,       655.59423828],
                                            [ 0.0,         904.71313477,  361.97033691],
                                            [ 0.0,             0.0,              1.0]])
       
        # The distortion matrix of the camera, distortion model: plumb bob
        self.distortion_coefficients = np.array([0.16045181453227997, -0.5054474472999573, -0.0016045733354985714, 0.0005038697272539139, 0.47396108508110046])

        rospy.spin()

        pass

    def callback(self, rosimg):
        
        cv_img = self.bridge.imgmsg_to_cv2(rosimg,'bgr8')
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        
        # Checking for markers
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray,self.arucoDict,parameters=self.arucoParams)

        if len(corners) > 0:
            for i in range(0, len(ids)):
                if (not ((0 in ids) and (1 in ids))):
                    break
                if ids[i] == 0 or ids[i] == 1:

                    # Estimating the poses
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.matrix_coefficients,
                                                                           self.distortion_coefficients)

                    cv2.aruco.drawDetectedMarkers(cv_img, corners)

                    cv2.drawFrameAxes(cv_img, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)

                    self.marker_tf.header.stamp = rospy.Time.now()

                    # Converting the rotation vector to rotation matrix and generating the corresponding euler angles
                    rmat, _ = cv2.Rodrigues(rvec)
                    euler_angles = self.rotationMatrixToEulerAngles(rmat)

                    q = tf_conversions.transformations.quaternion_from_euler(euler_angles[0],euler_angles[1],euler_angles[2])

                    if ids[i] == 0:
                        self.marker_tf.child_frame_id = "marker/zero"
                    else:
                        self.marker_tf.child_frame_id = "marker/one"

                    self.marker_tf.transform.rotation.x = q[0]
                    self.marker_tf.transform.rotation.y = q[1]
                    self.marker_tf.transform.rotation.z = q[2]
                    self.marker_tf.transform.rotation.w = q[3]

                    self.marker_tf.transform.translation.x = tvec[0][0][0]
                    self.marker_tf.transform.translation.y = tvec[0][0][1]
                    self.marker_tf.transform.translation.z = tvec[0][0][2]

                    # self.br.sendTransform(self.marker_tf)
                    self.br.publish(self.marker_tf)

        cv2.imshow("Image", cv_img)
        cv2.waitKey(1)

        pass

    def rotationMatrixToEulerAngles(self, R) :
    
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
    
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
    
        return np.array([x, y, z])

if __name__ == '__main__':
    obj = pose_estimator_class()