#!/usr/bin/env python2


import rospy # Python library for ROS
import sys
import numpy as np
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

class simple_class:

  def __init__(self):
    self.vid_1 = rospy.Subscriber("video_1",Image,self.video_1)
    self.vid_2 = rospy.Subscriber("video_2",Image,self.video_2)
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    self.current_frame_2 = 0
    self.current_frame_1 = 0

  def video_1(self,data):
    br = CvBridge()
    self.current_frame_1 = br.imgmsg_to_cv2(data)
    # cv2.imshow("camera", self.current_frame)
    # cv2.waitKey(1)
    
  def video_2(self,data):
    br = CvBridge()
    self.current_frame_2 = br.imgmsg_to_cv2(data)
    
    Hori = np.concatenate((self.current_frame_1, self.current_frame_2), axis=1)
 
# concatenate image Vertically
    Verti = np.concatenate((self.current_frame_1, self.current_frame_2), axis=0)
    
    cv2.imshow('HORIZONTAL', Hori)

    cv2.waitKey(1)



def main(args):
  obc = simple_class()
  rospy.init_node('simple_class', anonymous=True)
  try:
    rospy.spin()
    
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)