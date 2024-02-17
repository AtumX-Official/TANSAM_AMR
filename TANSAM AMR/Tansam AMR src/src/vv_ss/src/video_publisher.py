#!/usr/bin/env python2
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
  
def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  video_1 = rospy.Publisher('video_1', Image, queue_size=10)
  video_2 = rospy.Publisher('video_2', Image, queue_size=10)
     
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_pub_py', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     
  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  cam_1 = cv2.VideoCapture("/dev/video0")
  cam_2 = cv2.VideoCapture("/dev/video2")
     
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret_1, frame_1 = cam_1.read()
      ret_2, frame_2 = cam_2.read()
         
      if ret_1 == True:
        # Print debugging information to the terminal
        rospy.loginfo('publishing video frame')
             
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        video_1.publish(br.cv2_to_imgmsg(frame_1))
        video_2.publish(br.cv2_to_imgmsg(frame_2))
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass