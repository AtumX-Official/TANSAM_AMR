#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('pub_initpose_node', anonymous=True)
pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
initpose_msg = PoseWithCovarianceStamped()
initpose_msg.header.frame_id = "map"
initpose_msg.pose.pose.position.x = -0.20383691787719727
initpose_msg.pose.pose.position.y = -0.04881858825683594
initpose_msg.pose.pose.orientation.w = 0.9998511063418842
rate = rospy.Rate(1) #10hz
while not rospy.is_shutdown():
    pub.publish(initpose_msg)
    rate.sleep()

