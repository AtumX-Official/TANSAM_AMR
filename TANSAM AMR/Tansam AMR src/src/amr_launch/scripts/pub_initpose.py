#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

class initial_pose:
    def __init__(self):
        self.initial_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        initpose_msg = PoseWithCovarianceStamped()
        self.bool_sub = rospy.Subscriber('/reset_bool', Bool, self.bool_callback)

    def bool_callback(self, bool_msg):
        if bool_msg.data:
            initpose_msg = PoseWithCovarianceStamped()
            initpose_msg.header.frame_id = "map"
            initpose_msg.pose.pose.position.x = -0.26932209730148315
            initpose_msg.pose.pose.position.y = -0.11465907096862793
            initpose_msg.pose.pose.orientation.w = -0.4226834
            initpose_msg.pose.pose.orientation.z = 0.9038795
            self.initial_pub.publish(initpose_msg)
            print("/*/*/*/*/*/*/*/*/  INITIAL POSE INITIATED  /*/*/*/*/*/*/*/*/*/*/")
        
if __name__ == '__main__':
    rospy.init_node('pub_initpose_node', anonymous=True)
    initial_pose()
    rospy.spin()

