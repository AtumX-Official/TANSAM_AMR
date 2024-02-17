#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String

class VelocityController:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.log_pub = rospy.Publisher('/log', String, queue_size=10)
        self.log_pub.publish("AMR Status :   ON")
        self.vel_sub = rospy.Subscriber('/cmd_vel_mb', Twist, self.vel_callback)
        self.bool_sub = rospy.Subscriber('/bool_value', Bool, self.bool_callback)
        self.publish_vel = True

    def vel_callback(self, msg):
        if self.publish_vel:
            self.vel_pub.publish(msg)

    def bool_callback(self, bool_msg):
        if bool_msg.data and not self.publish_vel:  #1 * 0
            self.publish_vel = True
            self.log_pub.publish("AMR Status :   ON")
        elif bool_msg.data and self.publish_vel:   #1 * 1
            self.publish_vel = False
            stop_msg = Twist()
            stop_msg.linear.x = 0
            stop_msg.angular.z = 0
            self.vel_pub.publish(stop_msg)
            self.log_pub.publish("AMR Status :   OFF")

if __name__ == '__main__':
    rospy.init_node('velocity_controller')
    VelocityController()
    rospy.spin()
