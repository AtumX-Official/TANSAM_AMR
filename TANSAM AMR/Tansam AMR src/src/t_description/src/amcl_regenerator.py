#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal

pose = PoseWithCovarianceStamped()
has_goal = False

def update_pose(msg):
    global pose
    # Update the pose with the current robot position
    pose.pose.position = msg.pose.position
    pose.pose.orientation = msg.pose.orientation
    pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]         
    


def update_goal(msg):
    global has_goal
    # Update the has_goal variable when a new goal is received
    has_goal = True
    rospy.loginfo("Goal given",has_goal)

def regenerate_amcl():
    # Publish the new AMCL message
    pub.publish(pose)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('amcl_regenerator')
    rospy.loginfo("Entered_Loop")
    # Create a publisher for the AMCL topic
    pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=10)

 

    # Set the rate at which AMCL will be regenerated
    rate = rospy.Rate(0.1) # 10 Hz

    # Loop through the code continuously
    while not rospy.is_shutdown():
        rospy.loginfo("AMCL_REGENERATED_LOOP")
           # Create a subscriber for the current robot position
        sub_pose = rospy.Subscriber('/robot_pose', PoseStamped, update_pose)

    # Create a subscriber for the move_base_simple/goal topic
        sub_goal = rospy.Subscriber('/move_base_simple/goal', MoveBaseActionGoal, update_goal)
        # Regenerate the AMCL only when a new goal is received

        regenerate_amcl()
        rospy.loginfo("AMCL_REGENERATED")


        # Sleep for the specified duration
        rate.sleep()
