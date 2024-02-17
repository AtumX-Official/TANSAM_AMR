#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int16, Bool

global goal_data

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -0.232544153929
    goal.target_pose.pose.position.y = 0.275638401508


   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 0.53448781325
    goal.target_pose.pose.orientation.z = 0.84517618133


    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def goal_one(msg):
    global goal_data
    goal_point = msg.data
    rospy.loginfo("inside_sub1")
    if goal_point == True:
        goal_data = 1
    else:
        goal_data = 0

def goal_two(msg):
    global goal_data
    goal_point = msg.data
    rospy.loginfo("inside_sub2")
    if goal_point == True:
        goal_data = 2
    else:
        goal_data = 0

def goal_three(msg):
    global goal_data
    rospy.loginfo("inside_sub3")
    goal_point = msg.data
    if goal_point == True:
        goal_data = 3
    else:
        goal_data = 0


def action(data):
    print("inside_action")
    if data == 1:
        ## point_1
        goal.target_pose.pose.position.x = -0.232544153929
        goal.target_pose.pose.position.y = 0.275638401508
        goal.target_pose.pose.orientation.w = 0.53448781325
        goal.target_pose.pose.orientation.z = 0.84517618133
        client.send_goal(goal)

        print("go_to_goal_1")

    if data == 2:
        print("go_to_goal_2")
    if data == 3:
        print("go_to_goal_3")


def listener():


    rospy.Subscriber("Goal_1",Bool, goal_one)
    rospy.Subscriber("Goal_2",Bool, goal_two)
    rospy.Subscriber("Goal_3",Bool, goal_three)
    rospy.loginfo(goal_data)
    action(goal_data)
    rospy.spin()


if __name__ == '__main__':
    goal_data = 0
    rospy.init_node('movebase_client_py')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    listener()


