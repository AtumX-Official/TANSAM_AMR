#!/usr/bin/env python3

import rospy # Python library for ROS
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int16, Bool





class simple_class:

  def __init__(self):
    self.button_1 = rospy.Subscriber("Goal_1",Bool, self.goal_1)
    self.button_2 = rospy.Subscriber("Goal_2",Bool, self.goal_2)
    self.button_3 = rospy.Subscriber("Goal_3",Bool, self.goal_3)
    self.button_4 = rospy.Subscriber("Goal_4",Bool, self.goal_4)
    self.goal_point =0
    self.user_goal =0
    self.last_goal = MoveBaseGoal()
    self.current_goal = MoveBaseGoal()
    self.current_goal.target_pose.header.frame_id = "map"
    self.last_goal.target_pose.header.frame_id = "map"
    self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    self.client.wait_for_server()

  def goal_1(self,data):
    print("inside_1")
    self.user_goal = data.data

    if self.user_goal == True:
        self.goal_point = 1
    self.action()

  def goal_2(self,data):
    print("inside_2")
    self.user_goal = data.data

    if self.user_goal == True:
        self.goal_point = 2
    self.action()


  def goal_3(self,data):
    print("inside_3")
    self.user_goal = data.data

    if self.user_goal == True:
        self.goal_point = 3
    self.action()

  def goal_4(self,data):
    print("inside_4")
    self.user_goal = data.data

    if self.user_goal == True:
        self.goal_point = 4
    self.action()


  def action(self):
    self.current_goal.target_pose.header.stamp = rospy.Time.now()
    self.last_goal.target_pose.header.stamp = rospy.Time.now()
    
    if self.goal_point == 1:
        self.current_goal.target_pose.pose.position.x = -6.726800918579102   # 3D printer 
        self.current_goal.target_pose.pose.position.y = -2.0821495056152344
        self.current_goal.target_pose.pose.orientation.w = 0.2074292419935039
        self.current_goal.target_pose.pose.orientation.z = 0.7067841732611849
        self.client.send_goal(self.current_goal)
    
    if self.goal_point == 2:
        self.current_goal.target_pose.pose.position.x = -1.7212328910827637   #4.0
        self.current_goal.target_pose.pose.position.y = -9.866405487060547
        self.current_goal.target_pose.pose.orientation.w = -0.5986834
        self.current_goal.target_pose.pose.orientation.z = -0.9238795
        self.client.send_goal(self.current_goal)

    if self.goal_point == 3:
        self.current_goal.target_pose.pose.position.x = -8.105987548828125    #VR
        self.current_goal.target_pose.pose.position.y = -15.927983283996582
        self.current_goal.target_pose.pose.orientation.w = 0.9935433654811755
        self.current_goal.target_pose.pose.orientation.z = 0.1134529898607318
        self.client.send_goal(self.current_goal)
    
    if self.goal_point == 4:
        self.current_goal.target_pose.pose.position.x = 0.3608055114746094  #home
        self.current_goal.target_pose.pose.position.y = 0.1529729664325714
        self.current_goal.target_pose.pose.orientation.w = -0.5986834
        self.current_goal.target_pose.pose.orientation.z = 0.9238795
        self.client.send_goal(self.current_goal)

def main(args):
  rospy.init_node('simple_class', anonymous=True)
  obc = simple_class()
  
  
  try:
    rospy.spin()
    
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
