#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('initial_pose_publisher')
    rospy.loginfo("gg")
    # Create a publisher for the 'initialpose' topic with type 'PoseWithCovarianceStamped'
    initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

    # Create a new 'PoseWithCovarianceStamped' message
    initialpose_msg = PoseWithCovarianceStamped()

    # Set the position of the initial pose message
    initialpose_msg.pose.pose.position.x = 0.0
    initialpose_msg.pose.pose.position.y = 0.0
    initialpose_msg.pose.pose.position.z = 0.0

    # Set the orientation of the initial pose message as a quaternion
    initialpose_msg.pose.pose.orientation.x = 0.0
    initialpose_msg.pose.pose.orientation.y = 0.0
    initialpose_msg.pose.pose.orientation.z = 0.0
    initialpose_msg.pose.pose.orientation.w = 1.0

    # Set the covariance matrix of the initial pose message
    initialpose_msg.pose.covariance = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ]

    # Set the header of the initial pose message
    initialpose_msg.header.stamp = rospy.Time.now()
    initialpose_msg.header.frame_id = "map"

    # Publish the initial pose message
    initialpose_pub.publish(initialpose_msg)
    rospy.loginfo("published")
    # Sleep for 1 second to allow the message to be published
    rospy.sleep(1.0)
    rospy.spin()
    # Print the initial pose message
    # print(f"Initial pose set: x={initialpose_msg.pose.pose.position.x:.2f}, y={initialpose_msg.pose.pose.position.y:.2f}, yaw={0:.2f}")
