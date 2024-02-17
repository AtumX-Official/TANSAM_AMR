#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import Imu

class Imu_pub_sub:
    
    def __init__(self):
        
        rospy.init_node('IMU_combiner',anonymous=True)

        self.pub = rospy.Publisher("/imu/data_raw",Imu,queue_size=10)

        # self.accel_msg = []
        # self.gyro_msg = []
        self.accel_msg_x = float
        self.accel_msg_y = float
        self.accel_msg_z = float

        self.gyro_msg_x = float
        self.gyro_msg_y = float
        self.gyro_msg_z = float

        self.Imu_msg = Imu()

        # self.Imu_msg.header.frame_id = "camera_depth_frame"
        self.Imu_msg.header.frame_id = "imu_link"
        self.Imu_msg.orientation_covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.Imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        rospy.Subscriber("/camera/gyro/sample",Imu,self.gyro_callback)
        rospy.Subscriber("/camera/accel/sample",Imu,self.accel_callback)

        rate = rospy.Rate(100)
        print("//////////////////////////*****IMU STARTED********///////////////////////////////")
        while(not rospy.is_shutdown()):            
            self.combine()
            self.pub.publish(self.Imu_msg)
            rate.sleep()
            
    
    def gyro_callback(self,msg):
        # self.gyro_msg = msg.angular_velocity
        
        self.gyro_msg_x = msg.angular_velocity.x
        self.gyro_msg_y = -msg.angular_velocity.y
        self.gyro_msg_z = -msg.angular_velocity.z
        
        pass
    
    def accel_callback(self,msg):
        # self.accel_msg = msg.linear_acceleration
        
        self.accel_msg_x = msg.linear_acceleration.x
        self.accel_msg_y = -msg.linear_acceleration.y
        self.accel_msg_z = -msg.linear_acceleration.z
        pass

    def combine(self):
        self.Imu_msg.header.stamp = rospy.Time().now()
        # self.Imu_msg.linear_acceleration = self.accel_msg
        # self.Imu_msg.angular_velocity = self.gyro_msg
        
        
        self.Imu_msg.angular_velocity.x = self.gyro_msg_z
        self.Imu_msg.angular_velocity.y = self.gyro_msg_x
        self.Imu_msg.angular_velocity.z = self.gyro_msg_y

        self.Imu_msg.linear_acceleration.x = self.accel_msg_z
        self.Imu_msg.linear_acceleration.y = self.accel_msg_x
        self.Imu_msg.linear_acceleration.z = self.accel_msg_y


if __name__ == '__main__':
    obj = Imu_pub_sub()
