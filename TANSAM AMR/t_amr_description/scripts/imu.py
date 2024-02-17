#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import Imu

class Imu_pub_sub:
    
    def __init__(self):
        
        rospy.init_node('IMU_combiner',anonymous=True)

        self.pub = rospy.Publisher("/Imu",Imu,queue_size=10)

        self.accel_msg = float
        self.gyro_msg = float

        self.Imu_msg = Imu()
        self.Imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.Imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        rospy.Subscriber("/camera/gyro/sample",Imu,self.gyro_callback)
        rospy.Subscriber("/camera/accel/sample",Imu,self.accel_callback)

        prev_millis = time.time()
        interval = 0.1
        
        while(not rospy.is_shutdown()):
            current_millis = time.time()
            if(current_millis - prev_millis > interval):
                self.combine()
                self.pub.publish(self.Imu_msg)
            
    
    def gyro_callback(self,msg):
        # self.gyro_msg.stamp = msg.stamp
        self.gyro_msg = msg.angular_velocity.z
        pass
    
    def accel_callback(self,msg):
        # self.accel_msg.stamp = msg.stamp
        self.accel_msg = msg.linear_acceleration.x
        pass

    def combine(self):
        self.Imu_msg.header.stamp = rospy.Time().now()
        self.Imu_msg.header.frame_id = "base_link"
        self.Imu_msg.linear_acceleration.x = self.accel_msg
        self.Imu_msg.angular_velocity.z = self.gyro_msg


if __name__ == '__main__':
    obj = Imu_pub_sub()