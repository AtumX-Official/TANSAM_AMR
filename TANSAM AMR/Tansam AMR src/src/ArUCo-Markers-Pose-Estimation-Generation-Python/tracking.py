#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import TransformStamped,Twist
import matplotlib.pyplot as plt
from time import time
import math

class PID:

    def __init__(self,kp,ki,kd,upper_clamp=0.5,lower_clamp=-0.5,bias=0.1):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.upper_clamp = upper_clamp
        self.lower_clamp = lower_clamp
        self.bias = bias

        self.prev_error = 0
        self.error_accum = 0
        self.prev_millis = 0

        pass

    def get_output(self,error):
              
        millis = rospy.Time.now().nsecs * 10e-6
        # millis = int(time() )
        sample_time = millis - self.prev_millis
        
        self.error_diff = (error - self.prev_error)/(sample_time * 10e-3)
        # print(sample_time)
        self.error_accum += (error * sample_time)

        output = -((self.kp * error) + (self.ki * self.error_accum) + (self.kd * self.error_diff))
        
        # if output<0:
        #     output -= self.bias
        # else:
        #     output += self.bias

        output = min(self.upper_clamp, output)
        output = max(self.lower_clamp, output)

        self.prev_millis = millis

        # print(output)

        return output

class Controller:
    def __init__(self):
        
        rospy.init_node("Docking_node",anonymous=True)
        rospy.Subscriber("/marker_tf",TransformStamped,self.callback)
        pub = rospy.Publisher("/cmd_vel",Twist,queue_size=3)
        
        self.angular_kp = 7.55 #5 7.7 7.85
        self.angular_ki = 0.000025 # 0.0005
        self.angular_kd = 3.5 #0.8 2.77 2.77

        self.linear_kp = 6.7 #5 7.7 7.85
        self.linear_ki = 0.000025 # 0.0005
        self.linear_kd = 5.5 #0.8 2.77 2.77

        self.angular_pid = PID(self.angular_kp,self.angular_ki,self.angular_kd)

        self.linear_pid = PID(self.linear_kp,self.angular_ki,self.linear_kd)
        
        cmd = Twist()

        self.time_list = []
        self.error_list = []

        start_time = time() * 10e3

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                error_x = (self.left_val_x + self.right_val_x)/2
                error_z = (self.left_val_z + self.right_val_z)/2
                error_linear = error_z - 0.4
                error = math.atan(error_x/error_z)
                curr_time = int(time() * 10e3)
                self.error_list.append(error_linear)
                self.time_list.append(curr_time - start_time)
                
                if abs(error)>0.001:
                    # print("error",error)
                    # cmd.angular.z = self.angular_pid.get_output(error)
                    # print("output",cmd.angular.z)
                    # pub.publish(cmd)
                    self.left_val_x = None
                    self.left_val_z = None
                    self.right_val_x = None
                    self.right_val_z = None
                    rate.sleep()
                else:
                    # cmd.angular.z = 0
                    # pub.publish(cmd)
                    pass
                if error_z >0.01:
                    cmd.linear.x = self.linear_pid.get_output(error_linear)
                    print(cmd.linear.x)
                    pub.publish(cmd)
                    continue
            except:
                cmd.angular.z = 0
                cmd.linear.x = 0
                pub.publish(cmd)
                rate.sleep()
                continue
        plt.plot(self.time_list, self.error_list)
        plt.axhline(y = 0.0, color = 'r', linestyle = '-')
        plt.axhline(y = 0.3, color = 'b', linestyle = '-')
        plt.show()

        pass

    def callback(self,msg):
        if msg.child_frame_id == "marker/zero":
            self.left_val_x = msg.transform.translation.x
            self.left_val_z = msg.transform.translation.z
        else:
            self.right_val_x = msg.transform.translation.x
            self.right_val_z = msg.transform.translation.z
        pass

if __name__ == '__main__':
    obj = Controller()
    pass