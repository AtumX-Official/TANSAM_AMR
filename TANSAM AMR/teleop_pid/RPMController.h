#pragma once
#include "CytronMotorDriver.h"
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 error_msg;

ros::Publisher error_pub("error", &error_msg);

//std_msgs::Float32 l_vel;
//std_msgs::Float32 r_vel;
//
//std_msgs::Float32 l_tar;
//std_msgs::Float32 r_tar;
//
//ros::Publisher lvel_pub("l_vel", &l_vel);
//ros::Publisher rvel_pub("r_vel", &r_vel);
//
//ros::Publisher l_target("l_targ", &l_tar);
//ros::Publisher r_target("r_targ", &r_tar);

char buf[16];

CytronMD motor_l(PWM_DIR, 7, 6);
CytronMD motor_r(PWM_DIR, 4, 3);

class PID
{
  private:
    float kp, ki, kd;

    float prev_error;

    float error_diff;
    float error_accum;
    float saturation;

    int interval;

  public:
    PID(float proportional, float integral, float derivative, const int sample_time = 100, float bounds = 255)
    {
      kp = proportional;
      ki = integral;
      kd = derivative;

      interval = sample_time;

      error_accum = 0;
      prev_error = 0;

      saturation = bounds;
    }

    float get_output(float error)
    {
      float output;

      //Calculating the differential component of error
      error_diff = (error - prev_error) / (interval * 0.001);
      prev_error = error;

      //Calculating the integral component of error
      error_accum += (error * interval * 0.001);

      output = (kp * error) + (kd * error_diff) + (ki * error_accum);

      //nh.loginfo("output");
      //dtostrf(output, 10, 3, buf);
      //nh.loginfo(buf);

      // Clamping the output
      output = min(output, saturation);
      output = max(output, -saturation);

      return output;
    }

    void change_val(float val[4])
    {
      kp = val[1];
      ki = val[2];
      kd = val[3];

      prev_error = 0;
      error_accum = 0;

      nh.loginfo("Values updated");
    }
};

class AMR_Controller
{
  private:

    float kp_left = 1.2, ki_left = 37.8, kd_left = 0.0;
    float kp_right = 1.8, ki_right = 37.8, kd_right = 0.0;

    float wheel_seperation, radius;

    float interval;

    int prev_tick_left, prev_tick_right, ticks_rev = 6503;  // Ticks per revolution, 6503
    float target_left, target_right, left_vel, right_vel;
    float error_left, error_right;

    float left_out, right_out;

    PID left_pid = PID(kp_left, ki_left, kd_left);
    PID right_pid = PID(kp_right, ki_right, kd_right);

  public:

    AMR_Controller(float b, float r, const int sample_time)
    {
      wheel_seperation = b; // 0.68 m
      radius = r; // 0.05 m

      interval = sample_time;

      left_out = NULL;
      right_out = NULL;

      prev_tick_left = 0;
      prev_tick_right = 0;

//      nh.advertise(lvel_pub);
//      nh.advertise(rvel_pub);
//
//      nh.advertise(l_target);
//      nh.advertise(r_target);

        nh.advertise(error_pub);
    }

    void control_velocity(float linear_vel, float rot_vel, int tick_left, int tick_right)
    {
      // Calculating the wheel velocities in rad/s
      left_vel = ((tick_left - prev_tick_left) * 2 * 3.142) / (interval * 0.001 * ticks_rev);
      right_vel = ((tick_right - prev_tick_right) * 2 * 3.142) / (interval * 0.001 * ticks_rev);

//      l_vel.data = left_vel;
//      r_vel.data = right_vel;

//      lvel_pub.publish(&l_vel);
//      rvel_pub.publish(&r_vel);

      // Getting the target wheel velocities
      target_left = ((2 * linear_vel) - (wheel_seperation * rot_vel)) / (2 * radius);
      target_right = ((2 * linear_vel) + (wheel_seperation * rot_vel)) / (2 * radius);

//      l_tar.data = target_left;
//      r_tar.data = target_right;

//      l_target.publish(&l_tar);
//      r_target.publish(&r_tar);

      // Error function
      error_left = target_left - left_vel;
      error_right = target_right - right_vel;

      error_msg.data = error_left;
      error_pub.publish(&error_msg);
      
      left_out = left_pid.get_output(error_left);
      right_out = right_pid.get_output(error_right);

      // Setting motor speed
      motor_l.setSpeed(left_out);
      motor_r.setSpeed(right_out);

      // Updating the tick
      prev_tick_left = tick_left;
      prev_tick_right = tick_right;
    }

    void change_val(float val[4])
    {
      if (val[0] == 0)left_pid.change_val(val);
      else right_pid.change_val(val);
    }

    float get_left_out()
    {
      return left_out;
    }

    float get_right_out()
    {
      return right_out;
    }

};
