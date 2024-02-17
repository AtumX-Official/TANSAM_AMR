#pragma once
#include "CytronMotorDriver.h"
#include "RPMController.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>


//ros::NodeHandle nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 18  /// Y
#define ENC_IN_RIGHT_A 20  /// Y

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 19
#define ENC_IN_RIGHT_B 21

#define ticks_meter 20698

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("rwheel", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("lwheel", &left_wheel_tick_count);

// 100ms interval for measurements
const int publish_interval = 100;
const int control_interval = 50; //100;
long previousMillis_publish = 0;
long previousMillis_control = 0;
long currentMillis = 0;

float wheel_seperation = 0.68;
float wheel_radius = 0.05;

float l_val, r_val, linear_vel = 0, angular_vel = 0;

AMR_Controller controller(wheel_seperation, wheel_radius, control_interval);

// Increment the number of ticks
void right_wheel_tick() {

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }

  if (Direction_right) {

    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;
    }
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;
    }
  }
}

// Increment the number of ticks
void left_wheel_tick() {

  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }

  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;
    }
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;
    }
  }
}

void msgCallback(const geometry_msgs::Twist& state)
{
  linear_vel = state.linear.x;
  angular_vel = state.angular.z;

  //Clamping the linear velocity
  linear_vel = max(-1, linear_vel);
  linear_vel = min(1, linear_vel);

  //Clamping the angular velocity
  angular_vel = max(-1, angular_vel);
  angular_vel = min(1, angular_vel);
}

ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", msgCallback);

void change_pid(const std_msgs::Float32MultiArray& val)
{
  controller.change_val(val.data);
}
ros::Subscriber <std_msgs::Float32MultiArray> sub2("pid",change_pid);

void setup()
{
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  nh.initNode();
  nh.advertise(leftPub);
  nh.advertise(rightPub);
  nh.subscribe(sub);
  nh.subscribe(sub2);
}

void loop()
{
  //Record the time
  currentMillis = millis();

  if (currentMillis - previousMillis_publish > publish_interval) {

    previousMillis_publish = currentMillis;

    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
  }

  if (currentMillis - previousMillis_control > control_interval)
  {
//    dtostrf(linear_vel, 6, 3, buf);
//    nh.loginfo(buf);
    controller.control_velocity(linear_vel, angular_vel, left_wheel_tick_count.data, right_wheel_tick_count.data);
    previousMillis_control = currentMillis;
  }

  nh.spinOnce();
  delay(1);
}
