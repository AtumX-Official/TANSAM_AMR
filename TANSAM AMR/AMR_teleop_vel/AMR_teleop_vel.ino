#include "CytronMotorDriver.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

#define ENC_L_A 18
#define ENC_R_A 20

#define ENC_L_B 19
#define ENC_R_B 21

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

CytronMD motor_r(PWM_DIR, 7, 6);
CytronMD motor_l(PWM_DIR, 4, 3);

float linear_vel, angular_vel, l_val, r_val;

const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

const long full_rot_tick_val = 6457;

long right_wheel_tick_count,left_wheel_tick_count;
long prev_right_wheel_tick_count = 0,prev_left_wheel_tick_count = 0;

std_msgs::Float64 l_vel;
ros::Publisher left_pub("l_omega", &l_vel);

std_msgs::Float64 r_vel;
ros::Publisher right_pub("r_omega", &r_vel);


// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_R_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right)right_wheel_tick_count++;
  else right_wheel_tick_count--;
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_L_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left)left_wheel_tick_count++;  
  else left_wheel_tick_count--;
}

void msgCallback(const geometry_msgs::Twist& state)
{
  linear_vel = state.linear.x;
  angular_vel = state.angular.z;
}

ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", msgCallback);

void setup()
{
  // Set pin states of the encoder
  pinMode(ENC_L_A , INPUT_PULLUP);
  pinMode(ENC_L_B , INPUT);
  pinMode(ENC_R_A , INPUT_PULLUP);
  pinMode(ENC_R_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), right_wheel_tick, RISING);
  
  nh.initNode();
  nh.advertise(left_pub);
  nh.advertise(right_pub);
  nh.subscribe(sub);
}

void loop()
{
  currentMillis = millis();
  if(currentMillis - previousMillis > interval)
  {
    l_vel.data = (left_wheel_tick_count - prev_left_wheel_tick_count)/(full_rot_tick_val*interval*1e-3);
    r_vel.data = (right_wheel_tick_count - prev_right_wheel_tick_count)/(full_rot_tick_val*interval*1e-3);

    left_pub.publish(&l_vel);
    right_pub.publish(&r_vel);

    //Updating the variables:
    previousMillis = currentMillis;
    prev_left_wheel_tick_count = left_wheel_tick_count;
    prev_right_wheel_tick_count = right_wheel_tick_count;
  }
  
  if(linear_vel == 0 && angular_vel == 0)
  {
    motor_r.setSpeed(0);
    motor_l.setSpeed(0);
    delay(500);
  }
  else
  {
    if(linear_vel > 0)linear_vel = 1;
    else if(linear_vel < 0) linear_vel = -1;
    else linear_vel = 0;

    if(angular_vel > 0)angular_vel = 1;
    else if(angular_vel < 0)angular_vel = -1;
    else angular_vel = 0;

    l_val = linear_vel - angular_vel;
    r_val = linear_vel + angular_vel;

    l_val = map(l_val,-1.5,1.5,-255,255);
    r_val = map(r_val,-1.5,1.5,-255,255);

    motor_l.setSpeed(l_val);
    motor_r.setSpeed(r_val);
    delay(100);
  }

  nh.spinOnce();
  delay(1);
}
