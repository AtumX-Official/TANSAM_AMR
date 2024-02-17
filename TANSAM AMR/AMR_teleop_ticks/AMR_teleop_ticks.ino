#include "CytronMotorDriver.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 18  /// Y
#define ENC_IN_RIGHT_A 20  /// Y
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 19
#define ENC_IN_RIGHT_B 21
 
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
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

CytronMD motor_r(PWM_DIR, 7, 6);
CytronMD motor_l(PWM_DIR, 4, 3);

float linear_vel, angular_vel, l_val, r_val;

// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
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
 
  if(val == LOW) {
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
  linear_vel = max(-1,linear_vel);
  linear_vel = min(1,linear_vel);

  //Clamping the angular velocity
  angular_vel = max(-1,angular_vel);
  angular_vel = min(1,angular_vel);
}

ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", msgCallback);

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
}

void loop()
{
  //Record the time
  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
     
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
  }
  
  if(linear_vel == 0 && angular_vel == 0)
  {
    motor_r.setSpeed(0);
    motor_l.setSpeed(0);
    //delay(100);
  }
  else
  {
    l_val = linear_vel - (0.2*angular_vel);
    r_val = linear_vel + (0.2*angular_vel);

    if(l_val>0)l_val = 30 + (l_val*(225/1.2));
    else l_val = -30 + (l_val*(225/1.2));

    if(r_val>0)r_val = 30 + (r_val*(225/1.2));
    else r_val = -30 + (r_val*(225/1.2));

    motor_l.setSpeed(l_val);
    motor_r.setSpeed(r_val);
    //delay(100);
  }

  nh.spinOnce();
  delay(1);
}
