// Include necessary libraries for the Arduino board
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <control_motors.h>
#include <Encoder.h>
#include <MPU6500_WE.h>
#include <Wire.h>
#define MPU6500_ADDR 0x68


MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

bool rotationflag = false;
extern float theta_sp;
float flagID;
// Initialize ROS node handle
ros::NodeHandle nh;

float angle_z = 0;
long prev_time = 0;
long prevT = 0;

float desired_angle = 90;

// Initialize ROS publisher
geometry_msgs::Twist cmd_vel_msg2;
ros::Publisher pub("/our_cmd_vel2", &cmd_vel_msg2);



void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {

  flagID = cmd_vel_msg.angular.x;

  CMD_TO_RPM_REQ( (float) cmd_vel_msg.linear.x, (float) cmd_vel_msg.angular.z);  

  int finger = (int) cmd_vel_msg.linear.z;
  int colour = (int) cmd_vel_msg.linear.y;
  if(finger == 1 || colour == 1){
    theta_sp = angle_z;
    
    rotationflag = false;
    CMD_TO_RPM_REQ(0.2, 0);

  }
  
  else if(finger == 2 || colour == 2){
    theta_sp = angle_z;
    
    rotationflag = false;
    CMD_TO_RPM_REQ(-0.2, 0);
  }
  else if(finger == 3 || colour == 3){
    desired_angle = angle_z + 90;
    rotationflag = true;
    CMD_TO_RPM_REQ(0, 0);
   cmd_vel_msg2.angular.x = 1.0;
   cmd_vel_msg2.linear.y = desired_angle;
   cmd_vel_msg2.linear.z = angle_z; 
   pub.publish(&cmd_vel_msg2);
  }
  

  
}


ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("our_cmd_vel", &cmdVelCallback);


void setup(){

  nh.getHardware()->setBaud(57600);

  // Initialize ROS node
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(cmd_vel_sub);
 
  // Initialize the encoder, motors, and MPU6050

  initialize_encoder();
  intiate_motors();

  Wire.begin();
  myMPU6500.init();
  myMPU6500.autoOffsets();
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
}

void loop(){

  cmd_vel_msg2.angular.x = 0.0;
   cmd_vel_msg2.linear.y =  0.0;
  long current_time = micros();
  float dt = ((float) (current_time - prev_time)) / 1.e06;
  prev_time = current_time; 

  xyzFloat gyr = myMPU6500.getGyrValues();
  angle_z += gyr.z * dt;
  cmd_vel_msg2.linear.z = (float) angle_z;
  
 
  


  if(millis() - prevT > 100)
  {
    
    pub.publish(&cmd_vel_msg2);
    
    prevT = millis();
  }
  
 
  //CMD_TO_RPM_REQ(0.2, 0);
  PID_control_motors();
  angular_vel_func();
  
  // Spin ROS node
  nh.spinOnce();
}