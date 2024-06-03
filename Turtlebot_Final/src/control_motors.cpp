#include <control_motors.h>
#include <Encoder.h>


// Declare external variables for right and left RPM
extern float rightEncoder;
extern float leftEncoder;
extern float angle_z;

extern bool rotationflag;
extern float flagID;

float theta_sp = 0;   ///////////////////////////////////////////////////////////////////////
float err_theta = 0;

float rpm_left_required = 0;
float rpm_right_required = 0;


bool direction = true;   //true forward , false backward

// Motor pins
const int LEFT_PWM_PIN = 5; // PWM pin for left motor speed control
const int LEFT_INT1_PIN = 6; // Control pin 1 for left motor
const int LEFT_INT2_PIN = 7; // Control pin 2 for left motor
const int RIGHT_PWM_PIN = 9; // PWM pin for right motor speed control
const int RIGHT_INT1_PIN = 8; // Control pin 1 for right motor
const int RIGHT_INT2_PIN = 4; // Control pin 2 for right motor

// Motor calibration values
const float MAX_SPEED = 210.0; // Maximum RPM of the motor
const float WHEEL_RADIUS = 0.033; // Radius of the wheel in meter
const float center_distance = 0.16; // 


float eint_right = 0;
float eprev_right = 0;

float eint_left = 0;
float eprev_left = 0;

float kp = 1;
float ki = 0.15;
float kd = 0.0;

float kp_angular = 0.5;

int Input_left = 0;
int Input_right = 0;

int Setpoint_left = 0;
int Setpoint_right = 0;

long prevt_two_pid = 0;


float linear_vel = 0.0;
float angular_vel = 0.0;

void intiate_motors(){
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_INT1_PIN, OUTPUT);
  pinMode(LEFT_INT2_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_INT1_PIN, OUTPUT);
  pinMode(RIGHT_INT2_PIN, OUTPUT);
}


void CMD_TO_RPM_REQ(float linear_x, float angular_z){

    if((flagID ==1.0 && rotationflag == false)||(rotationflag == true && flagID!= 1.0))
    {}
    else{
    // Map linear and angular velocities to left and right motor PWM signals
    
        linear_vel = linear_x;
        angular_vel = angular_z;

        // Calculate left and right wheel velocities based on linear and angular velocities
        rpm_left_required = (((linear_vel*2 + center_distance*angular_vel)/(2*WHEEL_RADIUS))*60)/(2*PI); // Adjusting for angular velocity
        rpm_right_required = (((linear_vel*2 - center_distance*angular_vel)/(2*WHEEL_RADIUS))*60)/(2*PI);// Adjusting for angular velocity
        
        
        

        // Set direction of motors based on velocities
        if(rpm_left_required == 0){
            digitalWrite(LEFT_INT1_PIN, HIGH);
            digitalWrite(LEFT_INT2_PIN, HIGH);
            analogWrite(RIGHT_PWM_PIN , 0);
            analogWrite(LEFT_PWM_PIN , 0);  

        }
        else if (rpm_left_required > 0) {
            digitalWrite(LEFT_INT1_PIN, HIGH);
            digitalWrite(LEFT_INT2_PIN, LOW);
            //direction = true;
            rpm_left_required -= 5;
        } else {
            digitalWrite(LEFT_INT1_PIN, LOW);
            digitalWrite(LEFT_INT2_PIN, HIGH);

            //direction = false;
            rpm_left_required *= -1;
            rpm_left_required -= 8;
        }

        if(rpm_right_required == 0){
            digitalWrite(RIGHT_INT1_PIN, HIGH);
            digitalWrite(RIGHT_INT2_PIN, HIGH);
            analogWrite(RIGHT_PWM_PIN , 0);
            analogWrite(LEFT_PWM_PIN , 0); 
        }
        else if (rpm_right_required > 0) {
            digitalWrite(RIGHT_INT1_PIN, HIGH);
            digitalWrite(RIGHT_INT2_PIN, LOW);
            direction = true;
        } else {
            digitalWrite(RIGHT_INT1_PIN, LOW);
            digitalWrite(RIGHT_INT2_PIN, HIGH);

            rpm_right_required *= -1;
            direction = false;
        }

/*
        Serial.print(rpm_left_required);
        Serial.print("      ");
        Serial.print(rpm_right_required);
        Serial.print("      ");
*/
        //Setpoint_left = (int)(rpm_left_required);
        //Setpoint_right = (int)(rpm_right_required);
    }
}



void PID_control_motors()
{
  if(angular_vel == 0 && rotationflag == false )
  {
    float v1 = rightEncoder / 411 * 60.0; 
    float v2 = leftEncoder / 411 * 60.0;                                  
      
    if (v2 > 210 || v2 < -210){}

    else{
      Input_left = v2 * -1;
    }
                          
    if (v1 > 210 || v1 < -210){}

    else{
      Input_right = v1;
    }

    if(!direction){
      Input_right *= -1;
      Input_left *= -1;
    }

    err_theta = abs(theta_sp - angle_z);

    if (angle_z >= theta_sp)
    {
      if (abs(theta_sp - angle_z) < 5)
      {
        Setpoint_right = rpm_right_required;
        Setpoint_left = rpm_left_required;
      }
      else
      {

        if(!direction){
          Setpoint_right = rpm_right_required + kp_angular * err_theta;
          Setpoint_left = rpm_left_required - kp_angular * err_theta;
        }
        else{
        Setpoint_right = rpm_right_required - kp_angular * err_theta;
        Setpoint_left = rpm_left_required + kp_angular * err_theta;
        }
      }
    }
    else
    {
      if (abs(theta_sp - angle_z) < 5)
      {
        Setpoint_right = rpm_right_required;
        Setpoint_left = rpm_left_required;
      }
      else
      {
          if(!direction){
          Setpoint_right = rpm_right_required - kp_angular * err_theta;
          Setpoint_left = rpm_left_required + kp_angular * err_theta;
          }

        else{
        Setpoint_right = rpm_right_required + kp_angular * err_theta;
        Setpoint_left = rpm_left_required - kp_angular * err_theta;
          }
      
    }
  }

  
  


  long currt = micros();
  float deltat_two_pid = ((float)(currt-prevt_two_pid))/1.0e6;
  prevt_two_pid =currt;



  float e_right = ((float) Setpoint_right - (float) Input_right);
  eint_right = eint_right + e_right*deltat_two_pid;
  float delta_e_right = (e_right-eprev_right)/deltat_two_pid;
  float u_right = kp*e_right + ki*eint_right + kd*delta_e_right;
  u_right = map((long) u_right, 0, 210, 0, 255);

  eprev_right = e_right;

  int pwm_signal_right = (int) fabs(u_right);
  if(pwm_signal_right > 255){
    pwm_signal_right = 255;
  }
  if(pwm_signal_right < 40){
    pwm_signal_right = 40;
  }




  float e_left = ((float) Setpoint_left - (float) Input_left);
  eint_left = eint_left + e_left*deltat_two_pid;
  float delta_e_left = (e_left-eprev_left)/deltat_two_pid;
  float u_left = kp*e_left + ki*eint_left + kd*delta_e_left;
  u_left = map((long) u_left, 0, 210, 0, 255);

  eprev_left = e_left;

  int pwm_signal_left = (int) fabs(u_left);
  if(pwm_signal_left > 255){
    pwm_signal_left = 255;
  }
  if(pwm_signal_left < 40){
    pwm_signal_left = 40;
  }

    
    analogWrite(RIGHT_PWM_PIN , pwm_signal_right);
    analogWrite(LEFT_PWM_PIN , pwm_signal_left);  

/*
    Serial.print(Setpoint_left);
    Serial.print("     ");
    Serial.print(Input_left);

*/

    //analogWrite(RIGHT_PWM_PIN , 120);
    //analogWrite(LEFT_PWM_PIN , 110);  
    }  
  
}
void angular_vel_func(){
    if(linear_vel == 0 && rotationflag == true && flagID == 1.0){
    analogWrite(RIGHT_PWM_PIN , map((long)rpm_right_required, 0, 210, 0, 255));
    analogWrite(LEFT_PWM_PIN ,map((long)rpm_left_required, 0, 210, 0, 255));  
    }
}
