#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class AngularVelocityController:
    def __init__(self):
        self.desired_angle = 0.0
        self.current_angle = 0.0
        self.angular_velocity = 0.0

        # PID controller parameters
        self.kp = 0.1
        self.ki = 0.0
        self.kd = 0.0

        self.integral = 0.0
        self.previous_error = 0.0

        rospy.init_node('angular_velocity_controller', anonymous=True)

        # Publisher
        self.pub = rospy.Publisher('/our_cmd_vel', Twist, queue_size=10)
        
        # Subscriber
        rospy.Subscriber('/our_cmd_vel2', Twist, self.angle_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def angle_callback(self, msg):
        if msg.angular.x == 1:  # Check if angular.x is 1
            self.desired_angle = msg.linear.y
        self.current_angle = msg.linear.z

    def pid_control(self):
        error = self.desired_angle - self.current_angle
        self.integral += error
        derivative = error - self.previous_error

        self.angular_velocity = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

    def run(self):
        while not rospy.is_shutdown():
            self.pid_control()
            twist_msg = Twist()

            if abs(self.desired_angle - self.current_angle) < 2:  # Threshold to stop
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = self.angular_velocity
            if twist_msg.angular.z > 3.0:
                twist_msg.angular.z = 3.0
            if twist_msg.angular.z < -3.0:
                twist_msg.angular.z = -3.0
               
            
            twist_msg.angular.z *= -1
            
            if(twist_msg.angular.z  > -1.5 and twist_msg.angular.z < 1.5):
            	twist_msg.angular.z = 0
            twist_msg.angular.x = 1.0;
            self.pub.publish(twist_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = AngularVelocityController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

