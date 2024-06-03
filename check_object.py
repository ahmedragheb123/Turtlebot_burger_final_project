#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist

# Initialize global variables for object position
x = 0
y = 0

def callback(data):
    global x, y
    x = data.x
    y = data.y

def listener():
    rospy.init_node('object_position_subscriber', anonymous=True)

    rospy.Subscriber('object_positions', Point, callback)

    # Initialize a publisher for cmd_vel
    pub = rospy.Publisher('our_cmd_vel', Twist, queue_size=10)

    # Create a Twist message
    twist_msg = Twist()

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if (y > 395):
            twist_msg.linear.x = 0  # Set linear velocity
            twist_msg.angular.z = 0  # Set angular velocity
        # Publish the Twist message
            pub.publish(twist_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

