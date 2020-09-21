#!/usr/bin/env python

# Dummy value generator for dx, dy, dz (RIGHT HAND)

# Import libraries
import rospy
import random
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Twist

# Variables
global x_R, y_R, z_R, alpha_R, beta_R, gamma_R, Jaw_R ,count, up
x_R, y_R, z_R, alpha_R, beta_R, gamma_R, Jaw_R = 95.97, 277.34, 75, -90.0, 0.0, -105.0, 0.0
count, up = 0, 1

def callbackTurtle(data):
    global x_R, y_R, z_R, alpha_R, beta_R, gamma_R, Jaw_R ,count, up
    
    alpha_R = alpha_R - (data.linear.x*5)
    beta_R = beta_R - (data.angular.z*5)
    gamma_R = gamma_R + (data.linear.x*5)

def generator():
    global x_R, y_R, z_R, alpha_R, beta_R, gamma_R, Jaw_R, count, up
    
    pub_pos_R = rospy.Publisher('r_position', Point, queue_size=100)
    pub_ori_R = rospy.Publisher('r_orientation', Point, queue_size=100)
    pub_jaw_R = rospy.Publisher('r_grip_data', Float64, queue_size=100)
    rospy.Subscriber('turtle1/cmd_vel', Twist, callbackTurtle)

    rospy.init_node('sender_R', anonymous=True)
    rate = rospy.Rate(300) # 10hz
    finish = 0
    while not rospy.is_shutdown():
        # if (up == 1):
        #     Jaw_R = Jaw_R + 1
        #     z_R = z_R + 1
        #     if (finish == 0) :
        #         alpha_R = alpha_R + 0.6
        #         beta_R = beta_R + 0.6
        #         gamma_R = gamma_R + 1
        #     count = count +1
        #     if (count >= 100):
        #         up = 0
        #         finish = 1
        # else :
        #     Jaw_R = Jaw_R - 1
        #     z_R = z_R  - 1
        #     count = count - 1
        #     if (count <= 0):
        #         up = 1

        pos_R = Point()
        ori_R = Point()
        jaw_R = Float64()

        pos_R.x, pos_R.y, pos_R.z, ori_R.x, ori_R.y, ori_R.z, jaw_R.data = x_R, y_R, z_R, alpha_R, beta_R, gamma_R, Jaw_R
       
        rospy.loginfo(pos_R)
        rospy.loginfo(ori_R)
        rospy.loginfo(jaw_R)
        
        pub_pos_R.publish(pos_R)
        pub_ori_R.publish(ori_R)
        pub_jaw_R.publish(jaw_R)

        rate.sleep()

# Main Program
if __name__ == '__main__':
    try:
        generator()
    except rospy.ROSInterruptException:
        pass