#!/usr/bin/env python

# Dummy value generator for dx, dy, dz (LEFT HAND)

import rospy
import random
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Twist

# Variables
global x_L, y_L, z_L,  alpha_L, beta_L, gamma_L, Jaw_L, count, up
global flag, last_flag
x_L, y_L, z_L,  alpha_L, beta_L, gamma_L, Jaw_L = 95.97, 126.54, 75, -90.0, 0.0, -105.0, 0.0
count, up = 0, 1
last_flag = time.time()

def callbackTurtle(data):
    global x_L, y_L, z_L,  alpha_L, beta_L, gamma_L, Jaw_L, count, up
    
    alpha_L = alpha_L - (data.linear.x*5)
    beta_L = beta_L - (data.angular.z*5)
    gamma_L = gamma_L + (data.linear.x*5)

def generator():
    global x_L, y_L, z_L,  alpha_L, beta_L, gamma_L, Jaw_L, count, up

    pub_pos_L = rospy.Publisher('l_position', Point, queue_size=100)
    pub_ori_L = rospy.Publisher('l_orientation', Point, queue_size=100)
    pub_jaw_L = rospy.Publisher('l_grip_data', Float64, queue_size=100)
    #pub_reset = rospy.Publisher('RST',UInt8, queue_size=10)
    rospy.Subscriber('turtle1/cmd_vel', Twist, callbackTurtle)

    rospy.init_node('sender_L', anonymous=True)
    rate = rospy.Rate(300) # 10hz
    finish = 0
    while not rospy.is_shutdown():
        # if (up == 1):
        #     Jaw_L = Jaw_L + 1
        #     z_L = z_L - 1
        #     if (finish == 0) :
        #          alpha_L =  alpha_L + 0.6
        #         beta_L = beta_L + 0.6
        #         gamma_L = gamma_L + 1
        #     count = count +1
        #     if (count >= 100):
        #         up = 0
        #         finish = 1
        # else :
        #     Jaw_L = Jaw_L - 1
        #     z_L = z_L + 1
        #     count = count - 1
        #     if (count <= 0):
        #         up = 1

        pos_L = Point()
        ori_L = Point()
        jaw_L = Float64()
        #rst = UInt8()

        pos_L.x, pos_L.y, pos_L.z, ori_L.x, ori_L.y, ori_L.z, jaw_L.data = x_L, y_L, z_L,  alpha_L, beta_L, gamma_L, Jaw_L
        #rst.data = 0
        rospy.loginfo(pos_L)
        rospy.loginfo(ori_L)
        rospy.loginfo(jaw_L)
       
        pub_pos_L.publish(pos_L)
        pub_ori_L.publish(ori_L)
        pub_jaw_L.publish(jaw_L)
        #pub_reset.publish(rst)

        rate.sleep()

# Main Program
if __name__ == '__main__':
    try:
        generator()
    except rospy.ROSInterruptException:
        pass