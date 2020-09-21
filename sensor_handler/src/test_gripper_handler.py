#!/usr/bin/env python
#saving to file
import rospy
from profile_handler.msg import Gripper
from data_handler.msg import ui_msgs
from sensor_handler.msg import sensorData

def bacot():
    pub_grip_instr = rospy.Publisher('UI', ui_msgs, queue_size = 100)
    pub_grip_data = rospy.Publisher('r_arm_data_raw', sensorData, queue_size = 100)
    rospy.init_node('gripper_handlertest', anonymous = True)
    r = rospy.Rate(300) 
      
     #message for publisher
    gripDat = sensorData()
    grip_instr = ui_msgs()
    grip_instr.calibGrip = 0
    # gripDat.gripper_data = 876
    while not rospy.is_shutdown():
        state = 0
        while state < 100 :
            grip_instr.calibGrip = 0
            
            gripDat.gripper_data = 800
            #pub_grip_data.publish(gripDat)
            pub_grip_instr.publish(grip_instr)
            print('This instr:',gripDat.gripper_data, state)
            state = state +1
            r.sleep()
        while (state >= 100) and state <200:
            grip_instr.calibGrip = 1
            state = state +1
            gripDat.gripper_data = 800
            pub_grip_instr.publish(grip_instr)
            pub_grip_data.publish(gripDat)
            print('This instr:',grip_instr.calibGrip)
            r.sleep()
        while (state>= 200) and state < 300:
            grip_instr.calibGrip = 2
            state = state +1
            gripDat.gripper_data = 700
            pub_grip_instr.publish(grip_instr)
            pub_grip_data.publish(gripDat)
            print('This instr:',grip_instr.calibGrip)
            r.sleep()
        while (state>= 300) and state < 400:
            grip_instr.calibGrip = 3
            state = state +1
            pub_grip_instr.publish(grip_instr)
            pub_grip_data.publish(gripDat)
            print('This instr:',grip_instr.calibGrip)
            r.sleep()
        while (state>= 400) and state < 600:
            grip_instr.calibGrip = 0
            state = state +1
            #pub_grip_data.publish(gripDat)
            pub_grip_instr.publish(grip_instr)
            print('This instr:',grip_instr.calibGrip)
            r.sleep()
    # the shubbansha
    # i = 0
    # while not rospy.is_shutdown():
    #         pub_grip_data.publish(gripDat)
    #         pub_grip_instr.publish(grip_instr)
    #         i = i +1
    #         print('publisher test', i)
    #         r.sleep()
    rospy.spin()
if __name__ == '__main__':
    try:
        bacot()
    except rospy.ROSInterruptException: pass
