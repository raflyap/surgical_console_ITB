#!/usr/bin/env python
'''
    Gripper data handler Node Left
    Update = 26 8 2020
'''
import rospy
from data_handler.msg import ui_msgs
from sensor_handler.msg import sensorData, gripperLimit
from std_msgs.msg import UInt16

global gripperData, grip_open, grip_close, sumGripperClosed, sumGripperOpen , count,load,curr_st
gripperData, grip_open, grip_close, sumGripperClosed, sumGripperOpen, countOpen, countClose,load, curr_st = 0,0,0,0,0,0,0,0,0
pub_grip_limit = rospy.Publisher('l_gripper_limit', gripperLimit, queue_size= 200)

def callback_gripState(data):
    global gripperData, grip_open, grip_close, sumGripperClosed, sumGripperOpen, countOpen, countClose, curr_st
    if data.calibGrip == 0 :
        #publish only
        curr_st = 0
        r = rospy.Rate(300)
        i = 0
        grip_limData = gripperLimit()
        grip_limData.open_limit = grip_open
        grip_limData.close_limit = grip_close
        # print('FFFF')
        # while i <15:
        pub_grip_limit.publish(grip_limData)
        r.sleep() 
    elif data.calibGrip != 0 and data.calibArm == 1:
        r = rospy.Rate(300)
        if data.calibGrip == 1:
            curr_st = data.calibGrip
        elif data.calibGrip == 2:
            curr_st = data.calibGrip
        elif data.calibGrip == 3:
            curr_st = data.calibGrip
            #publish the limits
            gripper_msg = gripperLimit()
            gripper_msg.open_limit = grip_open
            gripper_msg.close_limit = grip_close
            i = 0
            list_saveGrip = list()
            list_saveGrip = [grip_open, grip_close]
            with  open('/home/epione/console_ws/src/save_data/l_gripper_dat.fla', 'w') as files:                 
                for i in range(len(list_saveGrip)):
                    files.write(str(list_saveGrip[i]))
                    files.write('\n')  
            pub_grip_limit.publish(gripper_msg)
            r.sleep()

def callbackGripper(data):
    global gripperData, sumGripperOpen, sumGripperClosed, curr_st,countOpen, countClose, grip_open, grip_close
    gripperData = data.gripper_data
    if curr_st == 3:
        countClose = 0
        countOpen = 0
        sumGripperOpen = 0
        sumGripperClosed = 0
    elif (curr_st == 1):
        sumGripperClosed += gripperData
        countClose += 1
        grip_close = sumGripperClosed / (countClose)
    elif (curr_st == 2):
        sumGripperOpen += gripperData
        countOpen += 1
        grip_open = sumGripperOpen / (countOpen)

#init node and subscriber
def gripper_handler():
    global grip_open, grip_close, load
    rospy.init_node('gripper_handler_left', anonymous = True)
    if load == 0:
        #load data
        list_gripperLim = list()
        r = rospy.Rate(300)
        fhandle = open('/home/epione/console_ws/src/save_data/l_gripper_dat.fla', 'r')
        for line in fhandle:
            list_gripperLim.append(int(line))
        gripper_msg = gripperLimit()
        grip_open = list_gripperLim[0]
        grip_close = list_gripperLim[1]
        load = 1
    rospy.Subscriber('UI', ui_msgs, callback_gripState)
    rospy.Subscriber('l_arm_data', sensorData, callbackGripper)
    rospy.spin()

#Main program
if __name__ == '__main__':
    try:
        gripper_handler()
    except rospy.ROSInterruptException:
        pass
