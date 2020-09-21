#!/usr/bin/env python
'''
    Gripper data saver Node
'''
import rospy
from sensor_handler.msg import gripperLimit

def callbackGrip(data):
    # if data.read==1:  #read from file
    #     r = rospy.Rate(300) #300hz
    #     list_gripper = list()
    #     fhandle = open('/home/flav/console_ws/src/profile_handler/save_data/gripper_dat.fla', 'r')
    #     for line in fhandle:
    #         list_gripper.append(int(line))
    #     # file data order: [gripL_open , gripL_close, gripR_open, gripR_close]
    #     gripDat = Gripper()
    #     gripDat.gripL_open = list_gripper[0]
    #     gripDat.gripL_close = list_gripper[1]
    #     gripDat.gripR_open = list_gripper[2]
    #     gripDat.gripR_close = list_gripper[3]
    #     gripDat.read      = 0
    #     #publisher
    #     i = 0
    #     while i <15:
    #         pub_grip.publish(gripDat)
    #         i = i + 1
    #         r.sleep()

     #write to file
    list_saveGrip = list()
    list_saveGrip = [data.grip_open, data.grip_close]
    with  open('/home/flav/console_ws/src/data_handler/save_data/gripper_dat.fla', 'w') as files:                 
        for i in range(len(list_saveGrip)):
            files.write(str(list_saveGrip[i]))
            files.write('\n')  


#init node and subscriber
def gripper_saver():
    rospy.init_node('gripper_save', anonymous = True)
    rospy.Subscriber('save_gripper_data', Gripper, callbackGrip)
    rospy.spin()

#Main program
if __name__ == '__main__':
    try:
        gripper_saver()
    except rospy.ROSInterruptException:
        pass
