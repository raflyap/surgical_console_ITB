#!/usr/bin/env python
#saving to file
import rospy
from sensor_handler.msg import gripperLimit

def talker():
    pub_grip = rospy.Publisher('save_gripper_data', Gripper, queue_size = 1000)
    rospy.init_node('gripper_savetest', anonymous = True)
    r = rospy.Rate(300) #10hz
    
    list_gripper = list() #read from file
    fhandle = open('/home/epione/console_ws/src/data_handler/save_data/gripper.fla','r')
    #list appended with data from file
    for line in fhandle:
        list_gripper.append(int(line))
    
     #message for publisher
    gripDat = Gripper()
    gripDat.read      = 0
    gripDat.gripL_open = list_gripper[0]
    gripDat.gripL_close = list_gripper[1]
    gripDat.gripR_open = list_gripper[2]
    gripDat.gripR_close = list_gripper[3]
    
    # the shubbansha
    while not rospy.is_shutdown():
            pub_grip.publish(gripDat)
            r.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
