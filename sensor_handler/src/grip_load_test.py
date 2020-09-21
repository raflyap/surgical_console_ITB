#!/usr/bin/env python
#loading from file to LCD
import rospy
from sensor_handler.msg import Gripper
pub_load = rospy.Publisher('save_gripper_data', Gripper, queue_size = 100)

def listener(data):
    list_grip = list()
    list_grip = [data.gripL_open, data.gripL_close, data.gripR_open, data.gripR_close]
    with  open('/home/epione/console_ws/src/data_handler/save_data/gripper.fla', 'w') as dame_da_ne:  
        for i in range(len(list_grip)):
            dame_da_ne.write(str(list_grip[i]))
            dame_da_ne.write('\n') 


def subs():
    rospy.init_node('gripper_datLoad', anonymous= True)
    rospy.Subscriber('load_gripper_data',Gripper,listener)
    gripLoad = Gripper()
    r = rospy.Rate(300)
    gripLoad.read = 1
    gripLoad.gripL_open = 1
    gripLoad.gripL_close = 2
    gripLoad.gripR_open = 1
    gripLoad.gripR_close = 2
    # the shubbansha
    while not rospy.is_shutdown():
    #     rospy.loginfo(gripLoad)
        pub_load.publish(gripLoad)
        r.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        subs()
    except rospy.ROSInterruptException: pass
