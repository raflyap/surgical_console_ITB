#!/usr/bin/env python
#loading from file to LCD
import rospy
from data_handler.msg import Profile
pub_instr = rospy.Publisher('profile_instruction', Profile, queue_size = 100)

def listener(data):
    with  open('/home/epione/console_ws/src/data_handler/save_data/LCD.fla', 'w') as dame_da_ne:      
        dame_da_ne.write(str(data.prof_value))
        dame_da_ne.write('\n') 


def subs():
    rospy.init_node('profile_handler_load', anonymous= True)
    rospy.Subscriber('load_profile',Profile,listener)
    instrDat = Profile()
    r = rospy.Rate(300)
    #this publishes only once
    instrDat.load, instrDat.prof_value, instrDat.prof_ID = 1, 0.33, 1
    # the shubbansha
    while not rospy.is_shutdown():
    #     rospy.loginfo(instrDat)
        pub_instr.publish(instrDat)
        r.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        subs()
    except rospy.ROSInterruptException: pass
