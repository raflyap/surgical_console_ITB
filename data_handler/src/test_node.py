#!/usr/bin/env python
#saving to file
import rospy
from data_handler.msg import Profile
global profA, profB, profC

def talker():
    pub_prof = rospy.Publisher('profile_instruction',Profile, queue_size = 100)
    rospy.init_node('profile_handler', anonymous = True)
    r = rospy.Rate(300) #10hz
    global P_A, P_B, P_C
    list_profile_R = list() #read from file
    fhandle = open('/home/epione/console_ws/src/data_handler/save_data/LCD.fla','r')
    #list appended with data from file
    for line in fhandle:
        list_profile_R.append(float(line))
    P_A = (list_profile_R[0])
    P_B = 0.56#(list_profile_R[1])
    P_C = 0.65#(list_profile_R[2])
    print(P_A)
     #message for publisher
    profDat = Profile()
    #profDat.prof1, profDat.prof2, profDat.prof3,profDat.load = P_A, P_B, P_C, 0
    profDat.load, profDat.prof_value, profDat.prof_ID = 0, P_A, 2
    # the shubbansha
    while not rospy.is_shutdown():
    #     rospy.loginfo(profDat)
        pub_prof.publish(profDat)
        r.sleep()
    rospy.spin()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
