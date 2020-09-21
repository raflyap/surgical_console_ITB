#!/usr/bin/env python
'''
    Profile Handler Node
    update: 1 9 2020
'''
#libraries
import rospy
from data_handler.msg import Profile
from std_msgs.msg import UInt8, Float32
global i
i = 0

pub_prof = rospy.Publisher('load_profile',Profile, queue_size = 100)


#callback from subscriber
def callbackProf(data):
    if data.load == 1: #user asks to read from file and publish
        
        r = rospy.Rate(300) #300hz
        list_profile_R = list() #read from file
        fhandle = open('/home/epione/console_ws/src/save_data/profiles.fla','r')
        #list appended with data from file
        for line in fhandle:
            list_profile_R.append(float(line))
        #message for publisher
        profDat = Profile()
        profDat.load, profDat.prof_value, profDat.prof_ID = 0, list_profile_R[data.prof_ID], 0
        #saving previous ID selected, for UI pre load
        with  open('/home/epione/console_ws/src/save_data/preload_dat.fla', 'w') as fh:                 
            fh.write(str(data.prof_ID))
        # the Publisher
        i = 0
        while i < 10:
            print(list_profile_R, i)
            pub_prof.publish(profDat)
            i = i + 1
            r.sleep()
        
    elif data.load == 0: # user asks to save data from subscriber
        list_prof = list()
        # open file to modify the correct line
        fhandle = open('/home/epione/console_ws/src/save_data/profiles.fla', 'r')
        #list appended with data from file
        for line in fhandle:
            list_prof.append(float(line))
        fhandle.close()

        list_prof[data.prof_ID] = round(data.prof_value,3)
        print(list_prof)
        with  open('/home/epione/console_ws/src/save_data/profiles.fla', 'w') as files:                 
            for i in range(len(list_prof)):
                files.write(str(round(list_prof[i],3)))
                if(i < 2):
                    files.write('\n')  
        #saving previous ID selected, for UI pre load
        with  open('/home/epione/console_ws/src/save_data/preload_dat.fla', 'w') as fh:                 
            fh.write(str(data.prof_ID))
def callbackPreload(data):
    #called first time ui fires up
    global i
    i = i + 1
    if data.data == 1:
        preload_list = list()
        fileID = open('/home/epione/console_ws/src/save_data/preload_dat.fla', 'r')
        for line in fileID:
            profile_ID = int(line)
        fileOpen =open('/home/epione/console_ws/src/save_data/profiles.fla', 'r')
        for line in fileOpen:
            preload_list.append(float(line))
        
        preloadProf = Profile()
        preloadProf.prof_value = preload_list[profile_ID]
        # preloadProf.load, preloadProf.prof_value, preloadProf.prof_ID = 0, preload_list[profile_ID], 0
        r = rospy.Rate(300)
        count = 0
        while count < 10:
            print(preload_list, i)
            print(profile_ID,preloadProf.prof_value)
            pub_prof.publish(preloadProf)
            count +=1
            r.sleep()
    else:
        print('wrong data')
#init node and subscriber
def data_handler():
    rospy.init_node('profile_handler', anonymous = True)
    rospy.Subscriber('profile_instruction', Profile, callbackProf)
    rospy.Subscriber('preload', UInt8, callbackPreload)
    rospy.spin()

#Main program
if __name__ == '__main__':
    try:
        data_handler()
    except rospy.ROSInterruptException:
        pass
