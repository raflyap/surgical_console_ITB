#!/usr/bin/env python

'''
    Client node Left Hand

    TA192001004
    "Master Console Robotics Surgery"
    
    Last Edited : August 30, 2020

    Below this, there are two main programs with its protocol :
    1. UDP Protocol 
    2. TCP Protocol
    
    The default state is using UDP Protocol. 
    If you want to use TCP please comment UDP section, and uncomment the TCP section
'''
# UDP Protocol

# Import libraries
import socket 
import rospy
import time 

# Import libraries
from std_msgs.msg import Float64, UInt8
from geometry_msgs.msg import Point
from signal_condition.msg import ui_msgs

# Initiate socket object
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Initiate left hand server
host = '192.168.123.2'
port = 1920
print('Starting up socket to port %s' % port)

# Define global variables
global first, reset
global x_L, y_L, z_L, alpha_L, beta_L, gamma_L, grip_L
global repos, running
global curr_t, freq, curr_t_f, freq_f
global length 
global cnt_err

# Initialization of variables
first, reset = 1, 0.0
x_L, y_L, z_L, alpha_L, beta_L, gamma_L, grip_L = 95.97, 126.54, 75.0, 0.0, 0.0, 0.0, 0.0
repos, running = 1, 1
curr_t, freq, curr_t_f, freq_f = time.time(), 0, time.time(), 0
length = 0
cnt_err = 0

# Callback after subscribe position data
def callbackPos_L(data):
    global first, curr_t, freq, curr_t_f, freq_f, reset
    global x_L, y_L, z_L, alpha_L, beta_L, gamma_L, grip_L
    global repos, running
    global pub_flag_client, pub_flag_colls
    global length
    global cnt_err

    # Save subsciribed data to a variables
    x_L, y_L, z_L  = data.x, data.y, data.z

    # Check if data is ready to be sent
    if(first == 1):
        rospy.loginfo("Data ready...")
        first = 0

    else :
        # Message making
        msg = ';' + format(x_L, '.4f')+ ';' + format(y_L, '.4f') + ';' + format(z_L, '.4f') + ';' + format(alpha_L, '.4f') + ';' + format(beta_L, '.4f')+ ';' + format(gamma_L, '.4f') + ';' + str(grip_L)
        msg = msg + ';' + str(reset) + ';' + str(repos) + ';' + str(running)
        msg = msg.encode('utf-8')

        # Send the message to the server
        sock.sendto(msg, (host, port))
        # rospy.loginfo('\n\nSending LEFT HAND :\n\nx=%.4f\ty=%.4f\tz=%.4f\n\nOrientation :\nalpha=%.4f\tbeta=%.4f\tgamma=%.4f\ngrip=%d', x_L, y_L, z_L, alpha_L, beta_L, gamma_L, grip_L)
        
        # Calculate sending frequency
        if (time.time()-curr_t < 1):
            freq = freq + 1
        else :
            rospy.loginfo('Sending frequency on LEFT HAND : %d times/s', freq)
            freq = 0
            curr_t = time.time()
        
        # Receive acknowledge from server
        try :
            feedback, address = sock.recvfrom(1024, socket.MSG_DONTWAIT)
            if feedback :
                length = length + len(feedback)
                feedback = feedback.decode('utf-8')
                list_data = feedback.split(";")
                ack = int(list_data[1])
                f_colls = int(list_data[2])

                ack_pub = UInt8()
                f_colls_pub = UInt8()
                ack_pub.data, f_colls_pub.data, = 1, f_colls

                if (time.time()-curr_t_f < 1):
                    freq_f = freq_f + 1
                else :
                    throughput = float(length) / 1000
                    rospy.loginfo('Receiving feedback frequency on LEFT HAND : %d times/s, throughput : %.4f kBps', freq_f, throughput)
                    freq_f = 0
                    length = 0
                    curr_t_f = time.time()
                cnt_err = 0

                # Publish
                pub_flag_client.publish(ack_pub)
                pub_flag_colls.publish(f_colls_pub)
        except :
            cnt_err += 1
            if (cnt_err >= 100):
                rospy.loginfo('LEFT HAND server is not responding')
                ack_pub = UInt8()
                ack_pub = 0
                cnt_err = 0
                # Publish flag to logger
                pub_flag_client.publish(ack_pub)
    
# Callback after subscribe orientation data
def callbackOri_L(data):
    global alpha_L, beta_L, gamma_L
    alpha_L, beta_L, gamma_L  = data.x, data.y, data.z

# Callback after subscribe jaw data
def callbackgrip_L(data):
    global grip_L
    grip_L = data.data

# Callback after subscribe reset data
def callbackUI(data):
    global reset, repos, running
    repos = (data.sys_flag & 4)/4
    running = data.sys_flag & 1
    reset = (data.sys_flag & 8)/8
    
def client():
    global pub_flag_client, pub_flag_colls

    # Creating ROS node and subscribers
    rospy.init_node('clientL', anonymous=True)
    rospy.Subscriber("l_position", Point, callbackPos_L)
    rospy.Subscriber("l_orientation", Point, callbackOri_L)
    rospy.Subscriber("l_grip_data", Float64, callbackgrip_L)
    rospy.Subscriber("UI",ui_msgs, callbackUI)
    pub_flag_client = rospy.Publisher("l_flag_client", UInt8, queue_size=100)
    pub_flag_colls = rospy.Publisher("l_flag_colls", UInt8, queue_size=100)

    rospy.spin()

# Main program
if __name__ == '__main__':
    try :
        client()
    except rospy.ROSInterruptException:
        pass

# TCP

# # Import libraries
# import socket 
# import rospy
# import time
# from std_msgs.msg import Float64, UInt8
# from geometry_msgs.msg import Point
# from signal_condition.msg import ui_msgs

# # Initiate socket object
# sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# host = '192.168.123.2'
# port = 1920

# while True:
#     try:
#         sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         sock.connect((host, port))
#         print('connected to %s port %s' %(host, port))
#         break
#     except socket.error:
#         print('Connection Failed, Retrying..')
#         time.sleep(1)

# # Variables
# global first, value, reset
# global x_L, y_L, z_L, alpha_L, beta_L, gamma_L, grip_L
# global repos, running
# global curr_t, freq
# global length

# first, reset = 1, 0.0
# x_L, y_L, z_L, alpha_L, beta_L, gamma_L, grip_L = 95.97, 126.54, 75.0, 0.0, 0.0, 0.0, 0.0
# repos, running = 1, 1
# curr_t, freq = time.time(), 0
# length = 0
# # Callback after subscribe position data

# def callbackPos_L(data):
#     global first, curr_t, freq, reset
#     global x_L, y_L, z_L, alpha_L, beta_L, gamma_L, grip_L
#     global repos, running
#     global pub_flag_client, pub_flag_colls
#     global length
#     x_L, y_L, z_L  = data.x, data.y, data.z

#     if(first == 1):
#         rospy.loginfo("Data ready...")
#         first = 0

#     else :
#         # Sending data to server
#         try:
#             msg = ';' + format(x_L, '.4f')+ ';' + format(y_L, '.4f') + ';' + format(z_L, '.4f') + ';' + format(alpha_L, '.4f') + ';' + format(beta_L, '.4f')+ ';' + format(gamma_L, '.4f') + ';' + str(grip_L)
#             msg = msg + ';' + str(reset) + ';' + str(repos) + ';' + str(running)
#             msg = msg.encode('utf-8')
#             length = length + len(msg)
#             sock.sendall(msg)

#             # rospy.loginfo('\n\nSending LEFT HAND :\n\nx=%.4f\ty=%.4f\tz=%.4f\n\nOrientation :\nalpha=%.4f\tbeta=%.4f\tgamma=%.4f\ngrip=%d', x_L, y_L, z_L, alpha_L, beta_L, gamma_L, grip_L)
#             if (time.time()-curr_t < 1):
#                 freq = freq + 1
#             else :
#                 rospy.loginfo('Sending frequency on LEFT HAND: %d times/s with length : %d', freq, length)
#                 freq = 0
#                 curr_t = time.time()
#                 length = 0
#             feedback = sock.recv(1024, socket.MSG_DONTWAIT)
#             if feedback :
#                 feedback = feedback.decode('utf-8')
#                 list_data = feedback.split(";")
#                 ack = int(list_data[0])
#                 f_colls = int(list_data[1])

#                 ack_pub = UInt8()
#                 f_colls_pub = UInt8()
#                 ack_pub.data, f_colls_pub.data, = 1, f_colls

#                 # Publish
#                 pub_flag_client.publish(ack_pub)
#                 pub_flag_colls.publish(f_colls_pub)
#         except:
#             ack_pub = UInt8()
#             ack_pub = 0
#             # Publish flag to logger
#             pub_flag_client.publish(ack_pub)
    
# def callbackOri_L(data):
#     global alpha_L, beta_L, gamma_L
#     alpha_L, beta_L, gamma_L  = data.x, data.y, data.z

# def callbackgrip_L(data):
#     global grip_L
#     grip_L = data.data

# # Callback after subscribe reset data
# def callbackUI(data):
#     global reset, repos, running
#     repos = (data.sys_flag & 4)/4
#     running = data.sys_flag & 1
#     reset = (data.sys_flag & 8)/8
    
# def client():
#     # Creating ROS node and subscribers
#     global pub_flag_client , pub_flag_colls

#     rospy.init_node('clientL', anonymous=False)
#     rospy.Subscriber("l_position", Point, callbackPos_L)
#     rospy.Subscriber("l_orientation", Point, callbackOri_L)
#     rospy.Subscriber("l_grip_data",Float64, callbackgrip_L)
#     rospy.Subscriber("UI",ui_msgs, callbackUI)
#     pub_flag_client = rospy.Publisher("l_flag_client", UInt8, queue_size=100)
#     pub_flag_colls = rospy.Publisher("l_flag_collision", UInt8, queue_size=100)
#     rospy.spin()

# # Main program
# if __name__ == '__main__':
#     try :
#         client()
#     except rospy.ROSInterruptException:
#         pass