#include <ros/ros.h>
#include <iostream>
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
//#include "std_msgs/Byte.h"
#include "std_msgs/String.h"
#include <stdint.h>
#include "geometry_msgs/Point.h"
#include "data_handler/ui_msgs.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sig_Cond_test");

  ros::NodeHandle nc;
  
  //publisher
  ros::Publisher dRawPos_pub = nc.advertise<const geometry_msgs::Point>("r_dPosition_raw", 100);
  ros::Publisher dRawOri_pub = nc.advertise<const geometry_msgs::Point>("r_dOrientation_raw",100);
  //ros::Publisher mul_pub     = nc.advertise<std_msgs::Float32>("Multiplier",100);
 // ros::Publisher frc_pub     = nc.advertise<std_msgs::UInt16>("forceR",100);
  ros::Publisher pub_UI     = nc.advertise<data_handler::ui_msgs>("UI",100);
  ros::Publisher pub_enc    = nc.advertise<std_msgs::UInt8>("encoder_status",100);

  
  ros::Rate loop_rate(300);

  int count = 0;
  int sys_flag = 0;
  while (ros::ok())
  {

    geometry_msgs::Point rawPos_msg;
    rawPos_msg.x = 2.44;
    rawPos_msg.y = 4.8;
    rawPos_msg.z = -2.44;

    geometry_msgs::Point rawOri_msg;
    rawOri_msg.x = 10.05;
    rawOri_msg.y = 0.05;
    rawOri_msg.z = -0.456;

    //UI messages
    sys_flag   = 0b00000100; // start_stop off, power on, repos off, reset off
    //sys_flag = 0b00000101; // start stop on power on ,repos off, reset off
    //sys_flag = 0b00001101; // start stop on, power on, repos off, reset on
    //sys_flag = 0b00000001; // repos on all same
    data_handler::ui_msgs ui_msg;
    ui_msg.sys_flag = sys_flag; 
    ui_msg.Multi = 1;
    ui_msg.calibID = 0;
    ui_msg.calibArm = 1; // rigth arm

    std_msgs::UInt8 enc_msg;
    enc_msg.data = 0;
    
  //  std_msgs::Float32 multip;
  //   multip.data = 0.5;

    // std_msgs::UInt8 failsaf;
    // failsaf.data = 0x00;

    //  std_msgs::UInt16 frc;
    //  frc.data = 667;


    //ROS_INFO("%f", rawPos_msg.x);

     dRawPos_pub.publish(rawPos_msg);
     dRawOri_pub.publish(rawOri_msg);
     pub_enc.publish(enc_msg);
    //mul_pub.publish(multip);
    // fs_pub.publish(failsaf);
    //frc_pub.publish(frc);
   pub_UI.publish(ui_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
