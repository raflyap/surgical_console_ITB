#include <ros/ros.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include <stdint.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include "data_handler/ui_msgs.h"

//constants
#define posX_ws    95.97
#define posY_ws    126.54
#define posZ_ws    75
#define oriY_ws    -87.4668
#define oriP_ws    0.9420
#define oriR_ws    -106.6126

//class perhitungan absolute
class absoluteCalculation
{
  public:
    float getAbsPos_X(uint8_t rst,float pos, float dPos){
      if (rst == 0){
        pos = pos + dPos;
      }
      else //reset 1
      {
        pos = posX_ws;
      }
    return pos;}
    
    float getAbsPos_Y(uint8_t rst,float pos, float dPos) {
      if (rst == 0){
        pos = pos + dPos;
      }
      else //reset 1
      {
        pos = posY_ws;
      }
    return pos;
    }

    float getAbsPos_Z(uint8_t rst,float pos, float dPos) {
      if (rst == 0){
        pos = pos + dPos;
      }
      else //reset 1
      {
        pos = posZ_ws;
      }
    return pos;
    }
};

//class node subsribe publish nya
class SubscribeAndPublish
{
public:
  absoluteCalculation ac;
  SubscribeAndPublish()
  {
    //published topic
    pub_absPosition    =nr.advertise<geometry_msgs::Point>("l_position",100);
    pub_absOrientation = nr.advertise<geometry_msgs::Point>("l_orientation",100);
    pub_gripper      = nr.advertise<std_msgs::Float64>("l_grip_data",100);

    //Topic you want to subscribe    
    sub_dRPos = nr.subscribe("l_dPosition_raw", 30, &SubscribeAndPublish::rawPosCallback, this);
    sub_dROri = nr.subscribe("l_dOrientation_raw", 30, &SubscribeAndPublish::rawOriCallback, this); 
    sub_ui = nr.subscribe("UI",30, &SubscribeAndPublish::uiCallback, this);
    sub_gripper = nr.subscribe("l_grip_data_raw", 30, &SubscribeAndPublish::gripCallback, this);
    
  }
  void gripCallback(const std_msgs::Float64& grip_msg){
      std_msgs::Float64 gripOut_msg;
      gripOut_msg.data = grip_msg.data   * (!flagCalib & 1) * (!start_stop & 1);
      pub_gripper.publish(gripOut_msg);
  }
  void uiCallback(const data_handler::ui_msgs& ui_msg){
        //output messages
        //Multiplier
      multi = ui_msg.Multi;
        //reset
      reset = (ui_msg.sys_flag >> 3) & 0x01;
        //calibration
       if(ui_msg.calibGrip == 0){
          flagCalib = 0;
        }
      else{ //other number besides 0
          flagCalib = 1;
      }
      //power
      if(((ui_msg.sys_flag >> 1) & 1) == 1){
        system("systemctl poweroff -i");    
      }
      
      //start stop (if 1 then stop)
      start_stop = (ui_msg.sys_flag) & 1;
      //Repositioning for Orientation
      repos = (ui_msg.sys_flag >> 2) & 1;
  }

  void rawPosCallback(const geometry_msgs::Point& dRPos_msg){
    // Message output    
    geometry_msgs::Point posC_msg;

    //output computation.
    dPos_msg_x = dRPos_msg.x * multi * (!flagCalib & 1) * !start_stop;
    dPos_msg_y = dRPos_msg.y * multi * (!flagCalib & 1) * !start_stop;
    dPos_msg_z = dRPos_msg.z * multi * (!flagCalib & 1) * !start_stop;

    pos_X      = ac.getAbsPos_X(reset, pos_X,dPos_msg_x);
    pos_Y      = ac.getAbsPos_Y(reset, pos_Y,dPos_msg_y);
    pos_Z      = ac.getAbsPos_Z(reset, pos_Z,dPos_msg_z);

    posC_msg.x = pos_X;
    posC_msg.y = pos_Y;
    posC_msg.z = pos_Z;    
    //publishing
    //pub_dPosition.publish(dPos_msg);
    pub_absPosition.publish(posC_msg);
  }

  void rawOriCallback(const geometry_msgs::Point& dROri_msg){
    //message output
    // geometry_msgs::Point dOri_msg;
    geometry_msgs::Point oriC_msg;
    if(reset == 1 ){
        //output computation
      oriC_msg.x   = oriY_ws;
      oriC_msg.y   = oriP_ws;
      oriC_msg.z   = oriR_ws;
  
    }
    else if(reset == 0 ) {
        if(flagCalib == 0 && start_stop == 0){
            //output computation
          oriC_msg.x   = dROri_msg.x  ;
          oriC_msg.y   = dROri_msg.y  ;
          oriC_msg.z   = dROri_msg.z  ;
              //saving previous value
          prev_ori_x   = oriC_msg.x;
          prev_ori_y   = oriC_msg.y;
          prev_ori_z   = oriC_msg.z;
        }
        else {
          oriC_msg.x = prev_ori_x;
          oriC_msg.y = prev_ori_y;
          oriC_msg.z = prev_ori_z;
        }
    }
    //publishing
    pub_absOrientation.publish(oriC_msg);
  }

private:
  ros::NodeHandle nr;
  ros::Publisher pub_dPosition;
  ros::Publisher pub_dOrientation;
  ros::Publisher pub_absOrientation;
  ros::Publisher pub_absPosition;
  ros::Publisher pub_gripper;
  ros::Subscriber sub_dRPos;
  ros::Subscriber sub_dROri;
  ros::Subscriber sub_fs;
  ros::Subscriber sub_ui;
  ros::Subscriber sub_gripper;
  
  uint8_t reset  =        0;
  uint8_t start_stop  =   0;
  float dPos_msg_x =   0;
  float dPos_msg_y = 0;
  float dPos_msg_z = 0;
  //WS variable for Position
  float pos_X     =     posX_ws;
  float pos_Y     =     posY_ws;
  float pos_Z     =     posZ_ws;
  float prev_ori_x=     oriY_ws;
  float prev_ori_y=     oriP_ws;
  float prev_ori_z=     oriR_ws;
  float multi    =        0;
  uint8_t repos =         0;
  uint8_t prevCal=        0; //conditioning the calibration signal
  uint8_t currCal=        0;
  uint8_t flagCalib =     0; //flag calib
}; 
//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "signal_condition_node_L");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish signal_condition_node_L;

  ros::spin();

  return 0;
}
