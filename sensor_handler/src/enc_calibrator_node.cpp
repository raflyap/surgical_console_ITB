#include <ros/ros.h>
#include <iostream>
#include <stdint.h>
#include "std_msgs/UInt8.h"
#include "data_handler/ui_msgs.h"


#define PI 3.1415926536

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_enc_inst_r = n_.advertise<std_msgs::UInt8>("r_arm_instruction", 100);
    pub_enc_inst_l = n_.advertise<std_msgs::UInt8>("l_arm_instruction", 100);

    //Topic you want to subscribe
    sub_ui_msgs = n_.subscribe("UI", 30, &SubscribeAndPublish::uiCallback, this);
  }

  void uiCallback(const data_handler::ui_msgs& ui_msg)
  {
    std_msgs::UInt8 enc_msg;

    if (ui_msg.calibArm == 1) {
      enc_msg.data = ui_msg.calibID;
      pub_enc_inst_l.publish(enc_msg);
    }
    else {
      enc_msg.data = ui_msg.calibID;
      pub_enc_inst_r.publish(enc_msg);
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_enc_inst_r;
  ros::Publisher pub_enc_inst_l;
  ros::Subscriber sub_ui_msgs;

}; //End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "enc_calibrator");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish enc_calib;

  ros::spin();

  return 0;
}