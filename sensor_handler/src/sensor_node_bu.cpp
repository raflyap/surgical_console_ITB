#include <ros/ros.h>
#include <iostream>
#include <stdint.h>
#include "sensor_handler/sensorData.h"

#define PI 3.1415926536

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_arm_data = n_.advertise<sensor_handler::sensorData>("r_arm_data", 100);

    //Topic you want to subscribe
    sub_arm = n_.subscribe("r_arm_data_raw", 30, &SubscribeAndPublish::armCallback, this);
  }

  void armCallback(const sensor_handler::sensorData& raw_arm_msg)
  {
    sensor_handler::sensorData arm_msg;    
    uint16_t raw_pos;
    double   converted_pos;
    bool enc_dc_detect = false;

    for (int i=0; i<7; i++) {
        raw_pos = raw_arm_msg.enc_position[i];
        if (raw_pos > 16384) {
          enc_dc_detect = true;
        }
        else {
          // converted_pos = (double)raw_pos * 2*PI/(double)16384;
          if (raw_pos==0 || i == 2) {
            converted_pos = raw_pos;
          }
          else {
            converted_pos = 16384 - raw_pos;
          }
          arm_msg.enc_position[i] = converted_pos;
        }
    }
  
    if (enc_dc_detect == false) {
        pub_arm_data.publish(arm_msg);
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_arm_data;
  ros::Subscriber sub_arm;

}; //End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "sensor_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish kinematic_node;

  ros::spin();

  return 0;
}