#include <ros/ros.h>
#include <iostream>
#include <stdint.h>
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include "sensor_handler/sensorData.h"
#include "sensor_handler/Float64ArrayLength7.h"
#include "sensor_handler/gripperLimit.h"


#define PI 3.1415926536
#define ENC_RES 14

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_enc_data = n_.advertise<sensor_handler::Float64ArrayLength7>("r_enc_data", 100);
    pub_gripper_data = n_.advertise<std_msgs::Float64>("r_grip_data_raw", 100);
    pub_flag_error = n_.advertise<std_msgs::UInt8>("r_flag_sensor",100);
    //Topic you want to subscribe
    sub_arm = n_.subscribe("r_arm_data", 30, &SubscribeAndPublish::armCallback, this);
    sub_grip_limit = n_.subscribe("r_gripper_limit", 30, &SubscribeAndPublish::grip_lim_Callback, this);
  }

  void initiate_grip_lim()
  {
    _open_lim = 800;
    _close_lim = 700;
  }
  void grip_lim_Callback(const sensor_handler::gripperLimit& gripper_lim_msg)
  {
    _open_lim = gripper_lim_msg.open_limit;
    _close_lim = gripper_lim_msg.close_limit;
  }

  void armCallback(const sensor_handler::sensorData& raw_arm_msg)
  {
    ROS_INFO("Right Arm Sensor Time");
    sensor_handler::Float64ArrayLength7 enc_msg;
    std_msgs::Float64 gripper_msg;
    std_msgs::UInt8 err_flag;
    uint16_t raw_pos;
    uint16_t raw_grip;
    double   converted_pos;
    double   norm_grip;
    uint8_t enc_dc_detect = 0;

    for (int i=0; i<7; i++) {
        raw_pos = raw_arm_msg.enc_position[i];
        if (raw_pos > pow(2,ENC_RES)) {
          enc_dc_detect |= (1<<i);
        }
        else {
          // converted_pos = (double)raw_pos * 2*PI/(double)16384;
          if (i == 2) {
            if (raw_pos ==0) {
              converted_pos = 0;
            }
            else {
              converted_pos = pow(2, ENC_RES) - raw_pos;
            }
          }
          else {
            converted_pos = raw_pos;
          }
          converted_pos = converted_pos*2*PI/(double)pow(2, ENC_RES);
          enc_msg.data[i] = converted_pos;
        }
    }

    /* Normalizing grip data */
    raw_grip = raw_arm_msg.gripper_data;
    if (_open_lim>_close_lim) {
      if (raw_grip > _open_lim) {
        raw_grip = _open_lim;
      }
      else if (raw_grip < _close_lim) {
        raw_grip = _close_lim;
      }
    }
    else if (_open_lim<_close_lim) {
      if (raw_grip < _open_lim) {
        raw_grip = _open_lim;
      }
      else if (raw_grip > _close_lim) {
        raw_grip = _close_lim;
      }
    }
    norm_grip = (double) ((_close_lim - raw_grip) * 30) / (double)(_close_lim - _open_lim);
    gripper_msg.data = norm_grip;

    /* Publish data if all encoder online */
    if (enc_dc_detect == 0) {
      err_flag.data = 0;
      pub_flag_error.publish(err_flag);
      pub_enc_data.publish(enc_msg);
      pub_gripper_data.publish(gripper_msg);

    }
    err_flag.data = enc_dc_detect;
    pub_flag_error.publish(err_flag);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher  pub_enc_data;
  ros::Publisher  pub_gripper_data;
  ros::Publisher  pub_flag_error;
  ros::Subscriber sub_arm;
  ros::Subscriber sub_grip_limit;

  uint16_t _open_lim;
  uint16_t _close_lim;

}; //End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "sensor_node_r");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish sensor_node;
  sensor_node.initiate_grip_lim();

  ros::spin();

  return 0;
}