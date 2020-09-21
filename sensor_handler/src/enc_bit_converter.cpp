#include "ros/ros.h"
#include "sensor_handler/sensorData.h"

void sensorCb(const sensor_handler::sensorData::ConstPtr& msg)
{
  uint16_t raw_pos;
  double angle_deg;

  for (int i=0; i<7; i++) {
    printf("[%d] : ", i);
    raw_pos = msg->enc_position[i];
    if (raw_pos > 16384) {
      printf("off | ");
    }
    else {
      angle_deg = (double)raw_pos * ((double)360/(double)16384);
      printf("%.2f | ", angle_deg);
    }
  }
  printf("\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "enc_bit_converter");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("r_arm_data_raw", 10, sensorCb);

  ros::spin();

  return 0;
}
