#include "ros/ros.h"
#include "sensor_handler/sensorInstruction.h"
#include <bits/stdc++.h> 

using namespace std; 

int main(int argc, char **argv)
{
  int input_length;
  input_length = argc;
  long int input_val;
  uint8_t calib_id[7] = {0, 0, 0, 0, 0, 0, 0};
  uint8_t calibration_pkg = 1;
  
  /* ROS initialization */
  ros::init(argc, argv, "encoderCalibrator");
  ros::NodeHandle n;
  ros::Publisher encInst_pub = n.advertise<sensor_handler::sensorInstruction>("r_arm_instruction", 30);
  ros::Rate loop_rate(1);
  

  if (input_length<2) {
    ROS_INFO("Please input id for calibration");
    return 1;
  }
  else {
    for (int i=1; i<input_length; i++) {
        input_val = atol(argv[i]);
        if ((input_val > 0) && (input_val < 8)) {
            calib_id[input_val-1] = 1;
        }
        else {
            cout <<"invalid id number : " << input_val << "\n";
            return 1;
        }        
    }
  }
//   printf("calib_id : ");
  for (int i=0; i<7; i++) {
    //   printf("%d ", calib_id[i]);
      if (calib_id[i] == 1) {
        calibration_pkg |= (1<<(i+1));
      }
  }
//   printf("\ncalibration package : 0x%x\n", calibration_pkg);

  while (ros::ok())
  {
    sensor_handler::sensorInstruction encInst_msg;

    printf("calibration_pkg : 0x%x\n", calibration_pkg);
    encInst_msg.inst_pkg = calibration_pkg;

    encInst_pub.publish(encInst_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
