#include <ros/ros.h>
#include <iostream>
#include <stdint.h>
#include "geometry_msgs/Point.h"
#include "sensor_handler/Float64ArrayLength7.h"
#include "../include/sensor_handler/forwardKinematic.h"

class SubscribeAndPublish
{
public:
  forwardKinematic fk;
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_dRawPosition = n_.advertise<geometry_msgs::Point>("r_dPosition_raw", 100);
    pub_dRawOrientation = n_.advertise<geometry_msgs::Point>("r_dOrientation_raw", 100);

    //Topic you want to subscribe
    sub_joint = n_.subscribe("r_enc_data", 30, &SubscribeAndPublish::jointCallback, this);
  }

  void jointCallback(const sensor_handler::Float64ArrayLength7& joint_msg)
  {
    double pres_pos[MAX_JOINT], pos_theta[MAX_JOINT], or_theta[MAX_JOINT];
    int norm_pos[MAX_JOINT];
    double pos_deg[MAX_JOINT];             // present position in degrees
    double *theta;
    double or_x, or_y, or_z;

    geometry_msgs::Point pos_msg;
    geometry_msgs::Point ori_msg;
    
    //---------Kinematic computation---------//
    //collecting input data and converting into radian
    for (int i=0; i<MAX_JOINT; i++) {
      pres_pos[i] = joint_msg.data[i];
      // pres_pos[i] = pres_pos[i]*2*PI/(double)16384;
    }
    for (int i=0; i<MAX_JOINT;i++) {
      theta=pres_pos;
    }
    //calculating forward kinematic using DH method
    fk.fkDHCalculation(theta);
    //extracting position and orientation changes
    pos_msg.x = fk.getDX();//getCurrentX();
    pos_msg.y = fk.getDY();//getCurrentY();
    pos_msg.z = fk.getDZ();//getCurrentZ();
    // pos_msg.x = fk.getCurrentX();
    // pos_msg.y = fk.getCurrentY();
    // pos_msg.z = fk.getCurrentZ();
    or_x = fk.getOriX();
    or_y = fk.getOriY();
    or_z = fk.getOriZ();
    // or_x *= 360/(2*PI);
    // or_y *= 360/(2*PI);
    // or_z *= 360/(2*PI);
    ori_msg.x = or_x;
    ori_msg.y = or_y;
    ori_msg.z = or_z;

    //---------Publishing position and orientation data---------//
    pub_dRawPosition.publish(pos_msg);
    pub_dRawOrientation.publish(ori_msg);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher  pub_dRawPosition;
  ros::Publisher  pub_dRawOrientation;
  ros::Subscriber sub_joint;

}; //End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "fk_node_r");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish fk_node;

  ros::spin();

  return 0;
}