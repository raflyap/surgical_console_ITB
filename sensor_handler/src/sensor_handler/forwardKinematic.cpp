#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include "forwardKinematic.h"


forwardKinematic::forwardKinematic()
{
  currentPos.X = 0;
  currentPos.Y = 0;
  currentPos.Z = 0;
  prevPos.X = 0;
  prevPos.Y = 0;
  prevPos.Z = 0;
  pres_ori.X = 0;
  pres_ori.Y = 0;
  pres_ori.Z = 0;
  currentOri.Pitch = 0;
  currentOri.Roll = 0;
  currentOri.Yaw = 0;
  prevOri.Pitch = 0;
  prevOri.Roll = 0;
  prevOri.Yaw = 0;
  dummyPos.X=100;
  dummyPos.Y=100;
  dummyPos.Z=100;
  threshold_pos.X=LIMIT_POS_X;
  threshold_pos.Y=LIMIT_POS_Y;
  threshold_pos.Z=LIMIT_POS_Z;
}

forwardKinematic::~forwardKinematic()
{
}

void forwardKinematic::fkDHCalculation(double *DH_theta)
/*function : calculate forwardKinematic of the console using Denavit Hartenberg method
  Input : joint angle in radian
  Output : XYZ position in mm (class variable) */
{
  Matrix4d DH_finalTrans = Matrix4d::Identity(4, 4);
  
  DH_theta[1] -= (PI/2.0);
  DH_theta[2] += (PI/2.0);
  DH_theta[5] += (PI/2.0);
  /*-------Calculate transformation matrix------*/
  for (int i=0; i<MAX_LINK; i++) {
    DH_trans[i] << cos(DH_theta[i]), -sin(DH_theta[i])*cos(DH_alpha[i]), sin(DH_theta[i])*sin(DH_alpha[i]), DH_a[i]*cos(DH_theta[i]),
                   sin(DH_theta[i]), cos(DH_theta[i])*cos(DH_alpha[i]), -cos(DH_theta[i])*sin(DH_alpha[i]), DH_a[i]*sin(DH_theta[i]),
                   0, sin(DH_alpha[i]), cos(DH_alpha[i]), DH_d[i],
                   0, 0, 0, 1;
  }

  /*------Calculate FK matrix------*/
  for (int i=0; i<MAX_LINK; i++) {
    DH_finalTrans *= DH_trans[i];
  }

  currentPos.X = DH_finalTrans(0,3);
  currentPos.Y = DH_finalTrans(1,3);
  currentPos.Z = DH_finalTrans(2,3);

  //factor euler angles in sequence : RxRyRz (z->y->x)
  pres_ori.Y = asin(DH_finalTrans(0,2));
  pres_ori.Y *= 360/(2*PI);
  if (pres_ori.Y == 90 || pres_ori.Y == -90) {
    pres_ori.Y = prev_ori.Y;
    pres_ori.X = prev_ori.X;
    pres_ori.Z = prev_ori.Z;
  }
  else {
    pres_ori.X = atan2(-DH_finalTrans(1,2), DH_finalTrans(2,2));
    pres_ori.Z = atan2(-DH_finalTrans(0,1), DH_finalTrans(0,0));
    pres_ori.X *= 360/(2*PI);
    pres_ori.Z *= 360/(2*PI);
  }

  //calculate displacement
  dPos.X = currentPos.X-prevPos.X;
  dPos.Y = currentPos.Y-prevPos.Y;
  dPos.Z = currentPos.Z-prevPos.Z;
  prevPos.Y = currentPos.Y;
  prevPos.X = currentPos.X;  
  prevPos.Z = currentPos.Z;
  
  prev_ori.X = pres_ori.X;
  prev_ori.Y = pres_ori.Y;
  prev_ori.Z = pres_ori.Z;

  //orientation displacement based without considering movement from -pi to pi
  // d_ori.X = pres_ori.X - prev_ori.X;
  // d_ori.X = pres_ori.Y - prev_ori.Y;
  // d_ori.Z = pres_ori.Z - prev_ori.Z;
  // prev_ori.X = pres_ori.X;
  // prev_ori.Y = pres_ori.Y;
  // prev_ori.Z = pres_ori.Z;
  
  if ((dPos.X>threshold_pos.X)||(dPos.Y>threshold_pos.Y)||(dPos.Z>threshold_pos.Z) || (dPos.X<-threshold_pos.X)||(dPos.Y<-threshold_pos.Y)||(dPos.Z<-threshold_pos.Z)) {
    std::cout << "Failsafe activated" << std::endl;
    dPos.X = 0;
    dPos.Y = 0;
    dPos.Z = 0;
  }

      // std::cout << "Matriks DH :" << std::endl;
      // for (int i=0;i<4;i++) {
      //   for (int j=0;j<4;j++) {
      //     std::cout << DH_finalTrans(i,j) << "\t";
      //   }
      //   std::cout << std::endl;
      // }
}

void forwardKinematic::fkGeometricPosition(double *geo_theta)
/*function : calculate forwardKinematic of the console using Denavit Hartenberg method
  Input : joint angle in radian
  Output : XYZ position in mm (class variable) */
{
  double r;
  double norm_geo_theta1 = 2*PI-geo_theta[1];
  double norm_geo_theta2 = geo_theta[2]-PI;

  //calculate position
  r = linkLength[1]*cos(norm_geo_theta1)-linkLength[2]*cos(norm_geo_theta1+norm_geo_theta2)-linkLength[3]*sin(norm_geo_theta1+norm_geo_theta2);
  currentPos.X = r*cos(geo_theta[0]);
  currentPos.Y = r*sin(geo_theta[0]);
  currentPos.Z = linkLength[0]+linkLength[1]*sin(norm_geo_theta1)-linkLength[2]*sin(norm_geo_theta1+norm_geo_theta2)+linkLength[3]*cos(norm_geo_theta1+norm_geo_theta2);
  
  //backup
  // r = linkLength[1]*cos(geo_theta[1])-linkLength[2]*cos(geo_theta[1]+geo_theta[2])-linkLength[3]*sin(geo_theta[1]+geo_theta[2]);
  // currentPos.X = r*cos(geo_theta[0]);
  // currentPos.Y = r*sin(geo_theta[0]);
  // currentPos.Z = linkLength[0]+linkLength[1]*sin(geo_theta[1])-linkLength[2]*sin(geo_theta[1]+geo_theta[2])+linkLength[3]*cos(geo_theta[1]+geo_theta[2]);
  
  //calculate displacement
  dPos.X = currentPos.X-prevPos.X;
  dPos.Y = currentPos.Y-prevPos.Y;
  dPos.Z = currentPos.Z-prevPos.Z;
  prevPos.X = currentPos.X;
  prevPos.Y = currentPos.Y;
  prevPos.Z = currentPos.Z;
  if ((dPos.X<100)&&(dPos.Y<100)&&(dPos.Z<100)) {
    dummyPos.X = dummyPos.X + dPos.X;
    dummyPos.Y = dummyPos.Y + dPos.Y;
    dummyPos.Z = dummyPos.Z + dPos.Z;
  }
}

void forwardKinematic::fkOrientation(double *theta)
/*function : calculate forwardKinematic of the console using Denavit Hartenberg method
  Input : joint angle in <input unit>
  Output : YPR orientation in <input unit> (class variable) */
{
  double norm_theta2 = theta[2]-PI;

  //calculate orientation
  currentOri.Yaw = theta[5]+theta[3]-theta[0];
  currentOri.Pitch = theta[4]-360.0-theta[1]-norm_theta2;
  currentOri.Roll = -theta[6];

  //calculate change
  dOri.Yaw = currentOri.Yaw-prevOri.Yaw;
  dOri.Pitch = currentOri.Pitch-prevOri.Pitch;
  dOri.Roll = currentOri.Roll-prevOri.Roll;
  prevOri.Yaw = currentOri.Yaw;
  prevOri.Pitch = currentOri.Pitch;
  prevOri.Roll = currentOri.Roll;
}