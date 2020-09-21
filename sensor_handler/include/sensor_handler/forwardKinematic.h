#include <eigen3/Eigen/Dense>

#define PI 3.1415926536

#define MAX_JOINT 7
#define MAX_LINK 7
#define LINK1 185
#define LINK2 308
#define LINK3 400
#define EF_HEIGHT 212

#define LIMIT_POS_X 7.5
#define LIMIT_POS_Y 7.5
#define LIMIT_POS_Z 7.5

using Eigen::Matrix4d;

class forwardKinematic
{
    private:
        //v1.0
        // double linkLength[MAX_LINK] = {LINK1, LINK2, LINK3, EF_HEIGHT, 0, 0, 0};
        // double DH_d[MAX_LINK] = {linkLength[0], 0, 0, linkLength[3], 0, 0, 0};
        // double DH_a[MAX_LINK] = {0, linkLength[1], linkLength[2], 0, 0, 0, 0};
        // double DH_alpha[MAX_LINK] = {-PI/2, 0, -PI/2, -PI/2, PI/2, PI/2, PI/2};

        //v2.o
        double linkLength[MAX_LINK] = {LINK1, LINK2, LINK3, EF_HEIGHT, 0, 0, 0};
        double DH_d[MAX_LINK] = {-linkLength[0], 0, 0, -linkLength[3], 0, 0, 0};
        double DH_a[MAX_LINK] = {0, linkLength[1], linkLength[2], 0, 0, 0, 0};
        double DH_alpha[MAX_LINK] = {PI/2, 0, PI/2, -PI/2, PI/2, -PI/2, -PI/2};

        //new type for position and orientation
        struct Point
        {
            double X, Y, Z;
        };
        struct Orientation
        {
            double Yaw, Pitch, Roll;
        };
        //end-effector position coordinate
        Point currentPos, prevPos, dPos, dummyPos, pres_ori, prev_ori, d_ori, threshold_pos;
        //end-effector orientation
        Orientation currentOri, prevOri, dOri;
        
        Matrix4d DH_trans[MAX_LINK];
        

    public:
        forwardKinematic();
        ~forwardKinematic();
        void fkDHCalculation(double *DH_theta);
        void fkGeometricPosition(double *geo_theta);
        void fkOrientation(double *theta);
        double getCurrentX()      {return currentPos.X;};
        double getCurrentY()  	  {return currentPos.Y;};
        double getCurrentZ()      {return currentPos.Z;};
        double getCurrentYaw()    {return currentOri.Yaw;};
        double getCurrentPitch()  {return currentOri.Pitch;};
        double getCurrentRoll()   {return currentOri.Roll;};

        double getOriX()        {return pres_ori.X;};
        double getOriY()  	    {return pres_ori.Y;};
        double getOriZ()        {return pres_ori.Z;};

        double getDummyX()        {return dummyPos.X;};
        double getDummyY()  	    {return dummyPos.Y;};
        double getDummyZ()        {return dummyPos.Z;};

        double getDX()        {return dPos.X;};
        double getDY()        {return dPos.Y;};
        double getDZ()        {return dPos.Z;};
        double getDYaw()      {return dOri.Yaw;};
        double getDPitch()    {return dOri.Pitch;};
        double getDRoll()     {return dOri.Roll;};
};