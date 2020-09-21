#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include "geometry_msgs/Point.h"

#define DATA_NUM 5

using namespace std;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
struct Point
{
    double x,y,z;
};
Point data;

void dataCallback(const geometry_msgs::Point& msg)
{
  data.x = msg.x;
  data.y = msg.y;
  data.z = msg.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_picker");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/r_position", 1000, dataCallback);

    data.x=0;
    data.y=0;
    data.z=0;

    string input;
    int count = 0;
    int n_pos_1 = 0;
    int n_pos_2 = 0;

    ofstream outFile3 ("/home/epione/console_ws/src/save_data/data_masuk_pos_1_r.txt");
    if (outFile3.is_open())
    {
        outFile3 << "[Position 1 Data]" << endl;
        outFile3.close();    
    }
    else cout << "Unable to open file";
    ofstream outFile4 ("/home/epione/console_ws/src/save_data/data_masuk_pos_2_r.txt");
    if (outFile4.is_open())
    {
        outFile4 << "[Position 2 Data]" << endl;
        outFile4.close();    
    }
    else cout << "Unable to open file";

    cout << "input 1 for position 1" << endl << "input 2 for position 2" << endl << "input exit to exit" << endl;
    cout << "Press enter to start storing data" << endl;

    while (ros::ok()) {
        // ros::spinOnce();
        cin >> input;
        count++;
        if (input== "exit") {
            cout << "finished taking data" << endl;
            break;
        }
        if (atoi(input.c_str())==1) {
            n_pos_1++;
            ofstream outFile ("/home/epione/console_ws/src/save_data/data_masuk_pos_1.txt", ios::app);
            if (outFile.is_open())
            {
                outFile << "[Data " << n_pos_1 << "]" << endl;
                int data_taken = 1;
                while (data_taken<(DATA_NUM+1)) {
                    ros::spinOnce();
                    outFile << "x: " << data.x << endl;
                    outFile << "y: " << data.y << endl;
                    outFile << "z: " << data.z << endl;
                    outFile << "------" << endl;
                    data_taken++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                outFile << endl;
                outFile.close();
                cout << "pos 1 stored data : " << n_pos_1 << endl;
            }
            else cout << "Unable to open file";
        }
        else if (atoi(input.c_str())==2) {
            n_pos_2++;
            ofstream outFile2 ("/home/epione/console_ws/src/save_data/data_masuk_pos_2.txt", ios::app);
            if (outFile2.is_open())
            {
                outFile2 << "[Data " << n_pos_2 << "]" << endl;
                int data_taken = 1;
                while (data_taken<(DATA_NUM+1)) {
                    ros::spinOnce();
                    outFile2 << "x: " << data.x << endl;
                    outFile2 << "y: " << data.y << endl;
                    outFile2 << "z: " << data.z << endl;
                    outFile2 << "------" << endl;
                    data_taken++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                outFile2 << endl;
                outFile2.close();
                cout << "pos 2 stored data : " << n_pos_2 << endl;
            }
            else cout << "Unable to open file";
        }
        else {
            cout << "please input 1 or 2" << endl;
        }
        
    }

     return 0;
}