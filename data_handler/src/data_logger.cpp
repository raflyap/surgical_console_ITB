#include <ros/ros.h>
#include "std_msgs/UInt8.h"
#include <chrono>
#include <fstream>
#include <ctime>
#include <string>
#include <iostream>

using namespace std;
class SubscribeData
{
    public:
    SubscribeData()
    {

        sub_r_sensor = nr.subscribe("r_flag_sensor", 100, &SubscribeData::callback_r_sensor, this);
        sub_r_client = nr.subscribe("r_flag_client", 100, &SubscribeData::callback_r_client, this);
        sub_r_colls = nr.subscribe("r_flag_colls", 100, &SubscribeData::callback_r_colls, this);
        
        sub_l_sensor = nr.subscribe("l_flag_sensor", 100, &SubscribeData::callback_l_sensor, this);
        sub_l_client = nr.subscribe("l_flag_client", 100, &SubscribeData::callback_l_client, this);
        sub_l_colls = nr.subscribe("l_flag_colls", 100, &SubscribeData::callback_l_colls, this);
    }
    bool fileValidation(std::string path){
        int count = 0;
        int valid;
        string line;
        ifstream file(path);
        while (getline(file,line))
            count++;
        if (count <= 30000)
            valid = 1;
        else
            valid = 0;
        return valid;
    }

    std::string getTime(){
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(buffer,sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
        std::string header(buffer);
        return(header);
    }

    bool* detectEncoder(std_msgs::UInt8 data){
        bool* en_enc = new bool[7];
        uint8_t enc_data =  data.data;
        for (int i=0; i<7; i++){
            if (enc_data & (1<<i))
                en_enc[i] = true;
            else
                en_enc[i] = false;
        }
        return en_enc;
    }
    void callback_r_sensor(const std_msgs::UInt8& data){
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        double diff = (now - last_err_r).count()/1000000000;
        if (data.data > 0 && first_r_sensor == 1 && ( diff >= 5.0)){
            cnt_r_sensor++;
            std::string path ("/home/epione/console_ws/src/data_handler/log/log.log");
            std::string header = getTime();
            bool valid = fileValidation(path);
            if (data.data != state_r_error){
                state_r_error = data.data;
                cnt_r_sensor = 0;
            }
            if (cnt_r_sensor >= 50){
                first_r_sensor = 0;
                ofstream log;
                if (!valid){
                    log.open (path, ios::out);
                    log<< "Last 30000 line of data has been erased" << endl << endl;
                }else
                log.open (path, ios::out | ios::app);
                bool* enc_num = detectEncoder(data);
                if (log.is_open())
                {
                    log << header << endl;
                    log << "SENSOR R : List of this right encoder is not readable:" << endl;
                    log << " Right Encoder : ";
                    for(int i=0;i<7;i++){
                        if (enc_num[i] == 1)
                            log << i+1 << " ";
                    }
                    log << endl;
                    log << "<---------------------------------------------<" << endl;
                    log.close();
                }
                last_err_r = std::chrono::system_clock::now();
            }

        }
        else if (data.data == 0 && first_r_sensor == 0){
            first_r_sensor = 1;
        }
    }

    void callback_r_client(const std_msgs::UInt8& data){
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        double diff = (now - last_client_r).count()/1000000000;
        if (data.data > 0 && first_r_client == 1 && ( diff >= 5.0)){
            first_r_client = 0;
            std::string path ("/home/epione/console_ws/src/data_handler/log/log.log");
            std::string header = getTime();
            bool valid = fileValidation(path);
            ofstream log;
            if (!valid){
                log.open (path, ios::out);
                log<< "Last 30000 line of data has been erased" << endl << endl;
            }else
            log.open (path, ios::out | ios::app);
            if (log.is_open())
            {
                log << header << endl;
                log << "SOCKET R : Right hand server is not reachable" << endl;
                log << "<---------------------------------------------<" << endl;
                log.close();
            }
            last_client_r = std::chrono::system_clock::now();
        }
        else if (data.data == 0 && first_r_client == 0){
            first_r_client = 1;
        }
    }
    void callback_r_colls(const std_msgs::UInt8& data){
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        double diff = (now - last_colls_r).count()/1000000000;
        if (data.data > 0 && first_r_colls == 1 && ( diff >= 5.0)){
            first_r_colls = 0;
            std::string path ("/home/epione/console_ws/src/data_handler/log/log.log");
            std::string header = getTime();
            bool valid = fileValidation(path);
            ofstream log;
            if (!valid){
                log.open (path, ios::out);
                log<< "Last 30000 line of data has been erased" << endl << endl;
            }else
            log.open (path, ios::out | ios::app);
            if (log.is_open())
            {
                log << header << endl;
                log << "ACTUATOR R : Right hand actuator is collided with another object/s" << endl;
                log << "<---------------------------------------------<" << endl;
                log.close();
            }
            last_colls_r = std::chrono::system_clock::now();
        }
        else if (data.data == 0 && first_r_colls == 0){
            first_r_colls = 1;
        }
    }
    void callback_l_sensor(const std_msgs::UInt8& data){
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        double diff = (now - last_err_l).count()/1000000000;
        if (data.data > 0 && first_l_sensor == 1 && ( diff >= 5.0)){
            cnt_l_sensor++;
            std::string path ("/home/epione/console_ws/src/data_handler/log/log.log");
            std::string header = getTime();
            bool valid = fileValidation(path);
            if (data.data != state_l_error){
                state_l_error = data.data;
                cnt_l_sensor = 0;
            }
            if (cnt_l_sensor >= 50){
                first_l_sensor = 0;
                ofstream log;
                if (!valid){
                    log.open (path, ios::out);
                    log<< "Last 30000 line of data has been erased" << endl << endl;
                }else
                log.open (path, ios::out | ios::app);
                bool* enc_num = detectEncoder(data);
                if (log.is_open())
                {
                    log << header << endl;
                    log << "SENSOR L : List of this left encoder is not readable:" << endl;
                    log << " Left Encoder : ";
                    for(int i=0;i<7;i++){
                        if (enc_num[i] == 1)
                            log << i+1 << " ";
                    }
                    log << endl;
                    log << "<---------------------------------------------<" << endl;
                    log.close();
                }
                last_err_l = std::chrono::system_clock::now();
            }
        }
        else if (data.data == 0 && first_l_sensor == 0){
            first_l_sensor = 1;
        }
    }
    void callback_l_client(const std_msgs::UInt8& data){
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        double diff = (now - last_client_l).count()/1000000000;
        if (data.data > 0 && first_l_client == 1 && ( diff >= 5.0)){
            first_l_client = 0;
            std::string path ("/home/epione/console_ws/src/data_handler/log/log.log");
            std::string header = getTime();
            bool valid = fileValidation(path);
            ofstream log;
            if (!valid){
                log.open (path, ios::out);
                log<< "Last 30000 line of data has been erased" << endl << endl;
            }else
            log.open (path, ios::out | ios::app);
            if (log.is_open())
            {
                log << header << endl;
                log << "SOCKET L : Left hand server is not reachable" << endl;
                log << "<---------------------------------------------<" << endl;
                log.close();
            }
            last_client_l = std::chrono::system_clock::now();
        }
        else if (data.data == 0 && first_l_client == 0){
            first_l_client = 1;
        }
    }
    void callback_l_colls(const std_msgs::UInt8& data){
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        double diff = (now - last_colls_l).count()/1000000000;
        if (data.data > 0 && first_l_colls== 1 && ( diff >= 5.0)){
            first_l_colls = 0;
            std::string path ("/home/epione/console_ws/src/data_handler/log/log.log");
            std::string header = getTime();
            bool valid = fileValidation(path);
            ofstream log;
            if (!valid){
                log.open (path, ios::out);
                log<< "Last 30000 line of data has been erased" << endl << endl;
            }else
            log.open (path, ios::out | ios::app);
            if (log.is_open())
            {
                log << header << endl;
                log << "ACTUATOR L : Left hand actuator is collided with another object/s" << endl;
                log << "<---------------------------------------------<" << endl;
                log.close();
            }
            last_colls_l = std::chrono::system_clock::now();
        }
        else if (data.data == 0 && first_l_colls == 0){
            first_l_colls = 1;
        }
    }
    private:
        ros::NodeHandle nr;
        ros::Subscriber sub_r_sensor;
        ros::Subscriber sub_r_client;
        ros::Subscriber sub_r_colls;
        ros::Subscriber sub_l_sensor;
        ros::Subscriber sub_l_client;
        ros::Subscriber sub_l_colls;

        uint8_t  first_r_sensor = 1;
        uint8_t  first_r_client = 1;
        uint8_t  first_r_colls = 1;
        uint8_t  first_l_sensor = 1;
        uint8_t  first_l_client = 1;
        uint8_t  first_l_colls = 1;
        uint8_t  cnt_r_sensor = 0;
        uint8_t  cnt_l_sensor = 0;
        uint8_t  state_r_error = 0;
        uint8_t  state_l_error = 0;

        std::chrono::time_point<std::chrono::system_clock> last_err_r = std::chrono::system_clock::now();
        std::chrono::time_point<std::chrono::system_clock> last_client_r = std::chrono::system_clock::now();
        std::chrono::time_point<std::chrono::system_clock> last_colls_r = std::chrono::system_clock::now();
        std::chrono::time_point<std::chrono::system_clock> last_err_l = std::chrono::system_clock::now();
        std::chrono::time_point<std::chrono::system_clock> last_client_l = std::chrono::system_clock::now();
        std::chrono::time_point<std::chrono::system_clock> last_colls_l = std::chrono::system_clock::now();
    
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_logger");

    SubscribeData data_logger;

    ros::spin();
    return 0;
}
