#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <vector>
#include "udp_base/wplist.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpl_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<udp_base::wplist>("wplist", 100);
    ros::Rate loop_rate(10);
    udp_base::wplist msg;
    double wpl[4][80];
    /*wplist txt file parsing*/
    int i = 0;
    int id = 0;
    while(ros::ok()){
        ifstream readFile;
        readFile.open("/home/nvidia/catkin_ws/src/udp_base/txtcontrol/wplist.txt");
        if(readFile.is_open()){
            readFile >> id;
            while(!readFile.eof()){
                readFile >> wpl[0][i];
                readFile >> wpl[1][i];
                readFile >> wpl[2][i];
                readFile >> wpl[3][i];
                i++;
            }
            readFile.close();
        }
        vector<double> x;
        vector<double> y;
        vector<double> z;
        vector<double> s;
        for(int j = 0; j<i; j++){
            x.push_back(wpl[0][j]);
            y.push_back(wpl[1][j]);
            z.push_back(wpl[2][j]);
            s.push_back(wpl[3][j]);
        }
        msg.x = x;
        msg.y = y;
        msg.z = z;
        msg.s = s;
        msg.size = i;
        msg.id = id;
        i = 0;
        pub.publish(msg); // Publish message
        loop_rate.sleep();
    }
    return 0;
}
