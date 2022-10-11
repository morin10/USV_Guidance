#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <vector>
#include "std_msgs/Int64.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int64>("cmd_base", 100);
    ros::Rate loop_rate(10);
    std_msgs::Int64 msg;
    int cmd = 0;
    while(ros::ok()){
        ifstream readFile;
        readFile.open("/home/morin/catkin_ws/src/udp_base/txtcontrol/cmd.txt");
        if(readFile.is_open()){
            readFile >> cmd;
            readFile.close();
        }
        msg.data = cmd;
        pub.publish(msg); // Publish message
        loop_rate.sleep();
    }
    return 0;
}