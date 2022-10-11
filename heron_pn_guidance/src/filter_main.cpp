#include "ublox_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ublox_filter_node");

    ros::NodeHandle nh;
    ros::Rate hz(10);    // 10 Hz

    Ublox_Filter filter;

    // Heron Version
    ros::Subscriber sub_gps = nh.subscribe("/ublox/fix", 1000, &Ublox_Filter::gps_callback, &filter);
    ros::Subscriber sub_vel = nh.subscribe("/ublox/fix_velocity", 1000, &Ublox_Filter::vel_callback, &filter);

    // // KingFisher Version
    // ros::Subscriber sub_gps = nh.subscribe("/ublox_gps/fix", 1000, &Ublox_Filter::gps_callback, &filter);
    // ros::Subscriber sub_vel = nh.subscribe("/ublox_gps/fix_velocity", 1000, &Ublox_Filter::vel_callback, &filter);

    ros::Publisher info_pub = nh.advertise<nav_msgs::Odometry>("/heron_info", 1000);

    while(ros::ok())
    {
        info_pub.publish(filter.own_ship_info);
        ros::spinOnce();

        hz.sleep();
    }



    return 0;
}
