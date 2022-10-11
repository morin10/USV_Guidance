#ifndef UBLOX_FILTER
#define UBLOX_FILTER

#include "ros/ros.h"
#include <vector>
// #include <math.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"


class Ublox_Filter
{
public:
    Ublox_Filter();
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msgs);
    void vel_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msgs);

	nav_msgs::Odometry own_ship_info;

private:

    void lla2utm(double latitude, double longitude, double height);

    std::vector<double> utm_;

    float prev_x = 0.0;
    float prev_y = 0.0;
    float prev_v_x = 0.0;
    float prev_v_y = 0.0;

};


#endif
