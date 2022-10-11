#include "ublox_filter.h"
#include <cmath>

Ublox_Filter::Ublox_Filter(){};

void Ublox_Filter::vel_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msgs)
{
    //filter code here
    /////////
    //filter code here

    own_ship_info.twist.twist.linear.x = msgs->twist.twist.linear.x;
    own_ship_info.twist.twist.linear.y = msgs->twist.twist.linear.y;
}

void Ublox_Filter::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msgs)
{
    int zone_ = 52;

	// Naive Filtering
    if (msgs->latitude == 0.0 && msgs->longitude == 0.0 && msgs->altitude == 0.0)
    {
        ROS_INFO("The GPS data is not valid | Case 0: Zero Values from GPS");
        utm_.clear();
    }
    else if (msgs->longitude > ((zone_ * 6) - 180) || msgs->longitude < (((zone_ * 6) - 180) - 6))
    { // Check UTM Zone 52
        ROS_INFO("The GPS data is not valid | Case 1: Measurement out of UTM Zone");
        utm_.clear();
    }
    else    // It has no error
    {
        //filter code here
        /////////
        //filter code here
        lla2utm(msgs->latitude, msgs->longitude, msgs->altitude);

        own_ship_info.pose.pose.position.x = utm_[0];
        own_ship_info.pose.pose.position.y = utm_[1];

        utm_.clear();
    }
}


void Ublox_Filter::lla2utm(double latitude, double longitude, double height)
{
    const double kNN_ = 0;
    const double kNS_ = 10000000.0;
    const double kE0_ = 500000.0;
    const double kPI_ = 3.14159265359;

    double east, north;

    double dLat = latitude * kPI_/180;
    double dLon = longitude * kPI_/180;

    double lon0_f = floor(longitude/6)*6+3;
    double lon0 = lon0_f*kPI_/180;
    double k0 = 0.9996;

    double FE = 500000;
    double FN = (latitude < 0) ? 10000000 : 0;

    double Wa = 6378137;
    double Weps = 0.006739496742333;
    double We = 0.081819190842965;

    double WN = Wa/sqrt(1-pow(We,2)*pow(sin(dLat),2));
    double WT = pow(tan(dLat),2);
    double WC = (pow(We,2)/(1-pow(We,2)))*pow(cos(dLat),2);
    double WLA = (dLon - lon0)*cos(dLat);

    double WM = (Wa*((1-pow(We,2)/4 - 3*pow(We,4)/64 - 5*pow(We,6)/256)*dLat-(3*pow(We,2)/8 + 3*pow(We,4)/32 + 45*pow(We,6)/1024)*sin(2*dLat)+(15*pow(We,4)/256 + 45*pow(We,6)/1024)*sin(4*dLat) - (35*pow(We,6)/3072)*sin(6*dLat)));


    east = (FE + k0*WN*(WLA + (1-WT+WC)*pow(WLA,3)/6 + (5-18*WT + pow(WT,2) + 72*WC - 58*Weps)*pow(WLA,5)/120));
    north =(FN + k0*WM + k0*WN*tan(dLat)*(pow(WLA,2)/2 + (5-WT + 9*WC + 4*pow(WC,2))*pow(WLA,4)/24 + (61 - 58*WT + pow(WT,2) + 600*WC - 330*Weps)*pow(WLA,6)/720));


    utm_.push_back(east);
    utm_.push_back(north);
    // utm_.push_back(height);
}









