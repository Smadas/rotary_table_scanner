#include "rrm_cv5_8_lidar/Lidar.h"

Lidar::Lidar()
{
    // subscriber
    lidar_sub_ = n_.subscribe("scan", 1, &Lidar::lidarCallback, this);

    // publisher
    lidar_pub_ = n_.advertise<sensor_msgs::LaserScan>("rrm_cv5_8_lidar/scan_filtered", 1);

    // init scan filter constants
    top_range_ = -0.3;
    n_.getParam("top_range", top_range_);
    bottom_range_ = -1.5;
    n_.getParam("bottom_range", bottom_range_);
    max_range_ = 10;
    n_.getParam("max_range", max_range_);

}

void Lidar::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
// filter static objects from scan
    scan_filtered_ = *scan_in;
    for (double i = 0.0; i < 360.0; i++)
    {
        //angle in desired range and distance in desired range
        if ((i*scan_filtered_.angle_increment > top_range_)||(i*scan_filtered_.angle_increment < bottom_range_)||(scan_filtered_.ranges[i] > max_range_))
        {
            scan_filtered_.ranges[i] = INFINITY;
            scan_filtered_.intensities[i] = 0;
        }
    }


    lidar_pub_.publish(scan_filtered_);
}

Lidar::~Lidar()
{

}