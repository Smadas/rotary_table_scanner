#ifndef PROJECT_LIDAR_H
#define PROJECT_LIDAR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Lidar {
public:
    // constructor
    Lidar();

    // lidar callback
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ~Lidar();
private:
    // ros node handle
    ros::NodeHandle n_;

    // subscriber
    ros::Subscriber lidar_sub_;

    // publisher
    ros::Publisher lidar_pub_;

    // messages
    sensor_msgs::LaserScan scan_filtered_;

    // scan filter constants
    double stationary_object_tolerance_;
    double top_range_;
    double bottom_range_;
    double max_range_;
};


#endif //PROJECT_LIDAR_H
