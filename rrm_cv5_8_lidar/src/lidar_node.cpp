#include "ros/ros.h"
#include "rrm_cv5_8_lidar/Lidar.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar");

    Lidar lidar;

    ros::spin();

    return 0;
}