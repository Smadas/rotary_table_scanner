#include "rrm_cv5_8_scanner/Scanner.h"
#include "rrm_cv5_8_scanner/LaserScanToPointCloud.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner");
    ros::NodeHandle n;

    //init scanner
    Scanner scanner;

    //init laser listener
    LaserScanToPointCloud laserScanToPointCloud;

    // init loop rate
    int velocity_publish_rate = 10;
    n.getParam("velocity_publish_rate", velocity_publish_rate);
    ros::Rate loop_rate(velocity_publish_rate);

    // execute scanning
    while(ros::ok())
    {
        scanner.executeScanning();
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}