#include "rrm_cv5_8_scanner/LaserScanToPointCloud.h"

LaserScanToPointCloud::LaserScanToPointCloud() :
        laser_sub_(n_, "rrm_cv5_8_lidar/scan_filtered", 1),
        laser_notifier_(laser_sub_,listener_, "/base_link", 1)
{
    ROS_INFO("Initializing scan listener");

    //init point cloud
    model_cloud_.header.stamp = ros::Time::now();
    model_cloud_.header.frame_id = "base_link";

    //init point cloud
    model_cloud_object_frame_.header.stamp = ros::Time::now();
    model_cloud_object_frame_.header.frame_id = "scan_object";

    // init laser scan transformation
    laser_notifier_.registerCallback(
            boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(TRANSFORM_TIME_TOLERANCE));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("scanned_model_cloud", 1);
    transform_wait_ = 0.002;
    n_.getParam("transform_wait", transform_wait_);
}

void LaserScanToPointCloud::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    // wait for scan to scan_object transform
    try {
        listener_.waitForTransform("/scan", "/scan_object",
                                   ros::Time::now(), ros::Duration(ros::Duration(transform_wait_)));
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    // project laser scan to 3D pointcloud
    sensor_msgs::PointCloud2 cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
                "scan_object", *scan_in, cloud, listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    // Append new scan to cloud
    pcl::concatenatePointCloud(model_cloud_, cloud, model_cloud_);

    //transform scan to scan_object frame
    //pcl_ros::transformPointCloud("/scan_object", model_cloud_, model_cloud_object_frame_, listener_);

    //publish scanned object in scan_object frame
    scan_pub_.publish(model_cloud_);
}