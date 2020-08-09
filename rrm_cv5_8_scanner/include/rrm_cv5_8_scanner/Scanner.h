#ifndef PROJECT_SCANNER_H
#define PROJECT_SCANNER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "rrm_cv5_8_motor/MotorState.h"
#include "rrm_cv5_8_scanner/StartScan.h"
#include "rrm_cv5_8_motor/SetStart.h"
#include "rrm_cv5_8_motor/SetStop.h"
#include "rrm_cv5_8_motor/SetPeriod.h"

#include <vector>

class Scanner {
public:
    // Constructor
    Scanner();

    // callbacks
    void motorStateCallback(const rrm_cv5_8_motor::MotorState::ConstPtr& msg);
    bool startScan(rrm_cv5_8_scanner::StartScan::Request &req, rrm_cv5_8_scanner::StartScan::Response &res);

    // methods
    void executeScanning();

    // Destructor
    ~Scanner();

private:

    // Publisher
    ros::Publisher motor_velocity_pub_;

    // Subscriber
    ros::Subscriber motor_state_sub_;

    // Service
    ros::ServiceServer service_;

    // variables scanning process
    bool scan_started_;
    bool scan_initialized_;
    double scan_angle_range_; //goal range
    double initial_plate_angle_;
    double plate_angle_;
    double plate_angle_prev_;
    double scanned_angle_;
    geometry_msgs::Twist motor_velocity_;

};


#endif //PROJECT_SCANNER_H
