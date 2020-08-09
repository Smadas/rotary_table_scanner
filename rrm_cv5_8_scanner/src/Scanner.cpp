#include "rrm_cv5_8_scanner/Scanner.h"

Scanner::Scanner()
{
    // NodeHandler
    ros::NodeHandle n;

    // topic names
    std::string motor_velocity_topic = "rrm_cv5_8_motor/cmd_vel";
    std::string motor_state_topic = "rrm_cv5_8_motor/motor_state";
    n.getParam("motor_velocity_topic", motor_velocity_topic);
    n.getParam("motor_state_topic", motor_state_topic);

    // Publisher
    motor_velocity_pub_ = n.advertise<geometry_msgs::Twist>(motor_velocity_topic, 1);

    // Subscriber
    motor_state_sub_ = n.subscribe<rrm_cv5_8_motor::MotorState>(motor_state_topic, 1, &Scanner::motorStateCallback, this);

    // advertise service
    service_ = n.advertiseService("/rrm_cv5_8_scanner/start_scan", &Scanner::startScan, this);

    // init variables
    scan_started_ = false;
    scan_angle_range_ = 6;
    scan_initialized_ = false;
    initial_plate_angle_ = 0;
    plate_angle_ = 0;
    plate_angle_prev_ = 0;
    scanned_angle_ = 0;
    motor_velocity_.angular.z = 0.05;
    n.getParam("motor_velocity", motor_velocity_.angular.z);

}
void Scanner::motorStateCallback(const rrm_cv5_8_motor::MotorState::ConstPtr& msg)
{
    plate_angle_ = msg->position;
}

bool Scanner::startScan(rrm_cv5_8_scanner::StartScan::Request &req, rrm_cv5_8_scanner::StartScan::Response &res)
{
    if (scan_started_)
    {
        res.started_scan = false;
        ROS_INFO("Scan in progress!");
    }
    else
    {
        scan_angle_range_ = req.scan_angle_range;
        scan_started_ = true;
        res.started_scan = true;
    }
    return res.started_scan;
}

void Scanner::executeScanning()
{
    if (scan_started_)
    {
        if (scan_initialized_)
        {
            // publish velocity to motor
            motor_velocity_.linear.x += 0.01;
            motor_velocity_pub_.publish(motor_velocity_);

            // check if scanning is finished
            bool position_condition_met = false;
            if (fabs(plate_angle_ - plate_angle_prev_) > 300)
            {
                scanned_angle_ += fabs(fabs(plate_angle_ - plate_angle_prev_) - 360);
            }
            else
            {
                scanned_angle_ += fabs(plate_angle_ - plate_angle_prev_);
            }

            ROS_INFO("Progress - scanned angle: %lf", scanned_angle_);
            if (scanned_angle_ >= scan_angle_range_)
            {
                position_condition_met = true;
            }
            plate_angle_prev_ = plate_angle_;

            // end scanning
            if (position_condition_met)
            {
                // reset flags
                scan_initialized_ = false;
                scan_started_ = false;
                position_condition_met = false;
                scanned_angle_ = 0;

                ROS_INFO("Scan complete!");
            }
        }
        else
        {
            // remember initial plate angle
            initial_plate_angle_ = plate_angle_;
            plate_angle_prev_ = plate_angle_;

            scan_initialized_ = true;
        }
    }
}

Scanner::~Scanner()
{

}