#ifndef PROJECT_MOTORDRIVER_H
#define PROJECT_MOTORDRIVER_H

#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "rrm_cv5_8_motor/SetDirection.h"
#include "rrm_cv5_8_motor/SetPeriod.h"
#include "rrm_cv5_8_motor/SetStart.h"
#include "rrm_cv5_8_motor/SetStop.h"
#include "rrm_cv5_8_motor/Acknowledge.h"
#include "rrm_cv5_8_motor/MotorState.h"

class MotorDriver {

public:
    // Constructor
    MotorDriver();

    // Callbacks
    bool acknowledgeService(rrm_cv5_8_motor::Acknowledge::Request &req, rrm_cv5_8_motor::Acknowledge::Response &res);
    bool setDirectionService(rrm_cv5_8_motor::SetDirection::Request &req, rrm_cv5_8_motor::SetDirection::Response &res);
    bool setPeriodService(rrm_cv5_8_motor::SetPeriod::Request &req, rrm_cv5_8_motor::SetPeriod::Response &res);
    bool setStartService(rrm_cv5_8_motor::SetStart::Request &req, rrm_cv5_8_motor::SetStart::Response &res);
    bool setStopService(rrm_cv5_8_motor::SetStop::Request &req, rrm_cv5_8_motor::SetStop::Response &res);
    void motorVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg);

    // Methods
    bool acknowledge(bool acknowledge_req, bool *acknowledge_res);
    bool setDirection(std::string direction_req, std::string *direction_res);
    bool setPeriod(int period_req, int *period_res);
    bool setStart(bool set_start_req, bool *set_start_res);
    bool setStop(bool set_stop_req, bool *set_stop_res);

    // Other public methods
    bool readPose(double *motor_pose);
    void publishMotorState(double motor_pose);
    uint32_t getStateRate();
    void setMotorPositionStatus(double motor_pose);
    bool isMotorEnabled();
    void publishJointStates();

    // Destructor
    ~MotorDriver();

private:

    // Consts
    const std::string ACKNOWLEDGE_MSG = "ACK";
    const std::string START_MSG = "STR";
    const std::string STOP_MSG = "STP";
    const std::string PERIOD_MSG = "PRD";
    const std::string LEFT_MSG = "LFT";
    const std::string RIGHT_MSG = "RGH";
    const std::string MSG_END = "\r\n";
    const double MOTOR_STEP_ANGLE = 0.9;
    const int COMMUNICATION_DELAY_ = 600;

    // member variables
    std::string port_;
    uint32_t baud_;
    uint32_t serial_timeout_;
    uint32_t motor_state_rate_;
    ros::Time last_position_change_;

    // motor state msg
    rrm_cv5_8_motor::MotorState motor_state_;

    // publishers
    ros::Publisher motor_state_pub_;
    ros::Publisher joint_state_pub_;

    // subscribers
    ros::Subscriber motor_velocity_sub_;

    // services
    ros::ServiceServer acknowledge_service_;
    ros::ServiceServer set_start_service_;
    ros::ServiceServer set_stop_service_;
    ros::ServiceServer set_direction_service_;
    ros::ServiceServer set_period_service_;

    // serial port
    std::shared_ptr<serial::Serial> motor_serial_;
};


#endif //PROJECT_MOTORDRIVER_H
