#include "rrm_cv5_8_motor/MotorDriver.h"

MotorDriver::MotorDriver(){

    // NodeHandler
    ros::NodeHandle n;

    //init serial constants
    int get_int = 0;
    port_ = "/dev/ttyACM0";
    baud_ = 9600;
    serial_timeout_ = 1000;
    n.getParam("baud", get_int);
    baud_ = (uint32_t)get_int;
    n.getParam("port", port_);
    n.getParam("serial_timeout", get_int);
    serial_timeout_= (uint32_t)get_int;
    n.getParam("motor_state_rate", get_int);
    motor_state_rate_ = (uint32_t)get_int;

    // open serial port
    motor_serial_ = std::make_shared<serial::Serial>(port_, baud_, serial::Timeout::simpleTimeout(serial_timeout_));

    // topic names
    std::string motor_velocity_topic = "rrm_cv5_8_motor/cmd_vel";
    std::string motor_state_topic = "rrm_cv5_8_motor/motor_state";
    n.getParam("motor_velocity_topic", motor_velocity_topic);
    n.getParam("motor_state_topic", motor_state_topic);

    // Publisher
    motor_state_pub_ = n.advertise<rrm_cv5_8_motor::MotorState>(motor_state_topic, 1);
    joint_state_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // Subscriber
    motor_velocity_sub_ = n.subscribe(motor_velocity_topic, 1, &MotorDriver::motorVelocityCallback, this);

    // Service server
    acknowledge_service_ = n.advertiseService("/rrm_cv5_8_motor/acknowledge", &MotorDriver::acknowledgeService, this);
    set_start_service_ = n.advertiseService("/rrm_cv5_8_motor/set_start", &MotorDriver::setStartService, this);
    set_stop_service_ = n.advertiseService("/rrm_cv5_8_motor/set_stop", &MotorDriver::setStopService, this);
    set_direction_service_ = n.advertiseService("/rrm_cv5_8_motor/set_direction", &MotorDriver::setDirectionService, this);
    set_period_service_ = n.advertiseService("/rrm_cv5_8_motor/set_period", &MotorDriver::setPeriodService, this);

    //init variables
    motor_state_.position = 0;
    motor_state_.direction = "LEFT";
    motor_state_.period = 30;
    motor_state_.enabled = false;
}

MotorDriver::~MotorDriver()
{
    try{
        motor_serial_->close();

    }catch (std::exception &e){
        ROS_ERROR_STREAM("Unable to close serial port!");
    }
}

bool MotorDriver::acknowledgeService(rrm_cv5_8_motor::Acknowledge::Request &req, rrm_cv5_8_motor::Acknowledge::Response &res)
{
    bool acknowledge_res;
    bool ret_val = acknowledge(req.acknowledge, &acknowledge_res);
    res.acknowledge = acknowledge_res;
    return ret_val;
}

bool MotorDriver::acknowledge(bool acknowledge_req, bool *acknowledge_res)
{
    //send serial acknowledge to motor
    if(acknowledge_req)
    {
        //send ack msg to motor
        motor_serial_->write(ACKNOWLEDGE_MSG);

        //send msg end character
        uint8_t msg_end = 0;
        motor_serial_->write(&msg_end, 1);
    }

    *acknowledge_res = true;
    return true;
}

bool MotorDriver::setStartService(rrm_cv5_8_motor::SetStart::Request &req, rrm_cv5_8_motor::SetStart::Response &res)
{
    bool start_res;
    bool ret_val = setStart(req.start, &start_res);
    res.start = start_res;
    return ret_val;
}

bool MotorDriver::setStart(bool set_start_req, bool *set_start_res)
{
    bool ret_val = true;
    *set_start_res = true;

    //send serial start to motor
    if(set_start_req)
    {
        //send start msg to motor
        motor_serial_->write(START_MSG);

        //send msg end character
        uint8_t msg_end = 0;
        motor_serial_->write(&msg_end, 1);

        //receive msg acknowledge from motor
        std::string read_msg = motor_serial_->readline(65536, "\r\n");

        if (read_msg != START_MSG + MSG_END)
        {
            *set_start_res = false;

        }
        else
        {
            motor_state_.enabled = true;
        }
    }

    return ret_val;
}

bool MotorDriver::setStopService(rrm_cv5_8_motor::SetStop::Request &req, rrm_cv5_8_motor::SetStop::Response &res)
{
    bool set_stop_res;
    bool ret_val = setStop(req.stop, &set_stop_res);
    res.stop = set_stop_res;
    return  ret_val;
}

bool MotorDriver::setStop(bool set_stop_req, bool *set_stop_res)
{
    //todo add sending command to motor and receiving response

    bool ret_val = true;
    *set_stop_res = true;

    //send serial stop to motor
    if(set_stop_req)
    {
        //send start msg to motor
        motor_serial_->write(STOP_MSG);

        //send msg end character
        uint8_t msg_end = 0;
        motor_serial_->write(&msg_end, 1);

        //receive msg acknowledge from motor
        std::string read_msg = motor_serial_->readline(65536, MSG_END);
        if (read_msg != STOP_MSG + MSG_END)
        {
            *set_stop_res = false;
        }
        else
        {
            motor_state_.enabled = false;
        }
    }

    return ret_val;
}

bool MotorDriver::setDirectionService(rrm_cv5_8_motor::SetDirection::Request &req, rrm_cv5_8_motor::SetDirection::Response &res)
{
    std::string direction_res;
    bool ret_val = setDirection(req.direction, &direction_res);
    res.direction = direction_res;
    return ret_val;
}

bool MotorDriver::setDirection(std::string direction_req, std::string *direction_res)
{
    //todo add sending command to motor and receiving response
    bool ret_val = true;
    *direction_res = "FAIL";

    //send serial direction to motor

    if (direction_req == "LEFT")
    {
        //send LEFT msg to motor
        motor_serial_->write(LEFT_MSG);

        //send msg end character
        uint8_t msg_end = 0;
        motor_serial_->write(&msg_end, 1);

        //receive msg acknowledge from motor
        std::string read_msg = motor_serial_->readline(65536, MSG_END);
        if (read_msg == LEFT_MSG + MSG_END)
        {
            *direction_res = "LEFT";
            motor_state_.direction = "LEFT";
        }
    }
    else if (direction_req == "RIGHT")
    {
        //send RIGHT msg to motor
        motor_serial_->write(RIGHT_MSG);

        //send msg end character
        uint8_t msg_end = 0;
        motor_serial_->write(&msg_end, 1);

        //receive msg acknowledge from motor
        std::string read_msg = motor_serial_->readline(65536, MSG_END);
        if (read_msg == RIGHT_MSG + MSG_END)
        {
            *direction_res = "RIGHT";
            motor_state_.direction = "RIGHT";
        }
    }

    return ret_val;
}

bool MotorDriver::setPeriodService(rrm_cv5_8_motor::SetPeriod::Request &req, rrm_cv5_8_motor::SetPeriod::Response &res)
{
    int period_res;
    bool ret_val = setPeriod(req.period, &period_res);//(req.period, &res.period);
    res.period = period_res;
    return ret_val;
}

bool MotorDriver::setPeriod(int period_req, int *period_res)
{
    //todo add sending command to motor and receiving response
    bool ret_val = true;
    *period_res = 0;

    //send serial period to motor
    if((period_req > 0)&&(period_req < 30000))
    {
        //send period msg to motor
        motor_serial_->write(PERIOD_MSG);

        //send msg end character
        uint8_t msg_end = 0;
        motor_serial_->write(&msg_end, 1);

        //receive msg acknowledge from motor
        std::string read_msg = motor_serial_->readline(65536, MSG_END);
        if (read_msg != PERIOD_MSG + MSG_END)
        {
            *period_res = 0;
        }

        //send period to motor
        std::stringstream ss;
        ss << period_req;
        motor_serial_->write(ss.str());

        //send msg end character
        msg_end = 0;
        motor_serial_->write(&msg_end, 1);

        //receive msg acknowledge from motor
        read_msg = motor_serial_->readline(65536, MSG_END);
        int read_msg_int = atoi(read_msg.c_str());
        motor_state_.period = read_msg_int;
        if (period_req != read_msg_int)
        {
            *period_res = 0;
        }
        else
        {
            *period_res = read_msg_int;
        }
    }
    else
    {
        ret_val = false;
        *period_res = 0;
    }
    return ret_val;
}

void MotorDriver::motorVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // stop motor if velocity = 0
    if (msg->angular.z == 0)
    {
        bool setStopRet;
        setStop(true, &setStopRet);
    }
    else
    {
        // acknowledge motor movement
        bool acknowledgeRet;
        acknowledge(true, &acknowledgeRet);
        // if motor is stopped, start it
        if (motor_state_.enabled == false)
        {
            bool setStartRet;
            setStart(true, &setStartRet);
        }

        // change direction if velocity direction was changed
        if (msg->angular.z > 0)
        {
            if (motor_state_.direction == "LEFT")
            {

                std::string setDirectionRet;
                setDirection("RIGHT", &setDirectionRet);
            }
        }
        if (msg->angular.z < 0)
        {
            if (motor_state_.direction == "RIGHT")
            {
                std::string setDirectionRet;
                setDirection("LEFT", &setDirectionRet);
            }
        }
        // change period if velocity was changed
        unsigned int msg_period = (unsigned int)(M_PI/fabs(msg->angular.z)/0.2);
        if (motor_state_.period != msg_period)
        {
            int setPeriodRet;
            setPeriod(msg_period, &setPeriodRet);
        }

    }
}

//returns false if no data is available
bool MotorDriver::readPose(double *motor_pose)
{
    if (motor_serial_->available())
    {
        std::string read_msg = "";
        read_msg = motor_serial_->readline(65536, MSG_END);
        *motor_pose = atoi(read_msg.c_str());
        last_position_change_ = ros::Time::now();
        return true;
    }
    return false;
}

void MotorDriver::publishMotorState(double motor_pose)
{
    motor_state_.position = motor_pose/10;
    motor_state_pub_.publish(motor_state_);
    return;
}

uint32_t MotorDriver::getStateRate()
{
    return motor_state_rate_;
}

void MotorDriver::setMotorPositionStatus(double motor_pose)
{
    motor_state_.position = motor_pose;
}

bool MotorDriver::isMotorEnabled()
{
    if (motor_state_.enabled == true)
    {
        ros::Duration period_duration;
        period_duration.sec = (int)((motor_state_.period + COMMUNICATION_DELAY_)/1000);
        period_duration.nsec = ((motor_state_.period + COMMUNICATION_DELAY_) % 1000) * 1000000;
        if (period_duration < (ros::Time::now() - last_position_change_))
        {
            bool set_stop_ret;
            setStop(true, &set_stop_ret);
        }
    }
}

// todo create jointStatePublisher for plate
void MotorDriver::publishJointStates()
{
    //fill plate_pose_msg with data based on motor position
    std::vector<double> joint_states;
    joint_states.push_back(M_PI*motor_state_.position/180.0);
    sensor_msgs::JointState plate_pose_msg;
    plate_pose_msg.position = joint_states;
    plate_pose_msg.header.stamp = ros::Time::now();
    std::vector<std::basic_string<char>> plate_joint_name;
    std::basic_string<char> plate_joint_name_basic;
    plate_joint_name_basic = "base_link_to_plate";
    plate_joint_name.push_back(plate_joint_name_basic);
    plate_pose_msg.name = plate_joint_name;

    //publish plate joint state
    joint_state_pub_.publish(plate_pose_msg);
}