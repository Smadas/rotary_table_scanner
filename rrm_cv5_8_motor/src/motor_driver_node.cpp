#include "ros/ros.h"
#include "rrm_cv5_8_motor/MotorDriver.h"

int main(int argc, char **argv)
{
    //init ros node
    ros::init(argc, argv, "motor_driver_node");

    //communicate with motor
    MotorDriver motorDriver;
    double motor_pose;
    int spin_counter = 0;
    ros::Rate loop_rate(motorDriver.getStateRate());
    while(ros::ok())
    {
        spin_counter++;
        if (spin_counter > 50)
        {
            spin_counter = 0;

            // check if motor is enabled
            motorDriver.isMotorEnabled();
        }


        // read and publish motor state
        if (motorDriver.readPose(&motor_pose))
        {
            motorDriver.setMotorPositionStatus(motor_pose);
        }

        // publish states
        motorDriver.publishMotorState(motor_pose);
        motorDriver.publishJointStates();

        // check callbacks
        ros::spinOnce();

        // sleep to get desired loop rate
        loop_rate.sleep();
    }
    return 0;
}
