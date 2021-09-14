#ifndef _YARP5_BASE_DRIVER_
#define _YARP5_BASE_DRIVER_

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float32MultiArray.h>

class BaseDriver: public hardware_interface::RobotHW{
    
public:
    BaseDriver();
    
    BaseDriver(std::string left_wheel_joint_name, std::string right_string_joint_name);
    
    void reopen();
    void read();
    void write(double dt);            
    
    void encoder_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);
    
    ros::Publisher motor_pub;
    
private:    
            
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface joint_vel_interface;
    
    double wR; 
    double wL;
    
    // information storages for diff drive
    double cmd[2]; // rad/s
    double pos[2]; // rad
    double vel[2]; // rad/s
    double eff[2]; //         
        
    //int speed_multiplier_;
};


#endif // _YARP5_BASE_DRIVER_
