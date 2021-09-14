#include "base_driver_node.h" 
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>

BaseDriver* bd;

BaseDriver::BaseDriver(){}

BaseDriver::BaseDriver(std::string left_wheel_joint_name, std::string right_wheel_joint_name){
    
    // DIFF DRIVE INTERFACE
    pos[0] = 0.0; pos[1] = 0.0;
    vel[0] = 0.0; vel[1] = 0.0;
    eff[0] = 0.0; eff[1] = 0.0;
    cmd[0] = 0.0; cmd[1] = 0.0;        
    
    hardware_interface::JointStateHandle left_wheel_state_handle( left_wheel_joint_name, pos, vel, eff);  
    joint_state_interface.registerHandle(left_wheel_state_handle);
    
    hardware_interface::JointStateHandle right_wheel_state_handle( right_wheel_joint_name, pos+1, vel+1, eff+1);
    joint_state_interface.registerHandle(right_wheel_state_handle);
    
    registerInterface(&joint_state_interface);
    
    hardware_interface::JointHandle left_wheel_handle(joint_state_interface.getHandle(left_wheel_joint_name), cmd);
    joint_vel_interface.registerHandle(left_wheel_handle);
    
    hardware_interface::JointHandle right_wheel_handle(joint_state_interface.getHandle(right_wheel_joint_name), cmd+1);
    joint_vel_interface.registerHandle(right_wheel_handle);
    
    registerInterface(&joint_vel_interface);        
        
}

void BaseDriver::read(){
    std_msgs::Float32MultiArray cmd_msg;
    cmd_msg.data.push_back(cmd[0]);
    cmd_msg.data.push_back(cmd[1]);
    motor_pub.publish(cmd_msg);    
}

void BaseDriver::write(double dt){                
    vel[0] = wL;
    vel[1] = wR;
    pos[0] += vel[0] * dt;
    pos[1] += vel[1] * dt;        
}

void BaseDriver::encoder_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
    wL = msg->data[0];
    wR = msg->data[1];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pololu_dual_vnh5019_base_driver_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh("~");        
    
    std::string lw_joint = "lwheel1_j";
    std::string rw_joint = "rwheel1_j";
    double rate_hz = 100;
    //TODO paramns
    nh.getParam("lw_joint", lw_joint);
    nh.getParam("rw_joint", rw_joint);
    nh.getParam("rate_hz", rate_hz);    
    
    bd = new BaseDriver(lw_joint, rw_joint);    
    controller_manager::ControllerManager cm(bd, nh_);    

    ros::Subscriber motor_info_sub = nh_.subscribe("motor_info", 1, &BaseDriver::encoder_cb, bd);
    bd->motor_pub = nh.advertise<std_msgs::Float32MultiArray>("motor_speed", 1);
            
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Rate rate(rate_hz);
    ros::Time prev_time = ros::Time::now();
    while(ros::ok()){        
        ros::Duration dt = ros::Time::now() - prev_time;
        prev_time = ros::Time::now();
        
        bd->write(dt.toSec());
            
        cm.update(ros::Time::now(), dt);
        bd->read();
                                                 
        rate.sleep();                
    }
    spinner.stop();
    return 0;
}
