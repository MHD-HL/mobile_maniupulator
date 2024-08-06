#ifndef ARM_HARDWARE_INTERFACE_H // edit the name of robot interface
#define ARM_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <serial/serial.h>

class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
	public:
        ROBOTHardwareInterface(ros::NodeHandle& nh);
        ~ROBOTHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        void tryToOpenPort();
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;

        joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
        joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
        
        int num_joints_;
        std::string joint_names_[5];  
        double joint_position_[5];
        double joint_velocity_[5];
        double joint_effort_[5];
        double joint_position_command_[5];
        
        std::string port_name_;
        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        serial::Serial serial_port_;
        std::string last_sent_message_;

};

#endif
