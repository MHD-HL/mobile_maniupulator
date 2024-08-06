// edit the name from the folder
#include "arm_hardware_interface/arm_hardware_interface.h"
#include <ros/callback_queue.h>

double remap(double value, double fromLow, double fromHigh, double toLow, double toHigh)
{
    // Ensure the input range is valid
    if (fromLow == fromHigh)
    {
        throw std::invalid_argument("fromLow and fromHigh cannot be the same value.");
    }

    // Calculate the scale factor
    double scale = (toHigh - toLow) / (fromHigh - fromLow);

    // Remap the value
    return toLow + (value - fromLow) * scale;
}

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 70;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
    port_name_ = "/dev/ttyACM0";
    tryToOpenPort();

    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
    
    num_joints_ = 5;

	joint_names_[0] = "joint_1";	
	joint_names_[1] = "joint_2";
	joint_names_[2] = "joint_3";
	joint_names_[3] = "joint_4";
	joint_names_[4] = "joint_5";
	

    for (int i = 0; i < num_joints_; ++i) {

         // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        
        position_joint_interface_.registerHandle(jointPositionHandle);
   
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {

    if(!serial_port_.isOpen())
        tryToOpenPort();
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {

    if(!serial_port_.isOpen())
        return;

    try {
        if (last_sent_message_.empty())
            return;

        if (!serial_port_.waitReadable()) {
            ROS_WARN("Serial port timed out without receiving bytes");
            return;
        }

        const auto serial_message = serial_port_.readline();
        
        ///////////////////////////////////////////////
        ROS_INFO_STREAM("back_from_ard is  " << serial_message);
        ///////////////////////////////////////////////

        std::vector<std::string> tokens;
        boost::split(tokens, serial_message, boost::is_any_of(",$\n"));

        tokens.erase(std::remove(tokens.begin(), tokens.end(), ""),tokens.end());

        if (tokens.size() != 5) {
            ROS_WARN_STREAM(
                "Received message did not contain the right number of tokens. Expected 5, but got "
                    << tokens.size() << "\n" << serial_message);
            return;
        }

        float readed[5];

        readed[0] = std::stod(tokens[0]);
        readed[1] = remap(std::stod(tokens[1]),180,0,0,180);
        readed[2] = remap(std::stod(tokens[2]),180,0,0,180);
        readed[3] = std::stod(tokens[3]);
        readed[4] = std::stod(tokens[4]);

        for(int i = 0; i < 5; i++)
        {
            readed[i] = remap(readed[i],0,180,0,3.14);
            joint_position_[i] = remap(readed[i],0,3.14,-M_PI/2,M_PI/2);
        }

        last_sent_message_.clear();
    } 
    catch(const std::exception& e)
    {
        ROS_WARN_STREAM("Serial exception: " << e.what());
        serial_port_.close();
    }







// read the the joints values from the sended joints
    // joint_position_[0]=joint_position_command_[0];
    // joint_position_[1]=joint_position_command_[1];
    // joint_position_[2]=joint_position_command_[2];
    // joint_position_[3]=joint_position_command_[3];
    // joint_position_[4]=joint_position_command_[4];
    
   

    // joint_read.request.req=1.0;
	
	// if(client.call(joint_read))
	// {
	    
	// 	joint_position_[0]=angles::from_degrees(90-joint_read.response.res[0]);
	// 	joint_position_[1]=angles::from_degrees(joint_read.response.res[1]-90);
	// 	joint_position_[2]=angles::from_degrees(joint_read.response.res[2]-90);
		
	// 	//ROS_INFO("Receiving  j1: %.2f, j2: %.2f, j3: %.2f",joint_read.response.res[0],joint_read.response.res[1], joint_read.response.res[2]);
		
	// }	
    // else
    // {
    // 	joint_position_[0]=0;
    //     joint_position_[1]=0;
    //     joint_position_[2]=0;
    //     //ROS_INFO("Service not found ");
    // }
        

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
    
    if (!serial_port_.isOpen())
        return;
    try
    {

        auto joint_1 = joint_position_command_[0];
        auto joint_2 = joint_position_command_[1];
        auto joint_3 = joint_position_command_[2];
        auto joint_4 = joint_position_command_[3];
        auto joint_5 = joint_position_command_[4];

        joint_1 = remap(joint_1,-M_PI/2,M_PI/2,0,3.14);
        joint_2 = remap(joint_2,-M_PI/2,M_PI/2,0,3.14);
        joint_3 = remap(joint_3,-M_PI/2,M_PI/2,0,3.14);
        joint_4 = remap(joint_4,-M_PI/2,M_PI/2,0,3.14);
        joint_5 = remap(joint_5,-M_PI/2,M_PI/2,0,3.14);

        joint_1 = (int)remap(joint_1,0,3.14,0,180);  //(joint_1*180)/M_PI ;
        joint_2 = (int)remap(joint_2,0,3.14,0,180);  //(joint_2*180)/M_PI ;
        joint_3 = (int)remap(joint_3,0,3.14,0,180);  //(joint_3*180)/M_PI ;
        joint_4 = (int)remap(joint_4,0,3.14,0,180);  //(joint_4*180)/M_PI ;
        joint_5 = (int)remap(joint_5,0,3.14,0,180);  //(joint_5*180)/M_PI ;

        const auto serial_message =
            "$" + std::to_string(joint_1) + "," 
                + std::to_string(joint_2) + "," 
                + std::to_string(joint_3) + "," 
                + std::to_string(joint_4) + "," 
                + std::to_string(joint_5) +
            "\n";

        //////////////////////////////////////////////////
        ROS_INFO_STREAM("serial_message is  " << serial_message);
        //////////////////////////////////////////////////

        serial_port_.write(serial_message);

        last_sent_message_ = serial_message;
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM("Serial exception: " << e.what());
        serial_port_.close();
    }
}

void ROBOTHardwareInterface::tryToOpenPort()
{
    try
    {
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(57600);
        auto serial_timeout = serial::Timeout::simpleTimeout(250);
        serial_port_.setTimeout(serial_timeout);
        serial_port_.open();
        ROS_INFO_STREAM("Connected to motor control arduino on serial port " << port_name_);
        return;
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM_THROTTLE(60, "Could not open serial port, " << port_name_ << ": " << e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_hardware_interface");
    ros::CallbackQueue callback_queue;
    ros::NodeHandle nh;
    nh.setCallbackQueue(&callback_queue);
    ROBOTHardwareInterface ROBOT(nh);
    ros::MultiThreadedSpinner spinner; 
    spinner.spin(&callback_queue);
    return 0;
}
