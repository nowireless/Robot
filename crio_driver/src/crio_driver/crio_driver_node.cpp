#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class CRIO : public hardware_interface::RobotHW {
public:
	CRIO();
	~CRIO();
	ros::Time GetTime() const;
	ros::Duration GetPeriod() const;
	void Read();
	void Write();
	
private:
	hardware_interface::JointStateInterface m_jointStateInterface;
	hardware_interface::VelocityJointInterface m_jointVelocityInterface;

	double m_cmd[2];
	double m_pos[2];
	double m_vel[2];
	double m_eff[2];
	
};

CRIO::CRIO() {
	hardware_interface::JointStateHandle stateHandleLeft("base_to_lwheel", &m_pos[0], &m_vel[0], &m_eff[0]);
	m_jointStateInterface.registerHandle(stateHandleLeft);
	
	hardware_interface::JointStateHandle stateHandleRight("base_to_rwheel", &m_pos[1], &m_vel[1], &m_eff[1]);
	m_jointStateInterface.registerHandle(stateHandleRight);
	
	this->registerInterface(&m_jointStateInterface);
	
	hardware_interface::JointHandle velHandleLeft(stateHandleLeft, &m_cmd[0]);
	m_jointVelocityInterface.registerHandle(velHandleLeft);
	
	hardware_interface::JointHandle velHandleRight(stateHandleRight, &m_cmd[1]);
	m_jointVelocityInterface.registerHandle(velHandleRight);
	
	this->registerInterface(&m_jointVelocityInterface);

}

CRIO::~CRIO() {}

ros::Time CRIO::GetTime() const {
	return ros::Time::now();
}

ros::Duration CRIO::GetPeriod() const {
	return ros::Duration(0.01);
}

void CRIO::Write() {
	//ROS_INFO("Writing");
}
void CRIO::Read() {
	//ROS_INFO("Reading");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "crio_driver");
	ros::NodeHandle nh;
	
	CRIO robot;
	controller_manager::ControllerManager cm(&robot, nh);
	
	ros::Rate rate(1.0/robot.GetPeriod().toSec());
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	while(ros::ok()) {
		robot.Read();
		robot.Write();
		cm.update(robot.GetTime(), robot.GetPeriod());
		rate.sleep();
	}
	ROS_INFO("Stopped");
	spinner.stop();
	
	return 0;
		
}
