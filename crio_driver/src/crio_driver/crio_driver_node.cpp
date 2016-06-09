#include <ros/ros.h>

#include <atomic>
#include <ntcore.h>
#include <tables/ITableListener.h>
#include <networktables/NetworkTable.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


/*
 * Log callback to bridge ros and Networktable logging
 */
void nt_log_callback(unsigned int level, const char* file, unsigned int line, const char* msg) {
	switch (level) {
		case NT_LOG_CRITICAL:
			ROS_FATAL("NT: %s (%s:%d)", msg, basename(file), line);
			break;
		case NT_LOG_ERROR:
			ROS_ERROR("NT: %s (%s:%d)", msg, basename(file), line);
			break;
		case NT_LOG_WARNING:
			ROS_WARN("NT: %s (%s:%d)", msg, basename(file), line);
			break;
		case NT_LOG_INFO:
			ROS_INFO("NT: %s (%s:%d)", msg, basename(file), line);
			break;
		case NT_LOG_DEBUG:
		case NT_LOG_DEBUG1:
		case NT_LOG_DEBUG2:
		case NT_LOG_DEBUG3:
		case NT_LOG_DEBUG4:
			ROS_DEBUG("NT: %s (%s:%d)", msg, basename(file), line);
			break;
		default:
			break;
	}
}

atomic<bool> CONNECTED;
void connection_callback(unsigned int uid, bool connected, const nt::ConnectionInfo& conn) {
	CONNECTED.store(connected);

	if(connected) {
		ROS_INFO("NT Connected");
	} else {
		ROS_INFO("NT Disconnected");
	}
}

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
	
	//Initialize Networktables
	string ip;
	n.param<string>("crio_ip", ip ,"127.0.0.1");
	ROS_INFO("NT server IP %s", ip.c_str());
	ROS_INFO("Starting NT");
	nt::AddConnectionListener(connection_callback,true);
	nt::SetLogger(nt_log_callback, NT_LOG_INFO);
	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress(llvm::StringRef(ip));

	//Get bridge table
	shared_ptr<NetworkTable> bridge = NetworkTable::GetTable("bridge");
	
	ros::Publisher pubConnected = n.advertise<std_msgs::Bool>("connected", 100);

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
