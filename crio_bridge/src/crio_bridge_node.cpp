#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <memory>
#include <string>
#include <atomic>
#include <limits>

#include <ntcore.h>
#include <tables/ITableListener.h>
#include <networktables/NetworkTable.h>

using namespace std;


class BridgeTableListener : public ITableListener {
public:
	BridgeTableListener() {}
	virtual ~BridgeTableListener() {}

	virtual void ValueChanged(
			ITable* source, llvm::StringRef key, std::shared_ptr<nt::Value> value, bool isNew);
};

void BridgeTableListener::ValueChanged(ITable* source,	llvm::StringRef key,std::shared_ptr<nt::Value> value, bool isNew) {
	ROS_INFO("Key %s", key.str().c_str());
}

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

int main(int argc, char **argv) {

	ros::init(argc, argv, "crio_bridge");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	CONNECTED.store(false);

	//Initialize Networktables
	string ip;
	n.param<string>("crio_ip",ip,"127.0.0.1");
	ROS_INFO("NT server IP %s", ip.c_str());
	ROS_INFO("Starting NT");
	nt::AddConnectionListener(connection_callback,true);
	nt::SetLogger(nt_log_callback, NT_LOG_INFO);
	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress(ip);

	//Get bridge table
	shared_ptr<NetworkTable> bridge = NetworkTable::GetTable("bridge");
	bridge->AddTableListener(new BridgeTableListener());

	//Get control table
	shared_ptr<NetworkTable> connectionTable = NetworkTable::GetTable("connection");

	ros::Publisher pubConnected = n.advertise<std_msgs::Bool>("connected", 100);



	while(ros::ok()) {
		std_msgs::Bool connectedMsg;
		connectedMsg.data = CONNECTED.load();
		pubConnected.publish(connectedMsg);

		connectionTable->PutNumber("time", ros::Time::now().toSec());
		ros::spinOnce();
		loop_rate.sleep();
	}


  return 0;
}
