#include "WPILib.h"
#include <iostream>
#include <boost/asio/asio.hpp>


class CommandBasedRobot : public IterativeRobot {
private:

	virtual void RobotInit() {
		cout << "Init\n";
	}

	virtual void AutonomousInit() {
		cout << "Auto Init" << DriverStation::GetInstance()->GetPacketNumber() << " " << Timer::GetPPCTimestamp();
	}

	virtual void AutonomousPeriodic() {
		cout << "Auto Periodic " << DriverStation::GetInstance()->GetPacketNumber() << " " << Timer::GetPPCTimestamp() << endl;
	}

	virtual void TeleopInit() {
		cout << "Teleop Init " << DriverStation::GetInstance()->GetPacketNumber() << " " << Timer::GetPPCTimestamp() << endl;
	}

	virtual void TeleopPeriodic() {
		cout << "Teleop Periodic " << DriverStation::GetInstance()->GetPacketNumber() << " " << Timer::GetPPCTimestamp() << endl;
	}

	virtual void TestPeriodic() {
		cout << "Teleop Periodic" << DriverStation::GetInstance()->GetPacketNumber() << " " << Timer::GetPPCTimestamp() << endl;
	}

	virtual void DisabledInit() {
		cout << "Disabled Periodic" << DriverStation::GetInstance()->GetPacketNumber() << " " << Timer::GetPPCTimestamp() << endl;
	}

	virtual void DisabledPeriodic() {
		cout << "Disabled Periodic" << DriverStation::GetInstance()->GetPacketNumber() << " " << Timer::GetPPCTimestamp() << endl;
	}
};

START_ROBOT_CLASS(CommandBasedRobot);
