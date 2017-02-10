#include "ZeroGyro.h"
#include "Robot.h"
#include "WPILib.h"

ZeroGyro::ZeroGyro() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::gyro.get());
}

// Called just before this  runs the first time
void ZeroGyro::Initialize() {
	Robot::gyro->ResetHeading();
}

// Called repeatedly when this  is scheduled to run
void ZeroGyro::Execute() {
}

// Make this return true when this  no longer needs to run execute()
bool ZeroGyro::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void ZeroGyro::End() {

}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void ZeroGyro::Interrupted() {
}
