#include "ReturnWheels.h"
#include "OI.h"
#include "Robot.h"
#include "WPILib.h"

ReturnWheels::ReturnWheels() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::drivetrain.get());
}

// Called just before this  runs the first time
void ReturnWheels::Initialize() {
}

// Called repeatedly when this  is scheduled to run
void ReturnWheels::Execute() {
	Robot::drivetrain->ReturnWheelsToZero();
}

// Make this return true when this  no longer needs to run execute()
bool ReturnWheels::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void ReturnWheels::End() {

}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void ReturnWheels::Interrupted() {
	End();
}
