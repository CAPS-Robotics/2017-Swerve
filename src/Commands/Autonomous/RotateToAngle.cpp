#include <Commands/Autonomous/RotateToAngle.h>
#include "Robot.h"
#include "WPILib.h"

RotateToAngle::RotateToAngle(float angle) {
	// Use Requires() here to declare subsystem dependencies
	this->angle = angle;
	Requires(Robot::drivetrain.get());
}

// Called just before this  runs the first time
void RotateToAngle::Initialize() {
}

// Called repeatedly when this  is scheduled to run
void RotateToAngle::Execute() {
	if ((Robot::gyro->GetHeading() - angle) > 0) {
		Robot::drivetrain->RotateRobot(0.3);
	} else {
		Robot::drivetrain->RotateRobot(-0.3);
	}
	SmartDashboard::PutString("Rotate Finished", "In Progress!");
}

// Make this return true when this  no longer needs to run execute()
bool RotateToAngle::IsFinished() {
	return fabs(Robot::gyro->GetHeading() - angle) < 7;
}

// Called once after isFinished returns true
void RotateToAngle::End() {
	SmartDashboard::PutString("Rotate Finished", "Done!");
	Robot::drivetrain->ReturnWheelsToZero();
	Robot::drivetrain->Brake();
}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void RotateToAngle::Interrupted() {

}
