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
	Robot::drivetrain->rotationPid->Enable();
}

// Called repeatedly when this  is scheduled to run
void RotateToAngle::Execute() {
	Robot::drivetrain->rotationPid->SetSetpoint(angle);
}

// Make this return true when this  no longer needs to run execute()
bool RotateToAngle::IsFinished() {
	return fabs(Robot::drivetrain->GetHeading() - distance, 5);
}

// Called once after isFinished returns true
void RotateToAngle::End() {
	Robot::drivetrain->rotationPid->Disable();
	Robot::drivetrain->Brake();
}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void RotateToAngle::Interrupted() {
}
