#include <Commands/Autonomous/DriveUntilDistance.h>
#include "Robot.h"
#include "WPILib.h"

DriveUntilDistance::DriveUntilDistance(float distance) {
	// Use Requires() here to declare subsystem dependencies
	this->distance = distance;
	Requires(Robot::drivetrain.get());
	this->speed = 0.3;
}

// Called just before this  runs the first time
void DriveUntilDistance::Initialize() {
	Robot::drivetrain->ReturnWheelsToZero();
}

// Called repeatedly when this  is scheduled to run
void DriveUntilDistance::Execute() {
	Robot::drivetrain->Drive(0, 0.35, 1);
	if (Robot::drivetrain->GetDistanceAway() < distance) {
		End();
	}
}

// Make this return true when this  no longer needs to run execute()
bool DriveUntilDistance::IsFinished() {
	return Robot::drivetrain->GetDistanceAway() < distance;
}

// Called once after isFinished returns true
void DriveUntilDistance::End() {
	Robot::drivetrain->Brake();
}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void DriveUntilDistance::Interrupted() {
	End();
}
