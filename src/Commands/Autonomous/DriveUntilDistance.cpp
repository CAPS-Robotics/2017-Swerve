#include <Commands/Autonomous/DriveUntilDistance.h>
#include "Robot.h"
#include "WPILib.h"

DriveUntilDistance::DriveUntilDistance(float distance) {
	// Use Requires() here to declare subsystem dependencies
	this->distance = distance;
	Requires(Robot::drivetrain.get());
}

// Called just before this  runs the first time
void DriveUntilDistance::Initialize() {
}

// Called repeatedly when this  is scheduled to run
void DriveUntilDistance::Execute() {
	Robot::drivetrain->ArcadeDrive(0.8, 0, 1);
}

// Make this return true when this  no longer needs to run execute()
bool DriveUntilDistance::IsFinished() {
	return fabs(Robot::drivetrain->GetDistanceAway() - distance) < 0.5;
}

// Called once after isFinished returns true
void DriveUntilDistance::End() {
	Robot::drivetrain->Brake();
}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void DriveUntilDistance::Interrupted() {
}
