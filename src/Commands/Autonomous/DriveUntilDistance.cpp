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
}

// Called repeatedly when this  is scheduled to run
void DriveUntilDistance::Execute() {
	/*if (fabs(Robot::drivetrain->GetDistanceAway() - distance) < 6) {
		speed -= 0.05;
	} else {
		speed += 0.05;
	}
	if (speed <= 0.2) {
		speed += 0.05;
	} else if (speed >= 0.75) {
		speed -= 0.05;
	}*/
	Robot::drivetrain->CrabDrive(0.05, 0.25, 0, 1);
}

// Make this return true when this  no longer needs to run execute()
bool DriveUntilDistance::IsFinished() {
	return fabs(Robot::drivetrain->GetDistanceAway() - distance) < 1;
}

// Called once after isFinished returns true
void DriveUntilDistance::End() {
	Robot::drivetrain->Brake();
}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void DriveUntilDistance::Interrupted() {
}
