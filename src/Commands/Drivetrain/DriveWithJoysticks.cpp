#include "DriveWithJoysticks.h"
#include "OI.h"
#include "Robot.h"
#include "WPILib.h"

DriveWithJoysticks::DriveWithJoysticks() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::drivetrain.get());
	Robot::oi.get();
}

// Called just before this  runs the first time
void DriveWithJoysticks::Initialize() {
}

// Called repeatedly when this  is scheduled to run
void DriveWithJoysticks::Execute() {
	double speedMultiplier = (1 - Robot::oi->GetSlider()) / 2;
	Robot::drivetrain->Drive(Robot::oi->GetX(), Robot::oi->GetY(), Robot::oi->GetTwist(), speedMultiplier);
}

// Make this return true when this  no longer needs to run execute()
bool DriveWithJoysticks::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveWithJoysticks::End() {
	Robot::drivetrain->Brake();
}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoysticks::Interrupted() {
	End();
}
