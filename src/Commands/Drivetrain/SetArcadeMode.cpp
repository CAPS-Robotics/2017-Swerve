#include "SetArcadeMode.h"
#include "OI.h"
#include "Robot.h"
#include "WPILib.h"
#include "../../Subsystems/Drivetrain.h"

SetArcadeMode::SetArcadeMode() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::drivetrain.get());
	Robot::oi.get();
}

// Called just before this  runs the first time
void SetArcadeMode::Initialize() {
}

// Called repeatedly when this  is scheduled to run
void SetArcadeMode::Execute() {
	Robot::drivetrain->SetControlMode(Drivetrain::ControlMode::arcade);
}

// Make this return true when this  no longer needs to run execute()
bool SetArcadeMode::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void SetArcadeMode::End() {
	Robot::drivetrain->SetControlMode(Drivetrain::ControlMode::fieldOrientedSwerve);
}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void SetArcadeMode::Interrupted() {
}
