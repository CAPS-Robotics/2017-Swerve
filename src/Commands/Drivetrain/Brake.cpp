#include "Brake.h"
#include "OI.h"
#include "Robot.h"
#include "WPILib.h"

Brake::Brake() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::drivetrain.get());
}

// Called just before this  runs the first time
void Brake::Initialize() {
}

// Called repeatedly when this  is scheduled to run
void Brake::Execute() {
	Robot::drivetrain->Brake();
}

// Make this return true when this  no longer needs to run execute()
bool Brake::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void Brake::End() {

}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void Brake::Interrupted() {
}
