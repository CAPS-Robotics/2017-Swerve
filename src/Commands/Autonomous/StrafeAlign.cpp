#include <Commands/Autonomous/StrafeAlign.h>
#include "Robot.h"
#include "WPILib.h"

StrafeAlign::StrafeAlign() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::drivetrain.get());
	Requires(Robot::vision.get());
	pos = 0;
}

// Called just before this  runs the first time
void StrafeAlign::Initialize() {
}

// Called repeatedly when this  is scheduled to run
void StrafeAlign::Execute() {
	pos = Robot::vision->GetCentralValue();
	double dist = pos - 160;
	if (dist > 5) {
		Robot::drivetrain->Drive(1.25, -0.225, 1);
	} else if (dist < -5) {
		Robot::drivetrain->Drive(1.25, 0.225, 1);
	} else if (dist < -155) {
		Robot::drivetrain->Brake();
		End();
	} else {
		Robot::drivetrain->Brake();
		End();
	}
}

// Make this return true when this  no longer needs to run execute()
bool StrafeAlign::IsFinished() {
	return pos <= 164 && pos >= 156;
}

// Called once after isFinished returns true
void StrafeAlign::End() {
	Robot::drivetrain->Brake();
}

// Called when another  which requires one or more of the same
// subsystems is scheduled to run
void StrafeAlign::Interrupted() {
	End();
}
