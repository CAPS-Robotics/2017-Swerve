#include <Commands/Climber/Climb.h>
#include "WPILib.h"
#include "Robot.h"
#include "RobotMap.h"

Climb::Climb() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::climber.get());
}

// Called just before this Command runs the first time
void Climb::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
	Robot::climber->SetClimberSpeed(-1);
}


// Make this return true when this  no longer needs to run execute()
bool Climb::IsFinished() {
	return Robot::climber->Get1Current() > 45 || Robot::climber->Get2Current() > 45;
}

// Called once after command times out
void Climb::End() {
	Robot::climber->StopClimber();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Climb::Interrupted() {

}
