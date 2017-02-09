#include <Commands/Climber/StopClimbing.h>
#include "WPILib.h"
#include "Robot.h"
#include "RobotMap.h"

StopClimbing::StopClimbing() {
	Requires(Robot::climber.get());
}

// Called just before this Command runs the first time
void StopClimbing::Initialize() {
	Robot::climber->StopClimber();
}

// Called repeatedly when this Command is scheduled to run
void StopClimbing::Execute() {
}


// Make this return true when this  no longer needs to run execute()
bool StopClimbing::IsFinished() {
	return true;
}

// Called once after command times out
void StopClimbing::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void StopClimbing::Interrupted() {

}
