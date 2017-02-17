#include <Commands/Drivetrain/DriveStraightForTime.h>
#include "RobotMap.h"
#include "Robot.h"

DriveStraightForTime::DriveStraightForTime(double timeout) : TimedCommand(timeout) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::drivetrain.get());
}

// Called just before this Command runs the first time
void DriveStraightForTime::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveStraightForTime::Execute() {
	Robot::drivetrain->CrabDrive(0, 0.8, 0, 1, true);
}

// Called once after command times out
void DriveStraightForTime::End() {
	Robot::drivetrain->Brake();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveStraightForTime::Interrupted() {

}
