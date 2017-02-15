#include <Commands/Autonomous/TestAuton.h>
#include <Commands/Autonomous/DriveUntilDistance.h>
#include <Commands/Drivetrain/DriveStraightForTime.h>

TestAuton::TestAuton() {
	AddSequential(new DriveUntilDistance(9));
}
