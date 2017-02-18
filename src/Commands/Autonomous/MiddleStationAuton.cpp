#include <Commands/Autonomous/TestAuton.h>
#include <Commands/Autonomous/DriveUntilDistance.h>
#include <Commands/Autonomous/RotateToAngle.h>
#include <Commands/Autonomous/DriveStraightForTime.h>

TestAuton::TestAuton() {
	AddSequential(new DriveUntilDistance(8.9));
}
