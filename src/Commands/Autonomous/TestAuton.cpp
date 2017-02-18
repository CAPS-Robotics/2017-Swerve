#include <Commands/Autonomous/TestAuton.h>
#include <Commands/Autonomous/DriveUntilDistance.h>
#include <Commands/Autonomous/RotateToAngle.h>
#include <Commands/Autonomous/DriveStraightForTime.h>

TestAuton::TestAuton() {
	AddSequential(new RotateToAngle(60));
}
