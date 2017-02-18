#include <Commands/Autonomous/MiddleStationAuton.h>
#include <Commands/Autonomous/DriveUntilDistance.h>
#include <Commands/Autonomous/RotateToAngle.h>
#include <Commands/Autonomous/DriveStraightForTime.h>

MiddleStationAuton::MiddleStationAuton() {
	AddSequential(new DriveUntilDistance(8.9));
}
