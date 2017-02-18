#include <Commands/Autonomous/LeftStationAuton.h>
#include <Commands/Autonomous/DriveUntilDistance.h>
#include <Commands/Autonomous/RotateToAngle.h>
#include <Commands/Autonomous/DriveStraightForTime.h>

LeftStationAuton::LeftStationAuton() {
	AddSequential(new DriveStraightForTime(2));
	AddSequential(new RotateToAngle(-120));
	AddSequential(new DriveUntilDistance(8.9));
}
