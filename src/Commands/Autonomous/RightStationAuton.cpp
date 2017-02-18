#include <Commands/Autonomous/RightStationAuton.h>
#include <Commands/Autonomous/DriveUntilDistance.h>
#include <Commands/Autonomous/RotateToAngle.h>
#include <Commands/Autonomous/DriveStraightForTime.h>

RightStationAuton::RightStationAuton() {
	AddSequential(new DriveStraightForTime(2));
	AddSequential((Command *) new RotateToAngle(60));
	AddSequential(new DriveUntilDistance(8.9));
}
