#include <Commands/Autonomous/MiddleStationAuton.h>
#include <Commands/Autonomous/DriveUntilDistance.h>
#include <Commands/Autonomous/RotateToAngle.h>
#include <Commands/Autonomous/StrafeAlign.h>
#include <Commands/Autonomous/DriveStraightForTime.h>
#include "../../RobotMap.h"

MiddleStationAuton::MiddleStationAuton() {
	AddSequential(new DriveUntilDistance(24));
	AddSequential(new StrafeAlign());
	AddSequential(new DriveUntilDistance(GEAR_DISTANCE));
}
