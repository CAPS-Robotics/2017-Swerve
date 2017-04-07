#include <Commands/Autonomous/EasyMiddle.h>
#include <Commands/Autonomous/DriveUntilDistance.h>
#include <Commands/Autonomous/RotateToAngle.h>
#include <Commands/Autonomous/StrafeAlign.h>
#include <Commands/Autonomous/DriveStraightForTime.h>
#include "../../RobotMap.h"

EasyMiddle::EasyMiddle() {
	AddSequential(new DriveUntilDistance(GEAR_DISTANCE));
}
