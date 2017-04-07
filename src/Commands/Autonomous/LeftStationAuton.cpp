#include <Commands/Autonomous/LeftStationAuton.h>
#include <Commands/Autonomous/PlaceGear.h>
#include <Commands/Autonomous/RotateToAngle.h>
#include <Commands/Autonomous/DriveStraightForTime.h>
#include <Commands/Autonomous/StrafeAlign.h>
#include <Commands/Drivetrain/ZeroGyro.h>

LeftStationAuton::LeftStationAuton() {
	AddSequential(new ZeroGyro());
	AddSequential(new DriveStraightForTime(2.05));
	AddSequential(new PlaceGear(-60));
}
