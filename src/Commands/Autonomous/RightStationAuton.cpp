#include <Commands/Autonomous/RightStationAuton.h>
#include <Commands/Autonomous/PlaceGear.h>
#include <Commands/Drivetrain/ZeroGyro.h>
#include <Commands/Autonomous/DriveStraightForTime.h>

RightStationAuton::RightStationAuton() {
	AddSequential(new ZeroGyro());
	AddSequential(new DriveStraightForTime(2.05));
	AddSequential(new PlaceGear(60));
}
