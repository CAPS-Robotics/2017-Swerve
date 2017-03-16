#include <Commands/Autonomous/PlaceGear.h>
#include <Commands/Autonomous/DriveUntilDistance.h>
#include <Commands/Autonomous/StrafeAlign.h>
#include <Commands/Autonomous/RotateToAngle.h>

PlaceGear::PlaceGear(double angle) {
	AddSequential(new RotateToAngle(angle));
	AddSequential(new DriveUntilDistance(24));
	AddSequential(new StrafeAlign());
	AddSequential(new DriveUntilDistance(9.1));
}
