#include <Commands/Autonomous/TestAuton.h>
#include <Commands/Autonomous/DriveUntilDistance.h>

TestAuton::TestAuton() {
	AddSequential(new DriveUntilDistance(10));
}
