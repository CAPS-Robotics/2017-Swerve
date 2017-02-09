#include "../RobotMap.h"
#include "../Robot.h"
#include <PigeonImu.h>
#include <Subsystems/PigeonNav.h>

PigeonNav::PigeonNav() : Subsystem("PigeonNav") {
	Robot::climber.get();
	this->gyro = new PigeonImu(Robot::climber->GetClimber1());
}

double PigeonNav::GetHeading() {
	return fmod(this->gyro->GetFusedHeading(), 360);
}

void PigeonNav::ResetHeading() {
	this->gyro->SetFusedHeading(0);
}

void PigeonNav::InitDefaultCommand() {

}

