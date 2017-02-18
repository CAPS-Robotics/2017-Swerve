#include "../RobotMap.h"
#include "../Robot.h"
#include <PigeonImu.h>
#include <Subsystems/PigeonNav.h>

PigeonNav::PigeonNav() : Subsystem("PigeonNav") {
	Robot::climber.get();
	this->gyro = new PigeonImu(Robot::climber->GetClimber2());
	this->ypr = new double[3];
	this->ResetHeading();
}

double PigeonNav::PIDGet() {
	return this->GetHeading();
}

double PigeonNav::GetHeading() {
	return fmod(this->gyro->GetFusedHeading(), 360);
}

double PigeonNav::GetAngularRate() {
	this->gyro->GetRawGyro(this->ypr);
	return ypr[0];
}

void PigeonNav::ResetHeading() {
	this->gyro->SetFusedHeading(0);
}

void PigeonNav::InitDefaultCommand() {

}

