#include "../RobotMap.h"
#include "../Robot.h"
#include <PigeonImu.h>
#include <Subsystems/PigeonNav.h>

PigeonNav::PigeonNav() : Subsystem("PigeonNav") {
	Robot::climber.get();
	this->gyro = new PigeonImu(/* Pass in the SRX that the board is plugged into */);
	this->ypr = new double[3];
	this->ResetHeading();
}

double PigeonNav::PIDGet() {
	return this->GetHeading();
}

double PigeonNav::GetHeading() {
	double angle = fmod(fmod(this->gyro->GetFusedHeading(), 360) + 360, 360);
	return angle <= 180 ? angle : -360 + angle;
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

