#include "SwerveModule.h"
#include "../RobotMap.h"
#include <CANTalon.h>

SwerveModule::SwerveModule(int steerMotor, int driveMotor, bool isInverted) : Subsystem("SwerveModule") {
	this->steer = new CANTalon(steerMotor);
	this->drive = new Talon(driveMotor);
	this->drive->SetInverted(isInverted);
	// Init the Talon SRX
	steer->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	steer->ConfigEncoderCodesPerRev(497);
	steer->SetControlMode(CANTalon::kPosition);
	steer->SetPosition(0);
	steer->SetPID(2.5f, 0.0002f, 0.0005f);
	steer->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	steer->SetAllowableClosedLoopErr(1);
}

void SwerveModule::InitDefaultCommand() {

}

void SwerveModule::Drive(double speed, double angle) {
	speed = fabs(speed) > 0.1 ? speed : 0;

	double dist = angle - this->GetAngle();

	if (fabs(dist) > 90 && fabs(dist) < 270) {
		angle = fmod(180 + angle, 360);
		speed = -speed;
	}

	this->drive->Set(speed);
	this->steer->Set(angle * GR / 360);
}

void SwerveModule::ReturnToZero() {
	this->steer->Set(0);
}

void SwerveModule::ResetEncoder() {
	this->steer->SetPosition(0);
}

double SwerveModule::GetAngle() {
	double revs = this->steer->Get();
	double netRev = revs;
	if (revs > 0.833) {
		netRev = revs - (int)revs;
	} else if (revs < -0.833) {
		netRev = revs + (int)revs;
	}
	return netRev / GR * 360;
}
