#include "WPILib.h"
#include "CANTalon.h"
#include "config.h"

SwerveModule::SwerveModule(int steerMotor, int driveMotor, bool inverted) {
	this->steer = new CANTalon(steerMotor);
	this->drive = new Talon(driveMotor);
	this->drive->SetInverted(inverted);
	this->InitCANTalon(this->steer);
}

void SwerveModule::InitCANTalon(CANTalon * talon) {
	talon->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	talon->ConfigEncoderCodesPerRev(497);
	talon->SetControlMode(CANTalon::kPosition);
	talon->SetPosition(0);
	talon->SetPID(2.5f, 0.0002f, 0.0005f);
	talon->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	talon->SetAllowableClosedLoopErr(1);
}

SwerveModule::~SwerveModule() {
	delete this->steer;
	delete this->drive;
}

void SwerveModule::Run(float angle, float speed) {
	this->angle = angle;
	this->speed = fabs(speed) > 0.1 ? speed : 0;

	float newAngle = this->steer->Get() - this->BestAngle();
	this->steer->Set(newAngle / 360.f * GR);
	this->drive->Set(this->speed);
}

float SwerveModule::BestAngle() {
	float currentAngle = this->steer->Get() / GR * 360;
	while (currentAngle > 360.f) {
		currentAngle -= 360;
	}
	while (currentAngle < 0.f) {
		currentAngle += 360;
	}
	float diff = currentAngle - angle;
	if (fabs(diff) > 90 && fabs(diff) < 270) {
		angle = fmod(180 + angle, 360);
		speed = -speed;
	}
	// This calculates going cw to the target
	float dist1 = currentAngle - angle;
	// This calculates going ccw to the target
	float dist2 = currentAngle - (360 + angle);
	if (fabs(dist1) < fabs(dist2)) {
		return dist1;
	} else {
		return dist2;
	}
}

void SwerveModule::ResetPosition() {
	this->steer->SetPosition(0);
}

void SwerveModule::Brake() {
	this->drive->Set(0);
}