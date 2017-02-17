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
	steer->Enable();
	currentSpeed = 0;
	steer->Set(0);
}

void SwerveModule::InitDefaultCommand() {

}

void SwerveModule::Drive(double speed, double angle) {
	speed = fabs(speed) > 0.1 ? speed : 0;

	double dist = angle - this->GetAngle();

	SmartDashboard::PutNumber("Dist", dist);

	if (fabs(dist) > 90 && fabs(dist) < 270) {
		angle = fmod(180 + angle, 360);
		speed = -speed;
	}/* else if (dist > 270) {
		angle = 360 - angle;
	} else if (dist < -270) {
		angle = angle + 360;
	}*/

	if (speed == 0 || fabs(speed - currentSpeed) > 1.2f) {
		currentSpeed = 0;
	} else if (currentSpeed > speed) {
		currentSpeed -= 0.08;
	} else if (currentSpeed < speed) {
		currentSpeed += 0.08;
	}

	this->drive->Set(currentSpeed);
	this->steer->Set(angle / 1.2 / 360);
}

void SwerveModule::ReturnToZero() {
	this->steer->Set(0);
}

void SwerveModule::ResetEncoder() {
	this->steer->SetPosition(0);
}

double SwerveModule::GetAngle() {
	float someAngle = this->steer->Get() * 1.2 * 360;
	while (someAngle > 360.f) {
		someAngle -= 360;
	}
	while (someAngle < 0.f) {
		someAngle += 360;
	}
	return someAngle;
}
