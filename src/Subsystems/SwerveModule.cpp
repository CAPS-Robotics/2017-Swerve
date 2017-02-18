#include "SwerveModule.h"
#include "../RobotMap.h"
#include <CANTalon.h>

SwerveModule::SwerveModule(int steerMotor, int driveMotor, int encoder, float offset, bool isInverted) : Subsystem("SwerveModule") {
	this->steer = new CANTalon(steerMotor);
	this->steer->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	this->offset = offset;
	this->drive = new Talon(driveMotor);
	this->drive->SetInverted(isInverted);
	this->positionEncoder = new AnalogInput(encoder);
	this->pid = new PIDController(4.0, 0.0, 0.0, this->positionEncoder, this->steer, 0.002);
	this->pid->SetContinuous(true);
	this->pid->SetTolerance(0.01);
	this->pid->SetInputRange(0.0, 5.0);
	this->pid->SetOutputRange(-1.0, 1.0);
	this->pid->Enable();
	currentSpeed = 0;
}

void SwerveModule::InitDefaultCommand() {

}

void SwerveModule::Drive(double speed, double setpoint) {
	speed = fabs(speed) > 0.1 ? speed : 0;
	double currentPos = fmod(this->positionEncoder->GetVoltage() - offset + 5, 5);

	double dist = setpoint - currentPos;

	if (fabs(dist) > 1.25 && fabs(dist) < 3.75) {
		setpoint = fmod(setpoint + 2.5, 5);
		speed *= -1;
	}

	/*if (setpoint >= 1.20 && setpoint <= 1.30) {
		setpoint = 1.25;
	} else if (setpoint >= 2.45 && setpoint <= 2.55) {
		setpoint = 2.50;
	} else if (setpoint >= 3.70 && setpoint <= 3.80) {
		setpoint = 3.75;
	} else if (setpoint <= 0.05 || setpoint >= 4.95) {
		setpoint = 0;
	}*/

	if (speed == 0 || fabs(speed - currentSpeed) > 1.f) {
		currentSpeed = 0;
	} else if (currentSpeed > speed) {
		currentSpeed -= 0.04;
	} else if (currentSpeed < speed) {
		currentSpeed += 0.04;
	}

	SmartDashboard::PutNumber("Distance", dist);

	this->pid->SetSetpoint(fmod(setpoint + offset, 5));
	this->drive->Set(currentSpeed);
}

void SwerveModule::ReturnToZero() {
	this->pid->SetSetpoint(offset);
	SmartDashboard::PutNumber("Setpoint", this->pid->GetSetpoint());
}

double SwerveModule::GetAngle() {
	return fmod(this->positionEncoder->GetVoltage() - offset + 5, 5) * 72.f;
}
