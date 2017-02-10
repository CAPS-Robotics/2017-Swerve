#include "SwerveModule.h"
#include "../RobotMap.h"
#include <CANTalon.h>

SwerveModule::SwerveModule(int steerMotor, int driveMotor, int encoder, bool isInverted) : Subsystem("SwerveModule") {
	this->steer = new CANTalon(steerMotor);
	this->steer->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	this->drive = new Talon(driveMotor);
	this->drive->SetInverted(isInverted);
	this->positionEncoder = new AnalogInput(encoder);
	this->pid = new PIDController(1.0, 0.0, 0.0, this->positionEncoder, this->steer, 0.02);
	this->pid->SetContinuous(true);
	this->pid->SetAbsoluteTolerance(0.1);
	this->pid->SetInputRange(0.0, 5.0);
	this->pid->SetOutputRange(-1.0, 1.0);
	//this->pid->Enable();
}

void SwerveModule::InitDefaultCommand() {

}

void SwerveModule::Drive(double speed, double angle) {
	speed = fabs(speed) > 0.1 ? speed : 0;

	double setpoint = angle / 360.f * 5;

	double dist = setpoint - this->positionEncoder->GetVoltage();

	if (fabs(dist) < 1.25 || fabs(dist) > 3.75) {
		setpoint = setpoint + 2.5;
		speed = -speed;
	}

	this->pid->SetSetpoint(setpoint);
	this->drive->Set(speed);
}

void SwerveModule::ReturnToZero() {
	this->pid->SetSetpoint(0);
}

double SwerveModule::GetAngle() {
	return this->positionEncoder->GetVoltage() * 360 / 5;
}
