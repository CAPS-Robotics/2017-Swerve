#include <Subsystems/Climber.h>
#include "../RobotMap.h"
#include <CANTalon.h>

Climber::Climber() : Subsystem("Climber") {
	this->climber1 = new CANTalon(CLIMBER1_TALON_SRX);
	this->climber2 = new CANTalon(CLIMBER2_TALON_SRX);
}

void Climber::SetClimberSpeed(double speed) {
	this->climber1->Set(speed);
	this->climber2->Set(speed);
}

void Climber::StopClimber() {
	this->climber1->Set(0);
	this->climber2->Set(0);
}

void Climber::InitDefaultCommand() {

}

CANTalon * Climber::GetClimber1() {
	return this->climber1;
}

double Climber::Get1Current() {
	return this->climber1->GetOutputCurrent();
}

double Climber::Get2Current() {
	return this->climber2->GetOutputCurrent();
}
