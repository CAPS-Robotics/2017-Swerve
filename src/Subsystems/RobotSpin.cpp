#include "../RobotMap.h"
#include "../Robot.h"
#include <PigeonImu.h>
#include <Subsystems/RobotSpin.h>

RobotSpin::RobotSpin() : output(0) {
	Robot::drivetrain.get();
}

double RobotSpin::Get() {
	return output;
}

void RobotSpin::PIDWrite(double output) {
	this->output = output;
	Robot::drivetrain->RotateRobot(output);
}