#include "Drivetrain.h"
#include "../Robot.h"
#include "../RobotMap.h"
#include "SwerveModule.h"

Drivetrain::Drivetrain() : Subsystem("Drivetrain") {
	Robot::gyro.get();
	this->fl = new SwerveModule(FL_TALON_SRX, FL_DRIVE_TALON, true);
	this->fr = new SwerveModule(FR_TALON_SRX, FR_DRIVE_TALON, false);
	this->bl = new SwerveModule(BL_TALON_SRX, BL_DRIVE_TALON, true);
	this->br = new SwerveModule(BR_TALON_SRX, BR_DRIVE_TALON, false);
	this->rangeFinder = new AnalogInput(RANGE_FINDER);
	this->currentMode = Drivetrain::ControlMode::fieldOrientedSwerve;
}

void Drivetrain::InitDefaultCommand() {

}

void Drivetrain::SetControlMode(Drivetrain::ControlMode cm) {
	this->currentMode = cm;
}

double Drivetrain::GetDistanceAway() {
	return this->rangeFinder->GetVoltage() / 0.012446;
}

void Drivetrain::Drive(double x, double y, double rotation, double speedMultiplier) {
	switch (this->currentMode) {
	case Drivetrain::ControlMode::fieldOrientedSwerve:
	case Drivetrain::ControlMode::swerve:
		this->CrabDrive(x, y, rotation, speedMultiplier);
		break;
	case Drivetrain::ControlMode::arcade:
		this->ArcadeDrive(y, rotation, speedMultiplier);
		break;
	};
}

void Drivetrain::Brake() {
	this->fl->Drive(0, 0);
	this->fr->Drive(0, 0);
	this->bl->Drive(0, 0);
	this->br->Drive(0, 0);
}

void Drivetrain::ArcadeDrive(double forward, double rotation, double speedMultiplier) {
	this->fl->Drive((forward + rotation * 0.5) * speedMultiplier, 0);
	this->fr->Drive((forward - rotation * 0.5) * speedMultiplier, 0);
	this->bl->Drive((forward + rotation * 0.5) * speedMultiplier, 0);
	this->br->Drive((forward - rotation * 0.5) * speedMultiplier, 0);
}

void Drivetrain::CrabDrive(double x, double y, double rotation, double speedMultiplier) {
	double heading = Robot::gyro->GetHeading();

	double forward, strafe;

	if (this->currentMode == Drivetrain::ControlMode::fieldOrientedSwerve) {
		forward = -x * sin(heading * PI / 180) + y * cos(heading * PI / 180);
		strafe  =  x * cos(heading * PI / 180) + y * sin(heading * PI / 180);
	} else {
		forward = y;
		strafe  = x;
	}

	double back, front, right, left;

	if (rotation != 0) {
		back  = strafe  - rotation * 0.5;
		front = strafe  + rotation * 0.5;
		right = forward - rotation * 0.5;
		left  = forward + rotation * 0.5;
	} else {
		back  = strafe;
		front = strafe;
		right = forward;
		left  = forward;
	}

	double flds = sqrt(front * front + left  * left);
	double frds = sqrt(front * front + right * right);
	double blds = sqrt(back  * back  + left  * left);
	double brds = sqrt(back  * back  + right * right);

	double maxSpeed = std::max(std::max(std::max(flds, frds), blds), brds);
	if (maxSpeed > 1) {
		flds /= maxSpeed;
		frds /= maxSpeed;
		blds /= maxSpeed;
		brds /= maxSpeed;
	}

	double fla = fmod(-(atan2(front, left)  * 180 / PI) + 360, 360);
	double fra = fmod(-(atan2(front, right) * 180 / PI) + 360, 360);
	double bla = fmod(-(atan2(back,  left)  * 180 / PI) + 360, 360);
	double bra = fmod(-(atan2(back,  front) * 180 / PI) + 360, 360);

	this->fl->Drive(flds * speedMultiplier, fla);
	this->fr->Drive(frds * speedMultiplier, fra);
	this->bl->Drive(blds * speedMultiplier, bla);
	this->br->Drive(brds * speedMultiplier, bra);
}
