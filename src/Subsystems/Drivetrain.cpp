#include "Drivetrain.h"
#include "../Robot.h"
#include "../RobotMap.h"
#include "SwerveModule.h"
#include <Commands/Drivetrain/DriveWithJoysticks.h>

Drivetrain::Drivetrain() : Subsystem("Drivetrain") {
	Robot::gyro.get();
	this->fl = new SwerveModule(FL_TALON_SRX, FL_DRIVE_TALON, FL_STEER_ENCODER, true);
	this->fr = new SwerveModule(FR_TALON_SRX, FR_DRIVE_TALON, FR_STEER_ENCODER, false);
	this->bl = new SwerveModule(BL_TALON_SRX, BL_DRIVE_TALON, BL_STEER_ENCODER, true);
	this->br = new SwerveModule(BR_TALON_SRX, BR_DRIVE_TALON, BR_STEER_ENCODER, false);
	this->rangeFinder = new AnalogInput(RANGE_FINDER);
	this->currentMode = Drivetrain::ControlMode::fieldOrientedSwerve;
}

void Drivetrain::InitDefaultCommand() {
	//SetDefaultCommand(new DriveWithJoysticks());
}

void Drivetrain::SetControlMode(Drivetrain::ControlMode cm) {
	this->currentMode = cm;
}

double Drivetrain::GetDistanceAway() {
	return this->rangeFinder->GetVoltage() / 0.012446;
}

void Drivetrain::ReturnWheelsToZero() {
	this->fl->ReturnToZero();
	this->fr->ReturnToZero();
	this->bl->ReturnToZero();
	this->br->ReturnToZero();
}

void Drivetrain::Drive(double x, double y, double rotation, double speedMultiplier) {
	SmartDashboard::PutNumber("FL Voltage", fl->positionEncoder->GetAverageVoltage());
	SmartDashboard::PutNumber("FR Voltage", fr->positionEncoder->GetAverageVoltage());
	SmartDashboard::PutNumber("BL Voltage", bl->positionEncoder->GetAverageVoltage());
	SmartDashboard::PutNumber("BR Voltage", br->positionEncoder->GetAverageVoltage());

	SmartDashboard::PutNumber("FL Angle", fl->GetAngle());
	SmartDashboard::PutNumber("FR Angle", fr->GetAngle());
	SmartDashboard::PutNumber("BL Angle", bl->GetAngle());
	SmartDashboard::PutNumber("BR Angle", br->GetAngle());

	SmartDashboard::PutNumber("FL Setpoint", fl->pid->GetSetpoint());
	SmartDashboard::PutNumber("FR Setpoint", fr->pid->GetSetpoint());
	SmartDashboard::PutNumber("BL Setpoint", bl->pid->GetSetpoint());
	SmartDashboard::PutNumber("BR Setpoint", br->pid->GetSetpoint());

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

	double fla = 0, fra = 0, bla = 0, bra = 0;

	if (front != 0 || left != 0)
		fla = fmod(-(atan2(front, left)  * 180 / PI) + 360, 360);
	if (front != 0 || right != 0)
		fra = fmod(-(atan2(front, right) * 180 / PI) + 360, 360);
	if (back != 0 || left != 0)
		bla = fmod(-(atan2(back,  left)  * 180 / PI) + 360, 360);
	if (back != 0 || right != 0)
		bra = fmod(-(atan2(back,  right) * 180 / PI) + 360, 360);

	this->fl->Drive(flds * speedMultiplier, fla);
	this->fr->Drive(frds * speedMultiplier, fra);
	this->bl->Drive(blds * speedMultiplier, bla);
	this->br->Drive(brds * speedMultiplier, bra);
}
