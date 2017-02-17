#include "Drivetrain.h"
#include "../Robot.h"
#include "../RobotMap.h"
#include "SwerveModule.h"
#include <Commands/Drivetrain/DriveWithJoysticks.h>

Drivetrain::Drivetrain() : Subsystem("Drivetrain") {
	Robot::gyro.get();
	this->fl = new SwerveModule(FL_TALON_SRX, FL_DRIVE_TALON, FL_STEER_ENCODER, 4.7485, true);
	this->fr = new SwerveModule(FR_TALON_SRX, FR_DRIVE_TALON, FR_STEER_ENCODER, 3.1104, false);
	this->bl = new SwerveModule(BL_TALON_SRX, BL_DRIVE_TALON, BL_STEER_ENCODER, 2.4633, true);
	this->br = new SwerveModule(BR_TALON_SRX, BR_DRIVE_TALON, BR_STEER_ENCODER, 2.0386, false);
	this->rangeFinder = new AnalogInput(RANGE_FINDER);
	this->desiredHeading = 0;
}

void Drivetrain::InitDefaultCommand() {
	SetDefaultCommand(new DriveWithJoysticks());
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

void Drivetrain::Brake() {
	this->fl->Drive(0, 0);
	this->fr->Drive(0, 0);
	this->bl->Drive(0, 0);
	this->br->Drive(0, 0);
}

void Drivetrain::Drive(double angle, double speed, double speedMultiplier) {
	this->fl->Drive(speed * speedMultiplier, angle);
	this->fr->Drive(speed * speedMultiplier, angle);
	this->bl->Drive(speed * speedMultiplier, angle);
	this->br->Drive(speed * speedMultiplier, angle);
}

void Drivetrain::ArcadeDrive(double forward, double rotation, double speedMultiplier) {
	double correction = 0.025 * (Robot::gyro->GetHeading() - desiredHeading);
	SmartDashboard::PutNumber("Difference", correction);
	this->fl->Drive((forward + rotation * 1 / sqrt(2) + correction) * speedMultiplier, 0);
	this->fr->Drive((forward - rotation * 1 / sqrt(2) - correction) * speedMultiplier, 0);
	this->bl->Drive((forward + rotation * 1 / sqrt(2) + correction) * speedMultiplier, 0);
	this->br->Drive((forward - rotation * 1 / sqrt(2) - correction) * speedMultiplier, 0);
}

void Drivetrain::CrabDrive(double x, double y, double rotation, double speedMultiplier, bool useGyro) {
	double forward, strafe;
	SmartDashboard::PutNumber("FL Angle", fl->GetAngle());
	SmartDashboard::PutNumber("FR Angle", fr->GetAngle());
	SmartDashboard::PutNumber("BL Angle", bl->GetAngle());
	SmartDashboard::PutNumber("BR Angle", br->GetAngle());
	if (useGyro) {
		double heading = Robot::gyro->GetHeading();
		forward = -x * sin(heading * PI / 180) + y * cos(heading * PI / 180);
		strafe  =  x * cos(heading * PI / 180) + y * sin(heading * PI / 180);
	} else {
		forward = y;
		strafe  = x;
	}
	if (x != 0 || y != 0 || rotation != 0) {
		double back, front, right, left;

		if (rotation != 0) {
			back  = strafe  - rotation * 1.0 / sqrt(2);
			front = strafe  + rotation * 1.0 / sqrt(2);
			right = forward - rotation * 1.0 / sqrt(2);
			left  = forward + rotation * 1.0 / sqrt(2);
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

		if (rotation != 0) {
			desiredHeading = Robot::gyro->GetHeading();
		}

		if (front != 0 || left != 0) {
			fla = fmod(5 + (2.5 / PI) * -atan2(front, left),  5);
		} else {
			fla = 0;
		}
		if (front != 0 || right != 0) {
			fra = fmod(5 + (2.5 / PI) * -atan2(front, right), 5);
		} else {
			fra = 0;
		}
		if (back != 0 || left != 0) {
			bla = fmod(5 + (2.5 / PI) * -atan2(back,  left),  5);
		} else {
			bla = 0;
		}
		if (back != 0 || right != 0) {
			bra = fmod(5 + (2.5 / PI) * -atan2(back,  right), 5);
		} else {
			bra = 0;
		}
		SmartDashboard::PutNumber("Desired Heading", desiredHeading);

		double correction = 0; //-0.0025 * (Robot::gyro->GetHeading() - desiredHeading);
		SmartDashboard::PutNumber("Difference", correction);
		this->fl->Drive((flds + correction) * speedMultiplier, fla);
		this->fr->Drive((frds - correction) * speedMultiplier, fra);
		this->bl->Drive((blds + correction) * speedMultiplier, bla);
		this->br->Drive((brds - correction) * speedMultiplier, bra);
	} else {
		this->fl->Drive(0, this->fl->GetAngle() / 72.f);
		this->fr->Drive(0, this->fr->GetAngle() / 72.f);
		this->bl->Drive(0, this->bl->GetAngle() / 72.f);
		this->br->Drive(0, this->br->GetAngle() / 72.f);
	}
}
