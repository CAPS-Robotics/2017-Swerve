#include "Drivetrain.h"
#include "../Robot.h"
#include "../RobotMap.h"
#include "SwerveModule.h"
#include <Commands/Drivetrain/DriveWithJoysticks.h>

Drivetrain::Drivetrain() : Subsystem("Drivetrain") {
	Robot::gyro.get();
	this->fl = new SwerveModule(FL_TALON_SRX, FL_DRIVE_TALON, true);
	this->fr = new SwerveModule(FR_TALON_SRX, FR_DRIVE_TALON, false);
	this->bl = new SwerveModule(BL_TALON_SRX, BL_DRIVE_TALON, true);
	this->br = new SwerveModule(BR_TALON_SRX, BR_DRIVE_TALON, false);
	this->rangeFinder = new AnalogInput(RANGE_FINDER);
	this->currentMode = Drivetrain::ControlMode::fieldOrientedSwerve;
	this->desiredHeading = 0;
}

void Drivetrain::InitDefaultCommand() {
	SetDefaultCommand(new DriveWithJoysticks());
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
	SmartDashboard::PutNumber("FL Angle", fl->GetAngle());
	SmartDashboard::PutNumber("FR Angle", fr->GetAngle());
	SmartDashboard::PutNumber("BL Angle", bl->GetAngle());
	SmartDashboard::PutNumber("BR Angle", br->GetAngle());

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
	this->fl->Drive((forward + rotation * 1) * speedMultiplier, 0);
	this->fr->Drive((forward - rotation * 1) * speedMultiplier, 0);
	this->bl->Drive((forward + rotation * 1) * speedMultiplier, 0);
	this->br->Drive((forward - rotation * 1) * speedMultiplier, 0);
}

void Drivetrain::CrabDrive(double x, double y, double rotation, double speedMultiplier) {
	double forward, strafe;
	if (x != 0 || y != 0 || rotation != 0) {
		if (this->currentMode == Drivetrain::ControlMode::fieldOrientedSwerve) {
			double heading = Robot::gyro->GetHeading();
			forward = -x * sin(heading * PI / 180) + y * cos(heading * PI / 180);
			strafe  =  x * cos(heading * PI / 180) + y * sin(heading * PI / 180);
		} else {
			forward = y;
			strafe  = x;
		}

		double back, front, right, left;

		if (rotation != 0) {
			back  = strafe  - rotation * 1;
			front = strafe  + rotation * 1;
			right = forward - rotation * 1;
			left  = forward + rotation * 1;
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
			if (front != 0 || left != 0)
				fla = fmod(-(atan2(front, left)  * 180 / PI) + 360, 360);
			if (front != 0 || right != 0)
				fra = fmod(-(atan2(front, right) * 180 / PI) + 360, 360);
			if (back != 0 || left != 0)
				bla = fmod(-(atan2(back,  left)  * 180 / PI) + 360, 360);
			if (back != 0 || right != 0)
				bra = fmod(-(atan2(back,  right) * 180 / PI) + 360, 360);
		} else {
			double diff = Robot::gyro->GetHeading() - desiredHeading;
			SmartDashboard::PutNumber("Difference", diff);
			if (front != 0 || left != 0)
				fla = fmod(-(atan2(front, left)  * 180 / PI) + 360 - diff, 360);
			if (front != 0 || right != 0)
				fra = fmod(-(atan2(front, right) * 180 / PI) + 360 - diff, 360);
			if (back != 0 || left != 0)
				bla = fmod(-(atan2(back,  left)  * 180 / PI) + 360 - diff, 360);
			if (back != 0 || right != 0)
				bra = fmod(-(atan2(back,  right) * 180 / PI) + 360 - diff, 360);
		}

		SmartDashboard::PutNumber("Desired Heading", desiredHeading);

		this->fl->Drive(flds * speedMultiplier, fla);
		this->fr->Drive(frds * speedMultiplier, fra);
		this->bl->Drive(blds * speedMultiplier, bla);
		this->br->Drive(brds * speedMultiplier, bra);
	} else {
		this->fl->Drive(0, this->fl->GetAngle());
		this->fr->Drive(0, this->fr->GetAngle());
		this->bl->Drive(0, this->bl->GetAngle());
		this->br->Drive(0, this->br->GetAngle());
	}
}
