#include "WPILib.h"
#include "config.h"
#include <CANTalon.h>
#include <cmath>
#include "Robot.h"
#include "SwerveModule.h"

/**
 * 		Note to self: Postive on steer motors is ccw
 */

Swerve::Swerve() {
	this->joystick = new Joystick(0);
	this->fl = new SwerveModule(FL_CAN, FL_PWM, true);
	this->fr = new SwerveModule(FR_CAN, FR_PWM, false);
	this->bl = new SwerveModule(BL_CAN, BL_PWM, true);
	this->br = new SwerveModule(BR_CAN, BR_PWM, false);
}

void Swerve::RobotInit() {

}



void Swerve::Autonomous() {


}

void Swerve::OperatorControl() {
	float fla = 0, fra = 0, bla = 0, bra = 0;
	double back, front, right, left;
	// SmartDashboard::PutNumber("Encoder Val", 1/1.2);
	// SmartDashboard::PutNumber("P", 2.5);
	// SmartDashboard::PutNumber("I", 0.0002);
	// SmartDashboard::PutNumber("D", 0.0005);

	while (IsOperatorControl() && IsEnabled()) {
		double strafe = this->Deadband(-this->joystick->GetRawAxis(0));
		double forward = this->Deadband(-this->joystick->GetRawAxis(1));
		double rotation = this->Deadband(this->joystick->GetRawAxis(2));

		SmartDashboard::PutNumber("Angle fl", fmod(fls->Get() * 360, 360));
		SmartDashboard::PutNumber("Angle fr", fmod(frs->Get() * 360, 360));
		SmartDashboard::PutNumber("Angle bl", fmod(bls->Get() * 360, 360));
		SmartDashboard::PutNumber("Angle br", fmod(brs->Get() * 360, 360));
		double flds = 0, frds = 0, blds = 0, brds = 0;
		if (this->joystick->GetRawButton(7)) {
			fl->Run(0, 0);
			bl->Run(0, 0);
			fr->Run(0, 0);
			br->Run(0, 0);
		}

		if (this->joystick->GetRawButton(JOY_BTN_X)) {
			fl->Brake();
			fr->Brake();
			bl->Brake();
			br->Brake();
		}

		if (this->joystick->GetRawButton(JOY_SPC_BCK) && this->joystick->GetRawButton(JOY_SPC_STR)) {
			fls->ResetPosition();
			frs->ResetPosition();
			bls->ResetPosition();
			brs->ResetPosition();
		}


		if (fabs(forward) != 0 || fabs(strafe) != 0 || fabs(rotation) != 0) {
			if (fabs(rotation) != 0) {
				back = strafe - rotation * pow(2, -0.5);
				front = strafe + rotation * pow(2, -0.5);
				right = forward - rotation * pow(2, -0.5);
				left = forward + rotation * pow(2, -0.5);
			} else {
				back = strafe;
				front = strafe;
				right = forward;
				left = forward;
			}

			flds = sqrt(front * front + left * left);
			frds = sqrt(front * front + right * right);
			blds = sqrt(back * back + left * left);
			brds = sqrt(back * back + right * right);
			double maxSpeed = std::max(std::max(std::max(flds, frds), blds), brds);
			if (maxSpeed > 1) {
				flds /= maxSpeed;
				frds /= maxSpeed;
				blds /= maxSpeed;
				brds /= maxSpeed;
			}

			if (front == 0 && left == 0) {
				fla = 0;
			} else {
				fla = atan2(front, left) * 180 / PI;
				fla = fmod((-fla + 360 + 90), 360);
			}

			if (front == 0 && right == 0) {
				fra = 0;
			} else {
				fra = atan2(front, right) * 180 / PI;
				fra = fmod((-fra + 360 + 90), 360);
			}

			if (back == 0 && left == 0) {
				bla = 0;
			} else {
				bla = atan2(back, left) * 180 / PI;
				bla = fmod((-bla + 360 + 90), 360);
			}

			if (back == 0 && right == 0) {
				bra = 0;
			} else {
				bra = atan2(back, right) * 180 / PI;
				bra = fmod((-bra + 360 + 90), 360);
			}

			this->fl->Run(fla, flds);
			this->fr->Run(fra, frds);
			this->bl->Run(bla, blds);
			this->br->Run(bra, brds);
		} else {
			this->fl->Brake();
			this->fr->Brake();
			this->bl->Brake();
			this->br->Brake();
		}
	}
}

float Swerve::Deadband(float val) {
	if (fabs(val) <= 0.05) {
		return 0;
	}
	return val;
}

START_ROBOT_CLASS(Swerve);
