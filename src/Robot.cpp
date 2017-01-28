#include "WPILib.h"
#include "config.h"
#include <CANTalon.h>
#include <cmath>
#include "Robot.h"

/**
 * 		Note to self: Postive on steer motors is ccw
 */

Swerve::Swerve() {
	this->joystick = new Joystick(0);
	this->fls = new CANTalon(FL_CAN);
	this->bls = new CANTalon(BL_CAN);
	this->frs = new CANTalon(FR_CAN);
	this->brs = new CANTalon(BR_CAN);

	this->fld = new Talon(FL_PWM);
	this->bld = new Talon(BL_PWM);
	this->frd = new Talon(FR_PWM);
	this->brd = new Talon(BR_PWM);
}

void Swerve::RobotInit() {
	this->InitTalon(this->fls);
	this->InitTalon(this->bls);
	this->InitTalon(this->frs);
	this->InitTalon(this->brs);

	this->fld->SetInverted(true);
	this->bld->SetInverted(true);
}

void Swerve::InitTalon(CANTalon * talon) {
	talon->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	talon->ConfigEncoderCodesPerRev(497);
	talon->SetControlMode(CANTalon::kPosition);
	talon->SetPosition(0);
	talon->SetPID(2.5f, 0.0002f, 0.0005f);
	talon->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	talon->SetAllowableClosedLoopErr(1);
}

void Swerve::Autonomous() {


}

void Swerve::OperatorControl() {
	float fla = 0, fra = 0, bla = 0, bra = 0;
	double back, front, right, left;
		SmartDashboard::PutNumber("Encoder Val", 1/1.2);
		SmartDashboard::PutNumber("P", 2.5);
		SmartDashboard::PutNumber("I", 0.0002);
		SmartDashboard::PutNumber("D", 0.0005);

	while (IsOperatorControl() && IsEnabled()) {
		double strafe = this->Deadband(-this->joystick->GetRawAxis(0));
		double forward = this->Deadband(-this->joystick->GetRawAxis(1));
		double rotation = this->Deadband(this->joystick->GetRawAxis(2));

		SmartDashboard::PutNumber("Angle fl", fmod(fls->Get() * 360, 360));
		SmartDashboard::PutNumber("Angle fr", fmod(frs->Get() * 360, 360));
		SmartDashboard::PutNumber("Angle bl", fmod(bls->Get() * 360, 360));
		SmartDashboard::PutNumber("Angle br", fmod(brs->Get() * 360, 360));
//		double flds = 0, frds = 0, blds = 0, brds = 0;
		if (this->joystick->GetRawButton(7)) {
			fls->Set(0);
			bls->Set(0);
			frs->Set(0);
			brs->Set(0);
		}

		if (this->joystick->GetRawButton(JOY_BTN_X)) {
			fld->Set(0);
			frd->Set(0);
			bld->Set(0);
			brd->Set(0);
		}

		if (this->joystick->GetRawButton(JOY_SPC_BCK) && this->joystick->GetRawButton(JOY_SPC_STR)) {
			fls->SetPosition(0);
			frs->SetPosition(0);
			bls->SetPosition(0);
			brs->SetPosition(0);
		}

		if ((fabs(forward) != 0 || fabs(strafe) != 0) && fabs(rotation) == 0) {
			float angle = atan2(forward, strafe) * 180 / PI;
			angle = fmod((-angle + 450), 360);
			float speed = sqrt(strafe * strafe + forward * forward);
			fls->Set(angle/360 * GR);
			frs->Set(angle/360 * GR);
			bls->Set(angle/360 * GR);
			brs->Set(angle/360 * GR);

			if (fabs(speed) <= 0.1) {
				speed = 0;
			}

			fld->Set(speed);
			frd->Set(speed);
			bld->Set(speed);
			brd->Set(speed);
		} else if (fabs(rotation) != 0) {
			fls->Set(-45.0 / 360 * GR);
			frs->Set(45.0 / 360 * GR);
			bls->Set(45.0 / 360 * GR);
			brs->Set(-45.0 / 360 * GR);

			fld->Set(rotation);
			frd->Set(-rotation);
			bld->Set(rotation);
			brd->Set(-rotation);
		} else {
			fld->Set(0);
			frd->Set(0);
			bld->Set(0);
			brd->Set(0);
		}


//		if (fabs(forward) != 0 || fabs(strafe) != 0 || fabs(rotation) != 0) {
//			if (fabs(rotation) != 0) {
//				back = strafe - rotation * pow(2, -0.5);
//				front = strafe + rotation * pow(2, -0.5);
//				right = forward - rotation * pow(2, -0.5);
//				left = forward + rotation * pow(2, -0.5);
//			} else {
//				back = strafe;
//				front = strafe;
//				right = forward;
//				left = forward;
//			}
//
//			flds = sqrt(front * front + left * left);
//			frds = sqrt(front * front + right * right);
//			blds = sqrt(back * back + left * left);
//			brds = sqrt(back * back + right * right);
//			double maxSpeed = std::max(std::max(std::max(flds, frds), blds), brds);
//			if (maxSpeed > 1) {
//				flds /= maxSpeed;
//				frds /= maxSpeed;
//				blds /= maxSpeed;
//				brds /= maxSpeed;
//			}
//
//			if (front == 0 && left == 0) {
//				fla = 0;
//			} else {
//				fla = atan2(front, left) * 180 / PI;
//				fla = fmod((-fla + 360 + 90), 360);
//			}
//
//			if (front == 0 && right == 0) {
//				fra = 0;
//			} else {
//				fra = atan2(front, right) * 180 / PI;
//				fra = fmod((-fra + 360 + 90), 360);
//			}
//
//			if (back == 0 && left == 0) {
//				bla = 0;
//			} else {
//				bla = atan2(back, left) * 180 / PI;
//				bla = fmod((-bla + 360 + 90), 360);
//			}
//
//			if (back == 0 && right == 0) {
//				bra = 0;
//			} else {
//				bra = atan2(back, right) * 180 / PI;
//				bra = fmod((-bra + 360 + 90), 360);
//			}
//
//			fls->Set(-fla/360.0 * GR);
//			frs->Set(-fra/360.0 * GR);
//			bls->Set(-bla/360.0 * GR);
//			brs->Set(-bra/360.0 * GR);
//
//			if (flds > 0.20)
//				fld->Set(flds);
//			else
//				fld->Set(0);
//
//			if (frds > 0.20)
//				frd->Set(frds);
//			else
//				frd->Set(0);
//
//			if (blds > 0.20)
//				bld->Set(blds);
//			else
//				bld->Set(0);
//
//			if (brds > 0.20)
//				brd->Set(brds);
//			else
//				brd->Set(0);
//		}
	}
}

float Swerve::Deadband(float val) {
	if (fabs(val) <= 0.05) {
		return 0;
	}
	return val;
}

START_ROBOT_CLASS(Swerve);
