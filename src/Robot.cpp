#include "WPILib.h"
#include "config.h"
#include <CANTalon.h>
#include <PigeonImu.h>
#include <cmath>
#include "Robot.h"
#include "SwerveModule.h"
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * 		Note to self: Postive on steer motors is ccw
 *
 * 	Controls: (will work for either joystick)
 * 		Joystick:
 * 			XY to swerve, Twist to rotate (huge deadband)
 * 			Slider is a speed multiplier (down is full speed)
 * 			Button 1 (trigger) is a parking brake
 * 			Button 2 (thumb) sets wheels back to home
 * 			Button 3 (on top) to climb
 * 			Button 7 & 8 (on base) sets current position as 0 (don't use)
 *
 * 		Gamepad:
 * 			Left XY to swerve, right x-axis to rotate (huge deadband)
 * 			Button X is a parking brake
 * 			Button A sets wheels back to home
 * 			Button B to climb
 * 			Both triggers together sets current position as 0 (don't use)
 *
 * 	Things to double check:
 * 		Make sure the climber motors are wired as directed:
 * 			If not, change the config file
 * 			CAN starts at the roborio and ends at the PDP
 * 		Try adjusting the deadzones (check lines 99-101)
 *
 *	How to push code:
 *		1. Save your code
 *		2. Make sure you are connected to the robot
 *		3. Right click on BetterSwerve (on the left)
 *		4. Select Run As > 2 WPILib C++ Deploy
 *		5. Run the robot
 *		6. If it fails, fix your errors
 */

// Creating our variables
Swerve::Swerve() {
	this->joystick = new Joystick(0);
	this->fl = new SwerveModule(FL_CAN, FL_PWM, true);
	this->fr = new SwerveModule(FR_CAN, FR_PWM, false);
	this->bl = new SwerveModule(BL_CAN, BL_PWM, true);
	this->br = new SwerveModule(BR_CAN, BR_PWM, false);
	this->redLeds = new PWM(12);
	this->greenLeds = new PWM(11);
	this->blueLeds = new PWM(10);
	this->climber = new CANTalon(CLIMB_CAN);
	this->climber2 = new CANTalon(CLIMB2_CAN);
	this->pigeon = new PigeonImu(climber);
	this->rangeFinder = new AnalogInput(0);
}

// Inits gyro since we aren't using it
void Swerve::RobotInit() {
	//std::thread vision(Swerve::VisionThread);
	//vision.detach();
	pigeon->SetFusedHeading(0);
}

// This auto currently drives forward at 50% power
void Swerve::Autonomous() {
	fl->Run(0, 0.5);
	fr->Run(0, 0.5);
	bl->Run(0, 0.5);
	br->Run(0, 0.5);

}

// Doesn't run yet. Don't worry about it
void Swerve::VisionThread() {
	cs::UsbCamera cam = CameraServer::GetInstance()->StartAutomaticCapture();
	cam.SetResolution(320, 240);
	cam.SetBrightness(10);
	cs::CvSink vid = CameraServer::GetInstance()->GetVideo();
	cs::CvSource output = CameraServer::GetInstance()->PutVideo("Contours", 320, 240);
	cv::Mat src, hsv, final;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	while (true) {
		vid.GrabFrame(src);
		cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
		cv::inRange(hsv, cv::Scalar(60, 150, 100), cv::Scalar(90, 255, 255), final);
		output.PutFrame(final);
	}
}

void Swerve::OperatorControl() {
	// Angle variables
	float fla = 0, fra = 0, bla = 0, bra = 0;
	double back, front, right, left;
	double heading = 0;
	while (IsOperatorControl() && IsEnabled()) {
		// LEDs don't work so who cares, I'm not deleting it
		this->SetRGB(0, 0, 192);

		// Reading joystick input after applying a deadband
		double lx = this->Deadband(this->joystick->GetRawAxis(0), 0.15);
		double ly = this->Deadband(-this->joystick->GetRawAxis(1), 0.15);
		double rotation = this->Deadband(this->joystick->GetRawAxis(2), 0.70);
		// Printing out joystick values
		SmartDashboard::PutNumber("Twist", rotation);
		SmartDashboard::PutNumber("Heading", heading);
		heading = fmod(pigeon->GetFusedHeading(), 360.0);
		// Speed multiplier for the memes
		double speedMultiplier = (1 - this->joystick->GetRawAxis(3)) / 2;
		double forward = ly * cos(heading * PI / 180) - lx * sin(heading * PI / 180);
		double strafe =  ly * sin(heading * PI / 180) + lx * cos(heading * PI / 180);

		SmartDashboard::PutNumber("Ultrasonic Voltage", this->rangeFinder->GetVoltage());
		SmartDashboard::PutNumber("Ultrasonic Distance", this->rangeFinder->GetVoltage() / 0.012446);

		SmartDashboard::PutNumber("X-Value", strafe);
		SmartDashboard::PutNumber("Y-Value", forward);
		// Reading out encoder values
		SmartDashboard::PutNumber("Angle fl", fl->GetEnc() * 180 / 120 * 360);
		SmartDashboard::PutNumber("Angle fr", fr->GetEnc() * 180 / 120 * 360);
		SmartDashboard::PutNumber("Angle bl", bl->GetEnc() * 180 / 120 * 360);
		SmartDashboard::PutNumber("Angle br", br->GetEnc() * 180 / 120 * 360);

		// Reading out current
		SmartDashboard::PutNumber("Climber Current", this->climber->GetOutputCurrent());
		SmartDashboard::PutNumber("Climber Temp", this->climber->GetTemperature());
		SmartDashboard::PutNumber("Climber 2 Current", this->climber2->GetOutputCurrent());

		// Speed variables
		double flds = 0, frds = 0, blds = 0, brds = 0;

		// If button 2 pressed, return wheels to zero
		if (this->joystick->GetRawButton(2)) {
			fl->Zero();
			bl->Zero();
			fr->Zero();
			br->Zero();
		}

		// If button 9 pressed, reset gyro
		if (this->joystick->GetRawButton(6)) {
			pigeon->SetFusedHeading(0);
		}

		// Press buttons 7 and 8 to zero the encoders
		if (this->joystick->GetRawButton(7) && this->joystick->GetRawButton(8)) {
			fl->ResetPosition();
			fr->ResetPosition();
			bl->ResetPosition();
			br->ResetPosition();
		}

		// If button 3 pressed and no crazy currents, climb
		if (this->joystick->GetRawButton(3) && this->climber->GetOutputCurrent() <= 45 && this->climber2->GetOutputCurrent() <= 45) {
			this->climber->Set(1);
			this->climber2->Set(-1);
		} else {
			this->climber->Set(0);
			this->climber2->Set(0);
		}

		// If button 4 pressed, or super current, don't climb
		if (this->joystick->GetRawButton(4) || this->climber->GetOutputCurrent() > 45 || this->climber2->GetOutputCurrent() > 45) {
			this->climber->Set(0);
			this->climber2->Set(0);
		}

		// If button 1 pressed, apply the brake
		if (this->joystick->GetRawButton(1)) {
			fl->Run(0, forward + rotation * 0.5);
			fr->Run(0, forward - rotation * 0.5);
			bl->Run(0, forward + rotation * 0.5);
			br->Run(0, forward - rotation * 0.5);
		}
		// Finally, the swerve code
		else if (fabs(forward) != 0 || fabs(strafe) != 0 || fabs(rotation) != 0) {
			if (fabs(rotation) != 0) {
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
				fla = fmod((-fla + 360), 360);
			}

			if (front == 0 && right == 0) {
				fra = 0;
			} else {
				fra = atan2(front, right) * 180 / PI;
				fra = fmod((-fra + 360), 360);
			}

			if (back == 0 && left == 0) {
				bla = 0;
			} else {
				bla = atan2(back, left) * 180 / PI;
				bla = fmod((-bla + 360), 360);
			}

			if (back == 0 && right == 0) {
				bra = 0;
			} else {
				bra = atan2(back, right) * 180 / PI;
				bra = fmod((-bra + 360), 360);
			}

			// Sets the angle and speed of the motors
			this->fl->Run(fla, flds * speedMultiplier);
			this->fr->Run(fra, frds * speedMultiplier);
			this->bl->Run(bla, blds * speedMultiplier);
			this->br->Run(bra, brds * speedMultiplier);
		} else {
			// If not pressing joysticks, don't move!
			this->fl->Brake();
			this->fr->Brake();
			this->bl->Brake();
			this->br->Brake();
		}
	}
}

float Swerve::Deadband(float val, float deadband) {
	if (fabs(val) <= deadband) {
		return 0;
	}
	return val;
}

void Swerve::SetRGB(int r, int g, int b) {
	this->redLeds->SetRaw(r);
	this->greenLeds->SetRaw(g);
	this->blueLeds->SetRaw(b);
}

START_ROBOT_CLASS(Swerve);

