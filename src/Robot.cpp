#include <cstdlib>

#include "OI.h"
#include "Robot.h"
#include "WPILib.h"
#include "Commands/Autonomous/TestAuton.h"
#include "Commands/Drivetrain/ReturnWheels.h"
#include <thread>

std::shared_ptr<Drivetrain> Robot::drivetrain;
std::shared_ptr<Climber> Robot::climber;
std::shared_ptr<PigeonNav> Robot::gyro;
std::shared_ptr<OI> Robot::oi;

Robot::Robot() {
	
}

void Robot::RobotInit() {
	Robot::drivetrain.reset(new Drivetrain());
	Robot::climber.reset(new Climber());
	Robot::gyro.reset(new PigeonNav());
	Robot::oi.reset(new OI());
	this->autonomousCommand = new TestAuton();
	std::thread vt(Robot::VisionThread);
	vt.detach();
	Robot::drivetrain->ReturnWheelsToZero();
}

void Robot::DisabledInit() {

}


void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	if (autonomousCommand != NULL) {
		autonomousCommand->Start();
	}
}

void Robot::AutonomousPeriodic() {
	SmartDashboard::PutNumber("FL Angle", Robot::drivetrain->fl->GetAngle());
	SmartDashboard::PutNumber("FR Angle", Robot::drivetrain->fr->GetAngle());
	SmartDashboard::PutNumber("BL Angle", Robot::drivetrain->bl->GetAngle());
	SmartDashboard::PutNumber("BR Angle", Robot::drivetrain->br->GetAngle());
	SmartDashboard::PutNumber("Distance Away", Robot::drivetrain->GetDistanceAway());
	frc::Scheduler::GetInstance()->Run();
}

void Robot::VisionThread() {
	cs::UsbCamera cam = CameraServer::GetInstance()->StartAutomaticCapture();
	cam.SetResolution(320, 240);
	cam.SetBrightness(10);
	cs::CvSink vid = CameraServer::GetInstance()->GetVideo();
	cs::CvSource output = CameraServer::GetInstance()->PutVideo("Contours", 320, 240);
	cs::MjpegServer * ms1 = new cs::MjpegServer("Camera 0", 1181);
	ms1->SetSource(cam);
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
	SmartDashboard::PutNumber("FL Angle", Robot::drivetrain->fl->GetAngle());
	SmartDashboard::PutNumber("FR Angle", Robot::drivetrain->fr->GetAngle());
	SmartDashboard::PutNumber("BL Angle", Robot::drivetrain->bl->GetAngle());
	SmartDashboard::PutNumber("BR Angle", Robot::drivetrain->br->GetAngle());
	SmartDashboard::PutNumber("Distance Away", Robot::drivetrain->GetDistanceAway());
	SmartDashboard::PutNumber("Heading", Robot::gyro->GetHeading());
	SmartDashboard::PutNumber("Angular Rate", Robot::gyro->GetAngularRate());

	frc::Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
	frc::LiveWindow::GetInstance()->Run();
}

START_ROBOT_CLASS(Robot)
