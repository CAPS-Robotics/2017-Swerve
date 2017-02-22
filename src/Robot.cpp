#include <cstdlib>

#include "OI.h"
#include "Robot.h"
#include "WPILib.h"
#include "Commands/Autonomous/TestAuton.h"
#include "Commands/Autonomous/LeftStationAuton.h"
#include "Commands/Autonomous/MiddleStationAuton.h"
#include "Commands/Autonomous/RightStationAuton.h"
#include <thread>

std::shared_ptr<Drivetrain> Robot::drivetrain;
std::shared_ptr<Climber> Robot::climber;
std::shared_ptr<Vision> Robot::vision;
std::shared_ptr<PigeonNav> Robot::gyro;
std::shared_ptr<OI> Robot::oi;

Robot::Robot() {
	
}

void Robot::RobotInit() {
	Robot::drivetrain.reset(new Drivetrain());
	Robot::climber.reset(new Climber());
	Robot::gyro.reset(new PigeonNav());
	Robot::vision.reset(new Vision());
	Robot::oi.reset(new OI());
	std::thread vt(Robot::VisionThread);
	vt.detach();

	this->autoPicker = new SendableChooser<Command *>();
	this->autoPicker->AddDefault("Middle Station Auton", new MiddleStationAuton());
	this->autoPicker->AddObject("Left Station Auton", new LeftStationAuton());
	this->autoPicker->AddObject("Right Station Auton", new RightStationAuton());
	this->autoPicker->AddObject("Test Auton", new TestAuton());
	SmartDashboard::PutData("Auto Picker", this->autoPicker);
}

void Robot::DisabledInit() {
	autonomousCommand = nullptr;
}


void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	autonomousCommand = (Command *) autoPicker->GetSelected();
	autonomousCommand->Start();
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
	if (autonomousCommand != nullptr) {
		autonomousCommand->Cancel();
	}
}

void Robot::TeleopPeriodic() {
	SmartDashboard::PutNumber("FL Voltage", 	Robot::drivetrain->fl->positionEncoder->GetVoltage());
	SmartDashboard::PutNumber("FR Voltage", 	Robot::drivetrain->fr->positionEncoder->GetVoltage());
	SmartDashboard::PutNumber("BL Voltage", 	Robot::drivetrain->bl->positionEncoder->GetVoltage());
	SmartDashboard::PutNumber("BR Voltage", 	Robot::drivetrain->br->positionEncoder->GetVoltage());
	SmartDashboard::PutNumber("FL Angle", 		Robot::drivetrain->fl->GetAngle());
	SmartDashboard::PutNumber("FR Angle", 		Robot::drivetrain->fr->GetAngle());
	SmartDashboard::PutNumber("BL Angle", 		Robot::drivetrain->bl->GetAngle());
	SmartDashboard::PutNumber("BR Angle", 		Robot::drivetrain->br->GetAngle());
	SmartDashboard::PutNumber("Distance Away", 	Robot::drivetrain->GetDistanceAway());
	SmartDashboard::PutNumber("Heading", 		Robot::gyro->GetHeading());
	SmartDashboard::PutNumber("Angular Rate", 	Robot::gyro->GetAngularRate());
	SmartDashboard::PutNumber("Center X", 		Robot::vision->GetCentralValue());

	frc::Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
	frc::LiveWindow::GetInstance()->Run();
}

START_ROBOT_CLASS(Robot)
