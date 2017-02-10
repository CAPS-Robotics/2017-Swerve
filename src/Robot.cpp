#include <cstdlib>

#include "OI.h"
#include "Robot.h"
#include "WPILib.h"

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
}

void Robot::DisabledInit() {

}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
	SmartDashboard::PutNumber("P", 1);
	SmartDashboard::PutNumber("I", 0);
	SmartDashboard::PutNumber("D", 0);
	SmartDashboard::PutNumber("Setpoint", 0);
}

void Robot::TeleopPeriodic() {
	SmartDashboard::PutNumber("Distance Away", Robot::drivetrain->GetDistanceAway());
	SmartDashboard::PutNumber("Heading", Robot::gyro->GetHeading());
	double p = SmartDashboard::GetNumber("P", 1);
	double i = SmartDashboard::GetNumber("I", 0);
	double d = SmartDashboard::GetNumber("D", 0);
	double setpoint = SmartDashboard::GetNumber("Setpoint", 0);
	SmartDashboard::PutNumber("FL Voltage", Robot::drivetrain->fl->positionEncoder->GetVoltage());
	SmartDashboard::PutNumber("FL Angle", Robot::drivetrain->fl->GetAngle());
	SmartDashboard::PutNumber("FR Voltage", Robot::drivetrain->fr->positionEncoder->GetVoltage());
	SmartDashboard::PutNumber("FR Angle", Robot::drivetrain->fr->GetAngle());
	SmartDashboard::PutNumber("BL Voltage", Robot::drivetrain->bl->positionEncoder->GetVoltage());
	SmartDashboard::PutNumber("BL Angle", Robot::drivetrain->bl->GetAngle());
	SmartDashboard::PutNumber("BR Voltage", Robot::drivetrain->br->positionEncoder->GetVoltage());
	SmartDashboard::PutNumber("BR Angle", Robot::drivetrain->br->GetAngle());
	if (Robot::oi->joy1->GetRawButton(3)) {
		Robot::drivetrain->bl->steer->Set(0.5);
	} else {
		Robot::drivetrain->bl->steer->Set(0);
	}
	if (Robot::oi->joy1->GetRawButton(5)) {
		Robot::drivetrain->fl->steer->Set(0.5);
	} else {
		Robot::drivetrain->fl->steer->Set(0);
	}
	if (Robot::oi->joy1->GetRawButton(4)) {
		Robot::drivetrain->br->steer->Set(0.5);
	} else {
		Robot::drivetrain->br->steer->Set(0);
	}
	if (Robot::oi->joy1->GetRawButton(6)) {
		Robot::drivetrain->fr->steer->Set(0.5);
	} else {
		Robot::drivetrain->fr->steer->Set(0);
	}
	//Robot::drivetrain->bl->pid->SetPID(p, i, d);
	//Robot::drivetrain->bl->pid->SetSetpoint(setpoint);
	//Robot::drivetrain->fr->pid->Enable();
	//frc::Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
	frc::LiveWindow::GetInstance()->Run();
}

START_ROBOT_CLASS(Robot)
