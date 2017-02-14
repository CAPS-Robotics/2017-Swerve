#ifndef Drivetrain_H
#define Drivetrain_H

#include <CANTalon.h>
#include "SwerveModule.h"
#include "WPILib.h"
#include "Commands/Subsystem.h"

class Drivetrain : public Subsystem {
public:
	enum class ControlMode {
		fieldOrientedSwerve,
		swerve,
		arcade
	};
private:
	ControlMode currentMode;
	AnalogInput * rangeFinder;
	double desiredHeading;
public:
	SwerveModule * fl;
	SwerveModule * fr;
	SwerveModule * bl;
	SwerveModule * br;
	Drivetrain();
	void InitDefaultCommand();
	double GetDistanceAway();
	void ReturnWheelsToZero();
	void SetControlMode(Drivetrain::ControlMode cm);
	void Drive(double x, double y, double rotation, double speedMultiplier);
	void CrabDrive(double x, double y, double rotation, double speedMultiplier);
	void ArcadeDrive(double forward, double rotation, double speedMultiplier = 1);
	void Brake();
};

#endif  // Drivetrain_H
