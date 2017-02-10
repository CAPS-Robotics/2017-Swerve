#ifndef SwerveModule_H
#define SwerveModule_H

#include <CANTalon.h>
#include "WPILib.h"
#include "Commands/Subsystem.h"

class SwerveModule : public Subsystem {
private:
	Talon * drive;
public:
	CANTalon * steer;
	PIDController * pid;
	AnalogInput * positionEncoder;
	void Drive(double speed, double angle);
	SwerveModule(int steerMotor, int driveMotor, int encoder, bool isInverted);
	void InitDefaultCommand();
	void ReturnToZero();
	void ResetEncoder();
	double GetAngle();
};

#endif  // SwerveModule_H
