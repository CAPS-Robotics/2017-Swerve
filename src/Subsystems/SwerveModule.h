#ifndef SwerveModule_H
#define SwerveModule_H

#include <CANTalon.h>
#include "WPILib.h"
#include "Commands/Subsystem.h"

class SwerveModule : public Subsystem {
private:
	Talon * drive;
	CANTalon * steer;
	double currentSpeed;
public:
	void Drive(double speed, double angle);
	SwerveModule(int steerMotor, int driveMotor, bool isInverted);
	void InitDefaultCommand();
	void ReturnToZero();
	void ResetEncoder();
	double GetAngle();
};

#endif  // SwerveModule_H
