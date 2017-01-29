#include "WPILib.h"
#include "CANTalon.h"

#ifndef SwerveModule_H_
#define SwerveModule_H_

class SwerveModule {
private:
	Talon * drive;
	CANTalon * steer;
	float speed;
	float angle;
public:
	SwerveModule(int steerMotor, int driveMotor, bool inverted);
	~SwerveModule();
	void Run(float angle, float speed);
	float BestAngle(); 
	void InitTalon(CANTalon *);
	void ResetPosition();
	void Brake();
};

#endif