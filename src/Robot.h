/*
 * Robot.h
 *
 *  Created on: Jan 25, 2017
 *      Author: robotics
 */

#include "WPILib.h"
#include <CANTalon.h>
#include <PigeonImu.h>
#include <thread>

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "SwerveModule.h"

int signum(int val) {
	return val == 0 ? 0 : val / fabs(val);
}

class Swerve: public SampleRobot {
	Joystick * joystick;
	SwerveModule * fl;
	SwerveModule * fr;
	SwerveModule * bl;
	SwerveModule * br;
	PWM * redLeds;
	PWM * greenLeds;
	PWM * blueLeds;
	CANTalon * climber;
	CANTalon * climber2;
	PigeonImu * pigeon;
public:
	Swerve();
	void RobotInit();
	void Autonomous();
	void OperatorControl();
	void Disabled() {};
	void Test() {};
	void InitTalon(CANTalon * );
	float Deadband(float, float);
	void SetRGB(int r, int g, int b);
	static void VisionThread();
};



#endif /* SRC_ROBOT_H_ */
