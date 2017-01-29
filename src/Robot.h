/*
 * Robot.h
 *
 *  Created on: Jan 25, 2017
 *      Author: robotics
 */

#include "WPILib.h"
#include <CANTalon.h>

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

int signum(int val) {
	return val == 0 ? 0 : val / fabs(val);
}

class Swerve: public SampleRobot {
	Joystick * joystick;
	SwerveModule * fl;
	SwerveModule * fr;
	SwerveModule * bl;
	SwerveModule * br;
public:
	Swerve();
	void RobotInit();
	void Autonomous();
	void OperatorControl();
	void Disabled() {};
	void Test() {};
	void InitTalon(CANTalon * );
	float Deadband(float);
};



#endif /* SRC_ROBOT_H_ */
