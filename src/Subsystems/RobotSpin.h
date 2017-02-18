#ifndef RobotSpin_H
#define RobotSpin_H

#include "WPILib.h"

class RobotSpin : public PIDOutput {
private:
	double output;
public:
	RobotSpin();
	void PIDWrite(double output);
	double Get();
};

#endif  // RobotSpin_H
