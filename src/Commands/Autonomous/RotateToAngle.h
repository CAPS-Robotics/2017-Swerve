#ifndef DriveUntilDistance_H
#define DriveUntilDistance_H

#include "WPILib.h"

class RotateToAngle : public Command {
float angle;
public:
	RotateToAngle(float angle);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveUntilDistance_H
