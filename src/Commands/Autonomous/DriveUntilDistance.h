#ifndef DriveUntilDistance_H
#define DriveUntilDistance_H

#include "WPILib.h"

class DriveUntilDistance : public Command {
float distance;
double speed;
public:
	DriveUntilDistance(float distance);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveUntilDistance_H
