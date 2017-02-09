#ifndef DriveWithJoysticks_H
#define DriveWithJoysticks_H

#include "WPILib.h"

class DriveWithJoysticks : public Command {
public:
	DriveWithJoysticks();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	double speed;
};

#endif  // DriveWithJoysticks_H
