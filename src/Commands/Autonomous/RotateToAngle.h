#ifndef RotateToAngle_H
#define RotateToAngle_H

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

#endif  // RotateToAngle_H
