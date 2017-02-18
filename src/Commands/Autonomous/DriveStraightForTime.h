#ifndef DriveStraightForTime_H
#define DriveStraightForTime_H

#include "Commands/TimedCommand.h"

class DriveStraightForTime : public TimedCommand {
public:
	DriveStraightForTime(double timeout);
	void Initialize();
	void Execute();
	void End();
	void Interrupted();
};

#endif  // DriveStraightForTime_H
