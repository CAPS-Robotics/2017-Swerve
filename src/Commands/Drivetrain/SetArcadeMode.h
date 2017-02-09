#ifndef SetArcadeMode_H
#define SetArcadeMode_H

#include "WPILib.h"

class SetArcadeMode : public Command {
public:
	SetArcadeMode();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // SetArcadeMode_H
