#ifndef StrafeAlign_H
#define StrafeAlign_H

#include "WPILib.h"

class StrafeAlign : public Command {
public:
	double pos;
	StrafeAlign();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // StrafeAlign_H
