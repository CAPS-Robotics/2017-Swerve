#ifndef ZeroGyro_H
#define ZeroGyro_H

#include "WPILib.h"

class ZeroGyro : public Command {
public:
	ZeroGyro();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ZeroGyro_H
