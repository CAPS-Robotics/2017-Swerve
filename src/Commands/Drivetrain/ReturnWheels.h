#ifndef ReturnWheels_H
#define ReturnWheels_H

#include "WPILib.h"

class ReturnWheels : public Command {
public:
	ReturnWheels();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReturnWheels_H
