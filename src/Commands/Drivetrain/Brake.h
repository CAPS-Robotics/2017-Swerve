#ifndef Brake_H
#define Brake_H

#include "WPILib.h"

class Brake : public Command {
public:
	Brake();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Brake_H
