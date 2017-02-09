#ifndef Climb_H
#define Climb_H

#include "Commands/Command.h"

class Climb : public Command {
public:
	Climb();
	void Initialize();
	void Execute();
	void End();
	bool IsFinished();
	void Interrupted();
};

#endif  // Climb_H
