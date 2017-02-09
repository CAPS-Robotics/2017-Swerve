#ifndef StopClimbing_H
#define StopClimbing_H

#include "Commands/Command.h"

class StopClimbing : public Command {
public:
	StopClimbing();
	void Initialize();
	void Execute();
	void End();
	bool IsFinished();
	void Interrupted();
	float speed;
};

#endif  // StopClimbing_H
